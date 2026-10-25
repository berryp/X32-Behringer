/*
 * X32USBTransport.c
 *
 * Network side of the X32 virtual audio driver, see X32USBTransport.h
 *
 * Input path (emulator -> computer): the receiver thread maps the emulator's
 * stream positions onto the driver's sample time line ("base"), so that frames
 * land in the ring at about the time they arrive; the IO thread reads them
 * XUT_SAFETY_FRAMES later. If packets arrive too late, too early (clock
 * drift between machines) or the emulator restarts its stream, the mapping is
 * reset, costing one small glitch.
 *
 * Output path (computer -> emulator): frames are sent as they are mixed by the
 * HAL; the emulator keeps them in its own jitter buffer.
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "X32USBTransport.h"

#ifdef __APPLE__
#include <mach/mach_time.h>
#include <pthread/qos.h>
#else
#include <time.h>
#endif

#define HELLO_INTERVAL	0.25	// seconds between HELLOs
#define EMU_TIMEOUT		2.0		// seconds without news before the emulator is considered gone
#define MAX_LEAD		512		// frames: data mapped further ahead of "now" (extra latency)...
#define MAX_LAG			(XUT_SAFETY_FRAMES * 3 / 4)	// ...or later than this (dropouts)...
#define MAX_BAD			16		// ...for this many packets in a row: resync (a single late packet is a hiccup)

uint64_t XUT_host_time(void) {
#ifdef __APPLE__
	return mach_absolute_time();
#else
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
#endif
}

double XUT_host_ticks_per_second(void) {
#ifdef __APPLE__
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	return 1.0e9 * (double)tb.denom / (double)tb.numer;
#else
	return 1.0e9;
#endif
}

// driver sample time now
static int64_t now_sample(XUT* t) {
	uint64_t anchor = atomic_load(&t->anchor);
	uint64_t now = XUT_host_time();
	if (now < anchor) return 0;
	return (int64_t)((double)(now - anchor) * atomic_load(&t->rate) / t->ticks_per_second);
}

static void send_hello(XUT* t) {
	unsigned char pkt[X32A_HEADER_SIZE];
	X32A_header h;
	memset(&h, 0, sizeof(h));
	h.type = X32A_HELLO;
	h.rate = atomic_load(&t->rate);
	h.epoch = atomic_load(&t->epoch);
	h.flags = atomic_load(&t->running) ? X32A_FLAG_RUNNING : 0;
	X32A_write_header(pkt, &h);
	send(t->fd, pkt, sizeof(pkt), MSG_DONTWAIT);
}

static void zero_frames(XUT* t, int64_t from, int64_t to) {
	if (to - from > XUT_RING_FRAMES) from = to - XUT_RING_FRAMES;
	for (; from < to; from++) memset(t->ring[from & (XUT_RING_FRAMES - 1)], 0, sizeof(t->ring[0]));
}

// audio from the emulator (receiver thread)
static void receive_audio(XUT* t, const unsigned char* pkt, const X32A_header* h) {
	int64_t now, dpos, wpos, lead;
	uint32_t io_epoch = atomic_load(&t->epoch);
	int i, c, nc;
	const unsigned char* p = pkt + X32A_HEADER_SIZE;

	if (!atomic_load(&t->running)) return;
	if (h->rate != atomic_load(&t->rate)) return;		// rate change pending
	now = now_sample(t);
	wpos = atomic_load(&t->in_wpos);
	if (io_epoch != t->in_io_epoch) {					// IO restarted: sample time restarted
		t->in_io_epoch = io_epoch;
		t->in_synced = 0;
		wpos = 0;
	}
	if (t->in_synced && h->epoch == t->in_epoch && h->position < t->in_expect) {
		if (h->position + h->frames <= t->in_expect) return;	// late or duplicate
		t->in_synced = 0;
	}
	if (t->in_synced && h->epoch == t->in_epoch) {
		dpos = (int64_t)h->position + t->in_base;
		lead = dpos - now;
		if (lead > MAX_LEAD || lead < -MAX_LAG) {
			if (++t->in_bad >= MAX_BAD || lead > XUT_RING_FRAMES / 4 || lead < -XUT_RING_FRAMES / 4) t->in_synced = 0;
		} else {
			t->in_bad = 0;
		}
	}
	if (!t->in_synced || h->epoch != t->in_epoch) {
		atomic_fetch_add(&t->in_resyncs, 1);
		t->in_synced = 1;
		t->in_bad = 0;
		t->in_epoch = h->epoch;
		t->in_base = now - (int64_t)h->position;
	}
	dpos = (int64_t)h->position + t->in_base;
	if (dpos > wpos) zero_frames(t, wpos, dpos);		// lost packets or resync forward
	nc = h->channels;
	for (i = 0; i < h->frames; i++) {
		float* f = t->ring[(dpos + i) & (XUT_RING_FRAMES - 1)];
		for (c = 0; c < nc; c++, p += 4) f[c] = X32A_getf(p);
		for (; c < XUT_CHANNELS; c++) f[c] = 0.0f;
	}
	t->in_expect = h->position + h->frames;
	atomic_store_explicit(&t->in_wpos, dpos + h->frames, memory_order_release);
}

static void receive_status(XUT* t, const unsigned char* pkt, const X32A_header* h) {
	const unsigned char* p = pkt + X32A_HEADER_SIZE;
	uint32_t rate = h->rate;

	atomic_store(&t->emu_to_computer, X32A_get16(p));
	atomic_store(&t->emu_from_computer, X32A_get16(p + 2));
	memcpy(t->emu_name, p + 4, sizeof(t->emu_name) - 1);
	if (rate != atomic_load(&t->emu_rate)) {
		atomic_store(&t->emu_rate, rate);
	}
	// the emulator is the clock master: follow its rate
	if ((rate == 44100 || rate == 48000) && rate != atomic_load(&t->rate) && rate != atomic_load(&t->requested_rate)) {
		atomic_store(&t->requested_rate, rate);
		if (t->rate_cb) t->rate_cb(t->rate_ctx, rate);
	}
}

static void* receiver(void* arg) {
	XUT* t = (XUT*)arg;
	unsigned char pkt[X32A_MAX_PACKET + 64];
	struct pollfd pfd;
	uint64_t last_hello = 0, hello_ticks = (uint64_t)(HELLO_INTERVAL * t->ticks_per_second);
	X32A_header h;
	int len;

#ifdef __APPLE__
	pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);	// audio arrives here
#endif
	while (!atomic_load(&t->quit)) {
		uint64_t now = XUT_host_time();
		if (now - last_hello >= hello_ticks) {
			send_hello(t);
			last_hello = now;
		}
		pfd.fd = t->fd;
		pfd.events = POLLIN;
		pfd.revents = 0;
		if (poll(&pfd, 1, 50) <= 0) continue;
		while ((len = recv(t->fd, pkt, sizeof(pkt), MSG_DONTWAIT)) > 0) {
			if (X32A_read_header(pkt, len, &h)) continue;
			atomic_store(&t->emu_seen, XUT_host_time());
			if (h.type == X32A_STATUS) receive_status(t, pkt, &h);
			else if (h.type == X32A_AUDIO) receive_audio(t, pkt, &h);
		}
	}
	return NULL;
}

int XUT_init(XUT* t, const char* host, int port, XUT_rate_cb cb, void* ctx) {
	struct addrinfo hints, *res, *rp;
	char sport[16];

	memset(t, 0, sizeof(XUT));
	t->fd = -1;
	t->rate_cb = cb;
	t->rate_ctx = ctx;
	t->ticks_per_second = XUT_host_ticks_per_second();
	atomic_store(&t->rate, 48000);
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_protocol = IPPROTO_UDP;
	snprintf(sport, sizeof(sport), "%d", port);
	if (getaddrinfo(host, sport, &hints, &res) != 0) return -1;
	for (rp = res; rp != NULL; rp = rp->ai_next) {
		if ((t->fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol)) < 0) continue;
		if (connect(t->fd, rp->ai_addr, rp->ai_addrlen) == 0) break;
		close(t->fd);
		t->fd = -1;
	}
	freeaddrinfo(res);
	if (t->fd < 0) return -1;
	{
		int sz = 1 << 20;	// room for bursts of audio
		setsockopt(t->fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof(sz));
		setsockopt(t->fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
	}
	if (pthread_create(&t->thread, NULL, receiver, t)) {
		close(t->fd);
		t->fd = -1;
		return -1;
	}
	return 0;
}

void XUT_shutdown(XUT* t) {
	if (t->fd < 0) return;
	atomic_store(&t->quit, 1);
	pthread_join(t->thread, NULL);
	close(t->fd);
	t->fd = -1;
}

void XUT_start(XUT* t, uint64_t anchor_host_time, uint32_t rate) {
	atomic_store(&t->in_wpos, 0);
	atomic_store(&t->anchor, anchor_host_time);
	atomic_store(&t->rate, rate);
	atomic_fetch_add(&t->epoch, 1);
	atomic_store(&t->running, 1);
	send_hello(t);			// get the emulator streaming right away
}

void XUT_stop(XUT* t) {
	atomic_store(&t->running, 0);
	send_hello(t);
}

void XUT_set_rate(XUT* t, uint32_t rate) {
	atomic_store(&t->rate, rate);
	atomic_store(&t->requested_rate, 0);	// a later mismatch asks for a change again
}

int XUT_connected(XUT* t) {
	uint64_t seen = atomic_load(&t->emu_seen);
	return seen && (double)(XUT_host_time() - seen) < EMU_TIMEOUT * t->ticks_per_second;
}

uint32_t XUT_emulator_rate(XUT* t) {
	return XUT_connected(t) ? atomic_load(&t->emu_rate) : 0;
}

void XUT_read_input(XUT* t, int64_t sample_time, float* dst, uint32_t frames) {
	int64_t wpos = atomic_load_explicit(&t->in_wpos, memory_order_acquire);
	int64_t oldest = wpos - (XUT_RING_FRAMES - XUT_SAFETY_FRAMES);
	uint32_t i;

	for (i = 0; i < frames; i++, dst += XUT_CHANNELS) {
		int64_t s = sample_time + i;
		if (s < wpos && s >= oldest && s >= 0) memcpy(dst, t->ring[s & (XUT_RING_FRAMES - 1)], sizeof(t->ring[0]));
		else memset(dst, 0, sizeof(t->ring[0]));
	}
}

void XUT_write_output(XUT* t, int64_t sample_time, const float* src, uint32_t frames) {
	X32A_header h;
	unsigned char* p;
	uint32_t done = 0, n, i;

	memset(&h, 0, sizeof(h));
	h.type = X32A_AUDIO;
	h.rate = atomic_load(&t->rate);
	h.channels = XUT_CHANNELS;
	h.epoch = atomic_load(&t->epoch);
	h.flags = frames;					// IO burst size, sizes the emulator's jitter buffer
	while (done < frames) {
		n = frames - done;
		if (n > X32A_PACKET_FRAMES) n = X32A_PACKET_FRAMES;
		h.frames = (uint16_t)n;
		h.position = (uint64_t)(sample_time + done);
		X32A_write_header(t->out_pkt, &h);
		p = t->out_pkt + X32A_HEADER_SIZE;
		for (i = 0; i < n * XUT_CHANNELS; i++, p += 4) X32A_putf(p, src[(done * XUT_CHANNELS) + i]);
		send(t->fd, t->out_pkt, p - t->out_pkt, MSG_DONTWAIT);
		done += n;
	}
}
