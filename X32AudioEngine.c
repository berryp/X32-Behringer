/*
 * X32AudioEngine.c
 *
 * Audio engine for the X32 emulator, giving it an X-USB expansion card: the
 * X32 virtual audio driver for macOS (X32USBDriver/) connects to the engine
 * over UDP and shows up on the computer as a 32 in / 32 out audio interface,
 * as a real X32 with an X-USB (or X-LIVE) card does.
 *
 * Signal flow (a simplified X32: no EQ, dynamics, FX, aux or matrix processing):
 *
 *   local inputs AN1-32 (silence or test signals, see -s option of X32)
 *   card inputs CARD1-32 (audio played by the computer)
 *        |
 *        +--> /config/routing/IN (or PLAY) --> inputs 1-32
 *                --> /ch/xx/config/source --> channel: trim, invert, fader, mute, DCA, mute groups
 *                       --> pan --> main LR,  mlevel --> M/C,  sends --> mix buses 1-16
 *                                   mix buses: fader, mute --> main LR / M/C
 *        /outputs/main/01..16/src (Main L/R, M/C, mix buses, channel direct outs)
 *        |
 *        +--> /config/routing/CARD --> card outputs 1-32 (recorded by the computer)
 *
 * The engine is the clock master: it runs at /-prefs/clockrate and generates
 * a 64 frame block every 1.33ms (48kHz). Audio received from the computer is
 * held in a small jitter buffer.
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */
#ifndef __WIN32__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#ifdef __APPLE__
#include <pthread/qos.h>
#endif

#include "X32AudioEngine.h"
#include "X32AudioProtocol.h"

#define BLOCK		X32A_PACKET_FRAMES	// frames per engine block
#define NCARD		X32A_MAX_CHANNELS	// card channels
#define CIN_RING	32768				// card input jitter buffer (frames)
#define CLIENT_TIMEOUT 2000000000LL		// ns without HELLO before dropping the driver
#define METER_FALL	26.0				// meter peak fall-back, dB/s

static X32AudioParams	P;
static int				Xa_fd = -1;
static int				Xa_signal;
static int				Xa_verbose;
static volatile int		Xa_quit;
static pthread_t		Xa_thread;
//
// connected driver
static struct sockaddr_in Xa_client;
static int				Xa_has_client;
static int				Xa_client_running;
static long long		Xa_client_seen;		// ns
//
// stream state
static unsigned int		Xa_rate;
static unsigned int		Xa_epoch;
static unsigned long long Xa_outpos;		// card output stream position
static int				Xa_to_computer = NCARD, Xa_from_computer = NCARD;
//
// card input (computer -> X32) jitter buffer
static float			Cin[CIN_RING][NCARD];
static long long		Cin_wpos, Cin_rpos;
static unsigned int		Cin_epoch;
static int				Cin_synced, Cin_primed;
static int				Cin_burst = BLOCK;
static int				Cin_underruns, Cin_lost, Cin_late;
//
// signals, one block each
static float			S_local[NCARD][BLOCK];
static float			S_card[NCARD][BLOCK];
static float			S_zero[BLOCK];
static float			S_chin[X32AE_NCH][BLOCK];	// channel, pre-fader
static float			S_chpost[X32AE_NCH][BLOCK];	// channel, post-fader & mute
static float			S_bus[X32AE_NBUS][BLOCK];	// bus master outputs
static float			S_main[2][BLOCK];
static float			S_mono[BLOCK];
static float			S_out[X32AE_NOUT][BLOCK];
static float			S_cout[NCARD][BLOCK];		// card outputs
//
// gains used for the previous block (ramped to avoid zipper noise)
static float			G_ch[X32AE_NCH], G_chL[X32AE_NCH], G_chR[X32AE_NCH], G_chM[X32AE_NCH];
static float			G_send[X32AE_NCH][X32AE_NBUS];
static float			G_bus[X32AE_NBUS], G_busL[X32AE_NBUS], G_busR[X32AE_NBUS], G_busM[X32AE_NBUS];
static float			G_main[2], G_mono;
static int				Ch_muted[X32AE_NCH];
static double			Phase[NCARD];
static unsigned int		Noise = 0x12345678;
//
// meters (linear peak, 1.0 = 0dBFS)
static float			M_chin[X32AE_NCH], M_bus[X32AE_NBUS], M_main[2], M_mono;
static float			M_fall;

//
// time in ns from a monotonic clock
static long long now_ns() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

#define IVAL(p, d)	((p) ? *(volatile int*)(p) : (d))
#define FVAL(p, d)	((p) ? *(volatile float*)(p) : (d))

//
// X32 fader law: float [0..1] to linear gain (-oo, -90dB...+10dB)
static float fader2gain(float f) {
	float db;
	if (f <= 0.0f) return 0.0f;
	if (f > 1.0f) f = 1.0f;
	if (f >= 0.5f) db = f * 40.0f - 30.0f;
	else if (f >= 0.25f) db = f * 80.0f - 50.0f;
	else if (f >= 0.0625f) db = f * 160.0f - 70.0f;
	else db = f * 480.0f - 90.0f;
	return powf(10.0f, db / 20.0f);
}

//
// constant power pan, [0..1], 0.5 is center
static void pan2gains(float pan, float* l, float* r) {
	if (pan < 0.0f) pan = 0.0f;
	if (pan > 1.0f) pan = 1.0f;
	*l = cosf(pan * (float)M_PI_2);
	*r = sinf(pan * (float)M_PI_2);
}

//
// dst += src * gain ramped from g0 to g1
static void mixramp(float* dst, const float* src, float g0, float g1) {
	int i;
	float g, dg;
	if (g0 == 0.0f && g1 == 0.0f) return;
	dg = (g1 - g0) / BLOCK;
	for (i = 0, g = g0; i < BLOCK; i++, g += dg) dst[i] += src[i] * g;
}

static float peak(const float* s) {
	int i;
	float p = 0.0f;
	for (i = 0; i < BLOCK; i++) if (fabsf(s[i]) > p) p = fabsf(s[i]);
	return p;
}

static void meter(float* m, const float* s) {
	float p = peak(s);
	*m = (p > *m * M_fall) ? p : *m * M_fall;
	if (*m < 1e-6f) *m = 0.0f;		// below -120dB: off (and no denormals)
}

//
// signal for a routing block entry: XRtgin/XRtaea enum values
// 0-3: AN1-8..AN25-32, 4-15: AES50 A/B (nothing connected), 16-19: CARD1-8..25-32,
// 20-21: OUT1-8, OUT9-16 (card output routing only), others: not emulated
static const float* block_signal(int blk, int k) {
	if (blk >= 0 && blk <= 3) return S_local[blk * 8 + k];
	if (blk >= 16 && blk <= 19) return S_card[(blk - 16) * 8 + k];
	if (blk == 20 || blk == 21) return S_out[(blk - 20) * 8 + k];
	return S_zero;
}

//
// signal feeding input n (0..31): through /config/routing/IN or PLAY
static const float* input_signal(int n) {
	int** rt = (IVAL(P.routswitch, 0) == 1) ? P.rt_play : P.rt_in;
	return block_signal(IVAL(rt[n / 8], n / 8), n % 8);
}

//
// local inputs: what's "plugged" in the emulator
static void gen_local() {
	int c, i;
	double inc;
	if (Xa_signal == X32AE_SIG_TONES) {
		for (c = 0; c < NCARD; c++) {
			inc = 2.0 * M_PI * 100.0 * (c + 1) / Xa_rate;
			for (i = 0; i < BLOCK; i++) {
				S_local[c][i] = 0.1f * (float)sin(Phase[c]);
				Phase[c] += inc;
			}
			Phase[c] = fmod(Phase[c], 2.0 * M_PI);
		}
	} else if (Xa_signal == X32AE_SIG_NOISE) {
		for (c = 0; c < NCARD; c++) {
			for (i = 0; i < BLOCK; i++) {
				Noise ^= Noise << 13; Noise ^= Noise >> 17; Noise ^= Noise << 5;
				S_local[c][i] = 0.0316f * ((float)Noise / 2147483648.0f - 1.0f);
			}
		}
	} else {
		memset(S_local, 0, sizeof(S_local));
	}
}

//
// card inputs: one block out of the jitter buffer
static void get_card() {
	int i, target;
	long long avail;

	memset(S_card, 0, sizeof(S_card));
	if (!Cin_synced) return;
	target = Cin_burst + 4 * BLOCK;
	avail = Cin_wpos - Cin_rpos;
	if (avail > target + CIN_RING / 2) {		// too much latency (drift), skip ahead
		Cin_rpos = Cin_wpos - target;
		avail = target;
	}
	if (!Cin_primed) {
		if (avail < target) return;
		Cin_primed = 1;
	}
	if (avail < BLOCK) {						// underrun: computer stopped or late
		Cin_primed = 0;
		Cin_underruns++;
		return;
	}
	for (i = 0; i < BLOCK; i++) {
		int c;
		float* f = Cin[(Cin_rpos + i) & (CIN_RING - 1)];
		for (c = 0; c < Xa_from_computer; c++) S_card[c][i] = f[c];
	}
	Cin_rpos += BLOCK;
}

//
// the mixer
static void process() {
	int c, b, i, k, src, mg, muted, mask;
	float g, gl, gr, gm, dcag;
	float raw[BLOCK];

	gen_local();
	get_card();
	memset(S_bus, 0, sizeof(S_bus));
	memset(S_main, 0, sizeof(S_main));
	memset(S_mono, 0, sizeof(S_mono));
	//
	// channels
	for (c = 0; c < X32AE_NCH; c++) {
		src = IVAL(P.ch[c].source, c + 1);	// 0: OFF, 1-32: In01-32, others not emulated
		if (src >= 1 && src <= 32) memcpy(raw, input_signal(src - 1), sizeof(raw));
		else memset(raw, 0, sizeof(raw));
		// trim: [0..1] -> -18..+18dB
		g = powf(10.0f, (FVAL(P.ch[c].trim, 0.5f) * 36.0f - 18.0f) / 20.0f);
		if (IVAL(P.ch[c].invert, 0)) g = -g;
		for (i = 0; i < BLOCK; i++) S_chin[c][i] = raw[i] * g;
		meter(&M_chin[c], S_chin[c]);
		//
		// mute: channel on, mute groups, DCA
		muted = !IVAL(P.ch[c].on, 1);
		mask = IVAL(P.ch[c].mute, 0);
		for (mg = 0; mg < X32AE_NMGRP; mg++)
			if ((mask & (1 << mg)) && IVAL(P.mute_grp[mg], 0)) muted = 1;
		dcag = 1.0f;
		mask = IVAL(P.ch[c].dca, 0);
		for (k = 0; k < X32AE_NDCA; k++) {
			if (mask & (1 << k)) {
				if (!IVAL(P.dca_on[k], 1)) muted = 1;
				dcag *= fader2gain(FVAL(P.dca_fader[k], 0.75f)) ;
			}
		}
		Ch_muted[c] = muted;
		g = muted ? 0.0f : fader2gain(FVAL(P.ch[c].fader, 0.0f)) * dcag;
		memset(S_chpost[c], 0, sizeof(S_chpost[c]));
		mixramp(S_chpost[c], S_chin[c], G_ch[c], g);
		G_ch[c] = g;
		//
		// to main LR, M/C
		gl = gr = 0.0f;
		if (IVAL(P.ch[c].st, 1)) pan2gains(FVAL(P.ch[c].pan, 0.5f), &gl, &gr);
		mixramp(S_main[0], S_chpost[c], G_chL[c], gl);
		mixramp(S_main[1], S_chpost[c], G_chR[c], gr);
		G_chL[c] = gl, G_chR[c] = gr;
		gm = IVAL(P.ch[c].mono, 0) ? fader2gain(FVAL(P.ch[c].mlevel, 0.0f)) : 0.0f;
		mixramp(S_mono, S_chpost[c], G_chM[c], gm);
		G_chM[c] = gm;
		//
		// mix bus sends: types IN/LC, <-EQ, EQ->, PRE are pre-fader, POST and GRP post-fader;
		// all sends follow the channel mute
		for (b = 0; b < X32AE_NBUS; b++) {
			g = 0.0f;
			if (!muted && IVAL(P.ch[c].send_on[b], 1)) {
				g = fader2gain(FVAL(P.ch[c].send_level[b], 0.0f));
				if (IVAL(P.ch[c].send_type[b], 4) >= 4) g *= G_ch[c];
			}
			mixramp(S_bus[b], S_chin[c], G_send[c][b], g);
			G_send[c][b] = g;
		}
	}
	//
	// mix buses
	for (b = 0; b < X32AE_NBUS; b++) {
		memcpy(raw, S_bus[b], sizeof(raw));
		memset(S_bus[b], 0, sizeof(S_bus[b]));
		g = IVAL(P.bus[b].on, 1) ? fader2gain(FVAL(P.bus[b].fader, 0.0f)) : 0.0f;
		mixramp(S_bus[b], raw, G_bus[b], g);
		G_bus[b] = g;
		meter(&M_bus[b], S_bus[b]);
		gl = gr = 0.0f;
		if (IVAL(P.bus[b].st, 0)) pan2gains(FVAL(P.bus[b].pan, 0.5f), &gl, &gr);
		mixramp(S_main[0], S_bus[b], G_busL[b], gl);
		mixramp(S_main[1], S_bus[b], G_busR[b], gr);
		G_busL[b] = gl, G_busR[b] = gr;
		gm = IVAL(P.bus[b].mono, 0) ? fader2gain(FVAL(P.bus[b].mlevel, 0.0f)) : 0.0f;
		mixramp(S_mono, S_bus[b], G_busM[b], gm);
		G_busM[b] = gm;
	}
	//
	// main LR (balance from /main/st/mix/pan) and M/C
	g = IVAL(P.main_on, 1) ? fader2gain(FVAL(P.main_fader, 0.0f)) : 0.0f;
	gl = gr = g;
	k = (int)(FVAL(P.main_pan, 0.5f) * 100.0f + 0.5f);
	if (k < 50) gr *= k / 50.0f;
	if (k > 50) gl *= (100 - k) / 50.0f;
	for (i = 0; i < 2; i++) {
		memcpy(raw, S_main[i], sizeof(raw));
		memset(S_main[i], 0, sizeof(S_main[i]));
		mixramp(S_main[i], raw, G_main[i], i ? gr : gl);
		meter(&M_main[i], S_main[i]);
	}
	G_main[0] = gl, G_main[1] = gr;
	memcpy(raw, S_mono, sizeof(raw));
	memset(S_mono, 0, sizeof(S_mono));
	g = IVAL(P.mono_on, 1) ? fader2gain(FVAL(P.mono_fader, 0.0f)) : 0.0f;
	mixramp(S_mono, raw, G_mono, g);
	G_mono = g;
	meter(&M_mono, S_mono);
	//
	// outputs: /outputs/main/NN/src
	// 0: OFF, 1-2: Main L/R, 3: M/C, 4-19: MixBus 1-16, 20-25: Matrix 1-6,
	// 26-57: DirectOut Ch 1-32, 58-65: DirectOut Aux, 66-73: DirectOut FX, 74-75: Monitor, 76: Talkback
	for (k = 0; k < X32AE_NOUT; k++) {
		const float* s = S_zero;
		src = IVAL(P.out_src[k], 0);
		if (src == 1 || src == 2) s = S_main[src - 1];
		else if (src == 3) s = S_mono;
		else if (src >= 4 && src <= 19) s = S_bus[src - 4];
		else if (src >= 26 && src <= 57) {
			// tap: IN/LC(+M), <-EQ(+M), EQ->(+M), PRE(+M): pre-fader, POST: post-fader
			c = src - 26;
			i = IVAL(P.out_pos[k], 8);
			if (i >= 8) s = S_chpost[c];
			else if ((i & 1) && Ch_muted[c]) s = S_zero;
			else s = S_chin[c];
		}
		memcpy(S_out[k], s, sizeof(S_out[k]));
	}
	//
	// card outputs: /config/routing/CARD
	for (c = 0; c < NCARD; c++) {
		if (c < Xa_to_computer) memcpy(S_cout[c], block_signal(IVAL(P.rt_card[c / 8], c / 8), c % 8), sizeof(S_cout[c]));
		else memset(S_cout[c], 0, sizeof(S_cout[c]));
	}
}

//
// card (USB) modes, /-prefs/card/USBmode: X32 -> computer / computer -> X32 channels
static void card_mode() {
	static const int to[] = {32, 16, 32, 8, 8, 2};
	static const int from[] = {32, 16, 8, 32, 8, 2};
	int m = IVAL(P.usbmode, 0);
	if (m < 0 || m > 5) m = 0;
	Xa_to_computer = to[m];
	Xa_from_computer = from[m];
}

static void send_status() {
	unsigned char pkt[X32A_HEADER_SIZE + X32A_STATUS_SIZE];
	X32A_header h;
	memset(&h, 0, sizeof(h));
	memset(pkt, 0, sizeof(pkt));
	h.type = X32A_STATUS;
	h.rate = Xa_rate;
	h.epoch = Xa_epoch;
	X32A_write_header(pkt, &h);
	X32A_put16(pkt + X32A_HEADER_SIZE, (uint16_t)Xa_to_computer);
	X32A_put16(pkt + X32A_HEADER_SIZE + 2, (uint16_t)Xa_from_computer);
	if (P.name && *P.name) strncpy((char*)pkt + X32A_HEADER_SIZE + 4, *P.name, 27);
	sendto(Xa_fd, pkt, sizeof(pkt), 0, (struct sockaddr*)&Xa_client, sizeof(Xa_client));
}

static void send_block() {
	unsigned char pkt[X32A_MAX_PACKET];
	unsigned char* p;
	X32A_header h;
	int c, i;

	memset(&h, 0, sizeof(h));
	h.type = X32A_AUDIO;
	h.rate = Xa_rate;
	h.channels = NCARD;
	h.frames = BLOCK;
	h.position = Xa_outpos;
	h.epoch = Xa_epoch;
	h.flags = BLOCK;
	X32A_write_header(pkt, &h);
	p = pkt + X32A_HEADER_SIZE;
	for (i = 0; i < BLOCK; i++)
		for (c = 0; c < NCARD; c++, p += 4) X32A_putf(p, S_cout[c][i]);
	sendto(Xa_fd, pkt, p - pkt, 0, (struct sockaddr*)&Xa_client, sizeof(Xa_client));
}

//
// audio from the computer: into the jitter buffer
static void receive_audio(const unsigned char* pkt, X32A_header* h) {
	int i, c, nc;
	long long pos = (long long)h->position;
	const unsigned char* p = pkt + X32A_HEADER_SIZE;

	if (h->rate != Xa_rate) return;		// computer runs at another rate, wait for driver to follow
	if (!Cin_synced || h->epoch != Cin_epoch || pos + h->frames <= Cin_wpos - CIN_RING / 2 || pos > Cin_wpos + CIN_RING / 2) {
		Cin_synced = 1, Cin_primed = 0;
		Cin_epoch = h->epoch;
		Cin_wpos = Cin_rpos = pos;
	}
	if (pos + h->frames <= Cin_wpos) {			// late or duplicate
		Cin_late++;
		return;
	}
	if (pos > Cin_wpos) Cin_lost++;
	for (; Cin_wpos < pos; Cin_wpos++)			// lost packets: silence
		memset(Cin[Cin_wpos & (CIN_RING - 1)], 0, sizeof(Cin[0]));
	if (h->flags > 0 && h->flags <= CIN_RING / 4) Cin_burst = h->flags;
	nc = h->channels;
	for (i = 0; i < h->frames; i++, pos++) {
		float* f = Cin[pos & (CIN_RING - 1)];
		for (c = 0; c < nc; c++, p += 4) {
			float v = X32A_getf(p);
			if (pos >= Cin_wpos) f[c] = v;
		}
		if (pos >= Cin_wpos) for (; c < NCARD; c++) f[c] = 0.0f;
	}
	if (pos > Cin_wpos) Cin_wpos = pos;
	if (Cin_wpos - Cin_rpos > CIN_RING - BLOCK) Cin_rpos = Cin_wpos - CIN_RING / 2;	// keep reader in the ring
}

static void receive() {
	unsigned char pkt[X32A_MAX_PACKET + 64];
	struct sockaddr_in from;
	socklen_t flen;
	X32A_header h;
	int len;

	for (;;) {
		flen = sizeof(from);
		len = recvfrom(Xa_fd, pkt, sizeof(pkt), MSG_DONTWAIT, (struct sockaddr*)&from, &flen);
		if (len <= 0) return;
		if (X32A_read_header(pkt, len, &h)) continue;
		if (h.type == X32A_HELLO) {
			if (!Xa_has_client || memcmp(&from, &Xa_client, sizeof(from))) {
				if (Xa_verbose) printf("X32 audio: X-USB driver connected from %s:%d\n", inet_ntoa(from.sin_addr), ntohs(from.sin_port));
				Xa_client = from;
				Xa_has_client = 1;
				Xa_client_running = 0;
			}
			if (Xa_verbose && !!(h.flags & X32A_FLAG_RUNNING) != Xa_client_running) {
				if (h.flags & X32A_FLAG_RUNNING) {
					printf("X32 audio: driver IO started (computer at %uHz)\n", h.rate);
				} else {
					printf("X32 audio: driver IO stopped; card inputs: %d underrun(s), %d lost, %d late packet(s)\n",
							Cin_underruns, Cin_lost, Cin_late);
				}
				Cin_underruns = Cin_lost = Cin_late = 0;
			}
			Xa_client_running = !!(h.flags & X32A_FLAG_RUNNING);
			Xa_client_seen = now_ns();
			send_status();
		} else if (h.type == X32A_AUDIO) {
			if (Xa_has_client && !memcmp(&from, &Xa_client, sizeof(from))) receive_audio(pkt, &h);
		}
		fflush(stdout);
	}
}

static void* engine(void* arg) {
	long long next, now, period;
	fd_set fds;
	struct timeval tv;
	unsigned int rate;

	(void)arg;
#ifdef __APPLE__
	pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
	next = now_ns();
	while (!Xa_quit) {
		rate = (IVAL(P.clockrate, 0) == 1) ? 44100 : 48000;
		if (rate != Xa_rate) {				// clock change: new stream epoch
			if (Xa_verbose && Xa_rate) printf("X32 audio: clock rate now %uHz\n", rate);
			Xa_rate = rate;
			Xa_epoch++;
			Xa_outpos = 0;
			Cin_synced = 0;
			M_fall = (float)pow(10.0, -METER_FALL / 20.0 * BLOCK / rate);
		}
		card_mode();
		period = (long long)BLOCK * 1000000000LL / Xa_rate;
		now = now_ns();
		if (now < next) {
			FD_ZERO(&fds);
			FD_SET(Xa_fd, &fds);
			tv.tv_sec = 0;
			tv.tv_usec = (long)((next - now) / 1000);
			if (select(Xa_fd + 1, &fds, NULL, NULL, &tv) > 0) receive();
			continue;
		}
		receive();
		if (Xa_has_client && now - Xa_client_seen > CLIENT_TIMEOUT) {
			if (Xa_verbose) printf("X32 audio: X-USB driver disconnected\n");
			Xa_has_client = Xa_client_running = 0;
			Cin_synced = 0;
		}
		process();
		if (Xa_has_client && Xa_client_running) send_block();
		Xa_outpos += BLOCK;
		next += period;
		if (now - next > 100000000LL) next = now;	// we were stalled, don't try to catch up
	}
	return NULL;
}

int X32AudioStart(X32AudioParams* params, int port, int signal, int verbose) {
	struct sockaddr_in addr;

	P = *params;
	Xa_signal = signal;
	Xa_verbose = verbose;
	if ((Xa_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		perror("X32 audio: socket");
		return 1;
	}
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);	// the driver usually is on 127.0.0.1
	addr.sin_port = htons(port);
	if (bind(Xa_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		perror("X32 audio: bind");
		close(Xa_fd);
		Xa_fd = -1;
		return 1;
	}
	{
		int sz = 4 << 20;	// room for the driver's IO bursts (up to 4096 frames x 32 channels)
		setsockopt(Xa_fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof(sz));
		setsockopt(Xa_fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
	}
	Xa_quit = 0;
	if (pthread_create(&Xa_thread, NULL, engine, NULL)) {
		perror("X32 audio: thread");
		close(Xa_fd);
		Xa_fd = -1;
		return 1;
	}
	return 0;
}

void X32AudioStop() {
	if (Xa_fd < 0) return;
	Xa_quit = 1;
	pthread_join(Xa_thread, NULL);
	close(Xa_fd);
	Xa_fd = -1;
}

int X32AudioMeters(int meter, unsigned char* blob, int nfloats) {
	int i;
	if (Xa_fd < 0) return 0;
	switch (meter) {
	case 0:		// 32 input channels, 8 aux returns, 4x2 st fx returns, 16 bus masters, 6 matrixes
		if (nfloats < 70) return 0;
		for (i = 0; i < X32AE_NCH; i++) X32A_putf(blob + 4 * i, M_chin[i]);
		for (i = 0; i < X32AE_NBUS; i++) X32A_putf(blob + 4 * (48 + i), M_bus[i]);
		return 1;
	case 1:		// 32 input channels, 32 gate gain reductions, 32 dynamics gain reductions
		if (nfloats < 96) return 0;
		for (i = 0; i < X32AE_NCH; i++) X32A_putf(blob + 4 * i, M_chin[i]);
		return 1;
	case 2:		// 16 bus masters, 6 matrixes, 2 main LR, 1 mono M/C, gain reductions...
		if (nfloats < 25) return 0;
		for (i = 0; i < X32AE_NBUS; i++) X32A_putf(blob + 4 * i, M_bus[i]);
		X32A_putf(blob + 4 * 22, M_main[0]);
		X32A_putf(blob + 4 * 23, M_main[1]);
		X32A_putf(blob + 4 * 24, M_mono);
		return 1;
	default:
		return 0;
	}
}

#endif // __WIN32__
