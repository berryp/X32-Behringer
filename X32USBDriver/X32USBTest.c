/*
 * X32USBTest.c
 *
 * Test client for the X32 emulator's X-USB audio: runs the X32 virtual audio
 * driver's transport (X32USBTransport.c) the way CoreAudio would, without
 * installing anything. Works on Linux and macOS.
 *
 * It plays a test tone on each computer -> X32 channel (card input n gets
 * 1000Hz * n at -6dBFS, n = 1..4, the others silence), records the X32 -> computer
 * channels, and prints the level and frequency found on each recorded channel.
 *
 * usage: X32USBTest [-i emulator IP, default 127.0.0.1] [-p port, default 10025]
 *                   [-t seconds, default 3] [-b IO buffer frames, default 512]
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include "X32USBTransport.h"

#define NCH XUT_CHANNELS

static XUT T;

static void rate_cb(void* ctx, uint32_t rate) {
	(void)ctx;
	printf("emulator asks for %uHz: following\n", rate);
	XUT_set_rate(&T, rate);
}

static void sleep_until(uint64_t host_time, double tps) {
	uint64_t now = XUT_host_time();
	if (host_time > now) {
		double s = (double)(host_time - now) / tps;
		struct timespec ts;
		ts.tv_sec = (time_t)s;
		ts.tv_nsec = (long)((s - (double)ts.tv_sec) * 1e9);
		nanosleep(&ts, NULL);
	}
}

int main(int argc, char** argv) {
	char host[64] = "127.0.0.1";
	int port = X32A_DEFAULT_PORT, seconds = 3, buffer = 512, opt, c;
	uint32_t rate;
	double tps, phase[NCH] = {0};
	float *in, *out;
	double peak[NCH], sumsq[NCH];
	long cross[NCH], frames_meas = 0, cycle, cycles;
	float last[NCH];
	uint64_t anchor;

	while ((opt = getopt(argc, argv, "i:p:t:b:h")) != -1) {
		switch (opt) {
		case 'i': strncpy(host, optarg, sizeof(host) - 1); break;
		case 'p': port = atoi(optarg); break;
		case 't': seconds = atoi(optarg); break;
		case 'b': buffer = atoi(optarg); break;
		default:
			printf("usage: X32USBTest [-i emulator IP] [-p port] [-t seconds] [-b IO buffer frames]\n");
			return 0;
		}
	}
	if (buffer < 16 || buffer > 4096) buffer = 512;
	if (XUT_init(&T, host, port, rate_cb, NULL)) {
		printf("cannot reach %s:%d\n", host, port);
		return 1;
	}
	tps = XUT_host_ticks_per_second();
	// wait for the emulator's status (and its sample rate)
	for (c = 0; c < 20 && !XUT_emulator_rate(&T); c++) usleep(100000);
	if (!(rate = XUT_emulator_rate(&T))) {
		printf("no answer from the X32 emulator at %s:%d\n", host, port);
		XUT_shutdown(&T);
		return 1;
	}
	usleep(100000);		// let a rate change settle
	rate = atomic_load(&T.rate);
	printf("X32 emulator \"%s\" at %uHz, %d ch to computer, %d ch from computer\n",
			T.emu_name, rate, atomic_load(&T.emu_to_computer), atomic_load(&T.emu_from_computer));

	in = calloc((size_t)buffer * NCH, sizeof(float));
	out = calloc((size_t)buffer * NCH, sizeof(float));
	memset(peak, 0, sizeof(peak));
	memset(sumsq, 0, sizeof(sumsq));
	memset(cross, 0, sizeof(cross));
	memset(last, 0, sizeof(last));
	anchor = XUT_host_time();
	XUT_start(&T, anchor, rate);
	cycles = (long)seconds * rate / buffer;
	for (cycle = 1; cycle <= cycles; cycle++) {
		int64_t now = (int64_t)cycle * buffer;	// IO cycle at sample time 'now'
		int i;
		sleep_until(anchor + (uint64_t)((double)now / rate * tps), tps);
		// output: the HAL mixes ahead of time
		for (i = 0; i < buffer; i++) {
			for (c = 0; c < NCH; c++) {
				float v = 0.0f;
				if (c < 4) {
					v = 0.5f * (float)sin(phase[c]);
					phase[c] += 2.0 * M_PI * 1000.0 * (c + 1) / rate;
				}
				out[i * NCH + c] = v;
			}
		}
		XUT_write_output(&T, now + 64, out, buffer);
		// input: read what was received a safety offset ago
		XUT_read_input(&T, now - buffer - XUT_SAFETY_FRAMES, in, buffer);
		if (cycle < cycles / 3) continue;		// let both jitter buffers settle
		for (i = 0; i < buffer; i++, frames_meas++) {
			for (c = 0; c < NCH; c++) {
				float v = in[i * NCH + c];
				if (fabsf(v) > peak[c]) peak[c] = fabsf(v);
				sumsq[c] += (double)v * v;
				if ((last[c] < 0.0f) != (v < 0.0f)) cross[c]++;
				last[c] = v;
			}
		}
	}
	XUT_stop(&T);
	printf("input timeline resyncs: %u (1 is the initial sync)\n", atomic_load(&T.in_resyncs));
	printf("X32 -> computer (X32 card outputs):\n ch   peak dBFS   rms dBFS   freq Hz\n");
	for (c = 0; c < NCH; c++) {
		double rms = sqrt(sumsq[c] / frames_meas);
		if (peak[c] < 1e-6) printf(" %2d     silence\n", c + 1);
		else printf(" %2d   %8.1f   %8.1f   %7.0f\n", c + 1, 20 * log10(peak[c]), 20 * log10(rms),
					cross[c] / 2.0 * rate / frames_meas);
	}
	XUT_shutdown(&T);
	free(in);
	free(out);
	return 0;
}
