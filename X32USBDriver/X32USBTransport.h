/*
 * X32USBTransport.h
 *
 * Network side of the X32 virtual audio driver: talks to the X32 emulator's
 * audio engine (see ../X32AudioProtocol.h) and keeps the audio received from
 * the emulator in a ring buffer indexed by the driver's own sample time.
 *
 * Portable POSIX C so it can be exercised outside of coreaudiod (see X32USBTest.c).
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */

#ifndef X32USBTRANSPORT_H_
#define X32USBTRANSPORT_H_

#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>
#include "../X32AudioProtocol.h"

#define XUT_CHANNELS		X32A_MAX_CHANNELS	// channels in each direction
#define XUT_RING_FRAMES		16384				// input ring buffer (power of 2)
#define XUT_SAFETY_FRAMES	256					// input safety offset the driver reports

typedef void (*XUT_rate_cb)(void* ctx, uint32_t rate);	// emulator runs at another rate

typedef struct {
	int					fd;
	pthread_t			thread;
	atomic_int			quit;
	XUT_rate_cb			rate_cb;
	void*				rate_ctx;
	// driver clock (set when IO starts)
	atomic_int			running;
	atomic_uint			epoch;					// incremented on each IO start
	_Atomic uint64_t	anchor;					// host time of sample time 0
	atomic_uint			rate;					// driver's nominal rate
	double				ticks_per_second;
	// emulator status
	atomic_uint			emu_rate;
	atomic_int			emu_to_computer, emu_from_computer;
	_Atomic uint64_t	emu_seen;				// host time of last packet from the emulator
	atomic_uint			requested_rate;			// rate change asked to the HAL
	char				emu_name[28];
	// input: emulator -> computer
	float				ring[XUT_RING_FRAMES][XUT_CHANNELS];
	_Atomic int64_t		in_wpos;				// driver sample time after the last written frame
	int64_t				in_base;				// driver sample time = emulator position + base
	uint64_t			in_expect;				// next expected emulator position
	uint32_t			in_epoch, in_io_epoch;
	int					in_synced;
	int					in_bad;					// consecutive packets too early/late
	atomic_uint			in_resyncs;				// statistics: input timeline resets
	// output: computer -> emulator (IO thread only)
	unsigned char		out_pkt[X32A_MAX_PACKET];
} XUT;

uint64_t	XUT_host_time(void);
double		XUT_host_ticks_per_second(void);

// non-realtime
int			XUT_init(XUT* t, const char* host, int port, XUT_rate_cb cb, void* ctx);
void		XUT_shutdown(XUT* t);
void		XUT_start(XUT* t, uint64_t anchor_host_time, uint32_t rate);
void		XUT_stop(XUT* t);
void		XUT_set_rate(XUT* t, uint32_t rate);
int			XUT_connected(XUT* t);				// heard from the emulator recently
uint32_t	XUT_emulator_rate(XUT* t);			// 0 if unknown

// realtime (IO thread), interleaved XUT_CHANNELS float buffers
void		XUT_read_input(XUT* t, int64_t sample_time, float* dst, uint32_t frames);
void		XUT_write_output(XUT* t, int64_t sample_time, const float* src, uint32_t frames);

#endif /* X32USBTRANSPORT_H_ */
