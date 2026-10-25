/*
 * X32AudioProtocol.h
 *
 * Wire protocol between the X32 emulator's audio engine (X32AudioEngine.c)
 * and the X32 emulator virtual audio driver for macOS (X32USBDriver/).
 *
 * The real X32 exposes its expansion card (X-USB / X-LIVE) to a computer as a
 * 32 in / 32 out USB audio interface. The emulator replicates this over UDP:
 *
 *   driver   --- HELLO (every ~250ms) ------------------------>  emulator
 *   driver   <-- STATUS (reply to HELLO: rate, channel counts) --  emulator
 *   driver   --- AUDIO  (computer -> X32, "card inputs") ------>  emulator
 *   driver   <-- AUDIO  (X32 -> computer, "card outputs") -----  emulator
 *
 * The emulator is the clock master (as a real X32 is): it runs at the rate set
 * by /-prefs/clockrate, and the driver follows that rate.
 *
 * All fields are little-endian. Audio payloads are interleaved 32-bit floats.
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */

#ifndef X32AUDIOPROTOCOL_H_
#define X32AUDIOPROTOCOL_H_

#include <stdint.h>
#include <string.h>

#define X32A_MAGIC          0x41323358u // "X32A" as little-endian bytes
#define X32A_VERSION        1
#define X32A_DEFAULT_PORT   10025       // OSC is 10023 (X32) / 10024 (XAir)

#define X32A_MAX_CHANNELS   32          // X-USB card: 32 in / 32 out
#define X32A_PACKET_FRAMES  64          // max frames in one AUDIO packet

enum X32A_type {
	X32A_HELLO  = 1,    // driver -> emulator: keepalive / registration
	X32A_STATUS = 2,    // emulator -> driver: reply to HELLO
	X32A_AUDIO  = 3,    // either direction: audio frames
};

#define X32A_FLAG_RUNNING   0x0001      // HELLO: driver IO is running, send card outputs

typedef struct {
	uint32_t magic;     // X32A_MAGIC
	uint16_t version;   // X32A_VERSION
	uint16_t type;      // enum X32A_type
	uint32_t rate;      // sample rate (Hz) of the sender's stream
	uint16_t channels;  // AUDIO: interleaved channels in payload; HELLO/STATUS: 0
	uint16_t frames;    // AUDIO: frames in payload; HELLO/STATUS: 0
	uint64_t position;  // AUDIO: stream position (frames) of the first frame
	uint32_t epoch;     // stream epoch; a change means "restart, resync"
	uint32_t flags;     // HELLO: X32A_FLAG_*; AUDIO: sender's IO burst size (frames)
} X32A_header;          // 32 bytes on the wire

#define X32A_HEADER_SIZE    32

typedef struct {
	uint16_t to_computer;   // channels the X32 sends to the computer (card outputs)
	uint16_t from_computer; // channels the X32 accepts from the computer (card inputs)
	char     name[28];      // console name (/-prefs/name), NUL terminated
} X32A_status;              // 32 bytes on the wire, follows the header

#define X32A_STATUS_SIZE    32
#define X32A_MAX_PACKET     (X32A_HEADER_SIZE + X32A_PACKET_FRAMES * X32A_MAX_CHANNELS * 4)

//
// little-endian helpers (no alignment assumptions)
static inline void X32A_put16(unsigned char* p, uint16_t v) {
	p[0] = (unsigned char)v; p[1] = (unsigned char)(v >> 8);
}
static inline void X32A_put32(unsigned char* p, uint32_t v) {
	X32A_put16(p, (uint16_t)v); X32A_put16(p + 2, (uint16_t)(v >> 16));
}
static inline void X32A_put64(unsigned char* p, uint64_t v) {
	X32A_put32(p, (uint32_t)v); X32A_put32(p + 4, (uint32_t)(v >> 32));
}
static inline uint16_t X32A_get16(const unsigned char* p) {
	return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t X32A_get32(const unsigned char* p) {
	return (uint32_t)X32A_get16(p) | ((uint32_t)X32A_get16(p + 2) << 16);
}
static inline uint64_t X32A_get64(const unsigned char* p) {
	return (uint64_t)X32A_get32(p) | ((uint64_t)X32A_get32(p + 4) << 32);
}
static inline void X32A_putf(unsigned char* p, float f) {
	uint32_t u; memcpy(&u, &f, 4); X32A_put32(p, u);
}
static inline float X32A_getf(const unsigned char* p) {
	uint32_t u = X32A_get32(p); float f; memcpy(&f, &u, 4); return f;
}

static inline void X32A_write_header(unsigned char* p, const X32A_header* h) {
	X32A_put32(p + 0, X32A_MAGIC);
	X32A_put16(p + 4, X32A_VERSION);
	X32A_put16(p + 6, h->type);
	X32A_put32(p + 8, h->rate);
	X32A_put16(p + 12, h->channels);
	X32A_put16(p + 14, h->frames);
	X32A_put64(p + 16, h->position);
	X32A_put32(p + 24, h->epoch);
	X32A_put32(p + 28, h->flags);
}

// returns 0 if the packet is valid (header + payload size consistent)
static inline int X32A_read_header(const unsigned char* p, int len, X32A_header* h) {
	if (len < X32A_HEADER_SIZE) return -1;
	h->magic = X32A_get32(p + 0);
	h->version = X32A_get16(p + 4);
	h->type = X32A_get16(p + 6);
	h->rate = X32A_get32(p + 8);
	h->channels = X32A_get16(p + 12);
	h->frames = X32A_get16(p + 14);
	h->position = X32A_get64(p + 16);
	h->epoch = X32A_get32(p + 24);
	h->flags = X32A_get32(p + 28);
	if (h->magic != X32A_MAGIC || h->version != X32A_VERSION) return -1;
	if (h->type == X32A_AUDIO) {
		if (h->channels == 0 || h->channels > X32A_MAX_CHANNELS) return -1;
		if (h->frames == 0 || h->frames > X32A_PACKET_FRAMES) return -1;
		if (len < X32A_HEADER_SIZE + h->frames * h->channels * 4) return -1;
	} else if (h->type == X32A_STATUS) {
		if (len < X32A_HEADER_SIZE + X32A_STATUS_SIZE) return -1;
	} else if (h->type != X32A_HELLO) {
		return -1;
	}
	return 0;
}

#endif /* X32AUDIOPROTOCOL_H_ */
