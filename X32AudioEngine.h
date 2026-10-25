/*
 * X32AudioEngine.h
 *
 * Audio engine for the X32 emulator: makes the emulator's expansion card
 * behave like an X-USB card (32 in / 32 out USB audio interface), streaming
 * audio over UDP to the X32 virtual audio driver (see X32USBDriver/).
 *
 * The engine reads the emulator's live parameter values (routing, faders,
 * mutes, sends, ...) through the pointers in X32AudioParams; X32.c fills them
 * in from its command tables. Values are only read by the engine.
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */

#ifndef X32AUDIOENGINE_H_
#define X32AUDIOENGINE_H_

#define X32AE_NCH	32		// input channels
#define X32AE_NBUS	16		// mix buses
#define X32AE_NDCA	8		// DCA groups
#define X32AE_NMGRP	6		// mute groups
#define X32AE_NOUT	16		// main outputs (/outputs/main/01..16)

enum X32AE_signal {			// what the emulator's local inputs (AN1-32) carry
	X32AE_SIG_SILENCE = 0,	// nothing plugged in
	X32AE_SIG_TONES = 1,	// local input n: sine at 100Hz * n, -20dBFS
	X32AE_SIG_NOISE = 2,	// local inputs: white noise, -30dBFS
};

typedef struct {
	int		*routswitch;				// /config/routing/routswitch (REC/PLAY)
	int		*rt_in[4];					// /config/routing/IN/1-8...25-32
	int		*rt_play[4];				// /config/routing/PLAY/1-8...25-32
	int		*rt_card[4];				// /config/routing/CARD/1-8...25-32
	int		*mute_grp[X32AE_NMGRP];		// /config/mute/1..6
	struct {
		int		*source;				// /ch/xx/config/source
		int		*dca;					// /ch/xx/grp/dca (bitmask)
		int		*mute;					// /ch/xx/grp/mute (bitmask)
		float	*trim;					// /ch/xx/preamp/trim
		int		*invert;				// /ch/xx/preamp/invert
		int		*on;					// /ch/xx/mix/on
		float	*fader;					// /ch/xx/mix/fader
		int		*st;					// /ch/xx/mix/st
		float	*pan;					// /ch/xx/mix/pan
		int		*mono;					// /ch/xx/mix/mono
		float	*mlevel;				// /ch/xx/mix/mlevel
		int		*send_on[X32AE_NBUS];	// /ch/xx/mix/NN/on
		float	*send_level[X32AE_NBUS];// /ch/xx/mix/NN/level
		int		*send_type[X32AE_NBUS];	// /ch/xx/mix/NN/type (odd sends carry the type)
	} ch[X32AE_NCH];
	struct {
		int		*on;					// /bus/xx/mix/on
		float	*fader;					// /bus/xx/mix/fader
		int		*st;					// /bus/xx/mix/st
		float	*pan;					// /bus/xx/mix/pan
		int		*mono;					// /bus/xx/mix/mono
		float	*mlevel;				// /bus/xx/mix/mlevel
	} bus[X32AE_NBUS];
	int		*dca_on[X32AE_NDCA];		// /dca/n/on
	float	*dca_fader[X32AE_NDCA];		// /dca/n/fader
	int		*main_on;					// /main/st/mix/on
	float	*main_fader;				// /main/st/mix/fader
	float	*main_pan;					// /main/st/mix/pan
	int		*mono_on;					// /main/m/mix/on
	float	*mono_fader;				// /main/m/mix/fader
	int		*out_src[X32AE_NOUT];		// /outputs/main/NN/src
	int		*out_pos[X32AE_NOUT];		// /outputs/main/NN/pos
	int		*clockrate;					// /-prefs/clockrate (48K, 44K1)
	int		*usbmode;					// /-prefs/card/USBmode (32/32, 16/16, ...)
	char	**name;						// /-prefs/name (console name)
} X32AudioParams;

//
// Start the audio engine thread: listen for the virtual driver on UDP 'port'.
// Returns 0 on success.
int		X32AudioStart(X32AudioParams* params, int port, int signal, int verbose);
void	X32AudioStop();
//
// Fill a /meters/n blob with live levels; 'blob' points at the first float of
// the meter blob (little-endian floats), 'nfloats' is the number of floats.
// Returns 1 if the meter was filled, 0 if the engine does not provide it.
int		X32AudioMeters(int meter, unsigned char* blob, int nfloats);

#endif /* X32AUDIOENGINE_H_ */
