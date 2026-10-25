/*
 * X32USBDriver.c
 *
 * X32 Emulator virtual audio driver for macOS (a CoreAudio AudioServerPlugIn).
 *
 * Replicates using an X32 with an X-USB (or X-LIVE) expansion card as an audio
 * interface: the driver publishes a 32 in / 32 out audio device, "X32 Emulator",
 * whose audio is streamed over UDP to/from the X32 emulator (X32.c and its audio
 * engine X32AudioEngine.c), by default on 127.0.0.1 port 10025.
 *
 *  - device inputs 1-32 (what the computer records) are the X32 card outputs,
 *    i.e. whatever /config/routing/CARD sends to the card;
 *  - device outputs 1-32 (what the computer plays) are the X32 card inputs,
 *    used by /config/routing/IN (or PLAY) blocks set to CARD1-8 ... CARD25-32.
 *
 * As with a real X32, the console is the clock master: the device follows the
 * emulator's sample rate (/-prefs/clockrate, 44.1kHz or 48kHz).
 *
 * The emulator's address can be changed in the driver's Info.plist
 * (X32EmulatorHost, X32EmulatorPort); restart coreaudiod afterwards.
 *
 * The plug-in structure follows Apple's "NullAudio" AudioServerPlugIn sample.
 *
 *  Copyright (C) 2026, released under the same GPL terms as the rest of this repository
 */

#include <CoreAudio/AudioServerPlugIn.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdatomic.h>
#include <syslog.h>
#include "X32USBTransport.h"

#define kBundleID				"org.x32emulator.driver.X32USB"
#define kDeviceUID				"X32EmulatorXUSB_UID"
#define kDeviceModelUID			"X32EmulatorXUSB_ModelUID"
#define kDeviceName				"X32 Emulator"
#define kManufacturer			"X32 Emulator"
#define kChannels				XUT_CHANNELS
#define kBytesPerFrame			(kChannels * sizeof(Float32))
#define kZeroTimeStampPeriod	16384
#define kLatencyFrames			64				// one emulator block
#define kOutputSafetyFrames		64

#define Log(...)				syslog(LOG_NOTICE, "X32USB: " __VA_ARGS__)

enum {
	kObjectID_PlugIn			= kAudioObjectPlugInObject,
	kObjectID_Device			= 2,
	kObjectID_Stream_Input		= 3,
	kObjectID_Stream_Output		= 4,
};

static const Float64 kRates[] = {44100.0, 48000.0};
#define kNumRates (sizeof(kRates) / sizeof(kRates[0]))

//
// Driver interface
static HRESULT	X32USB_QueryInterface(void* inDriver, REFIID inUUID, LPVOID* outInterface);
static ULONG	X32USB_AddRef(void* inDriver);
static ULONG	X32USB_Release(void* inDriver);
static OSStatus	X32USB_Initialize(AudioServerPlugInDriverRef inDriver, AudioServerPlugInHostRef inHost);
static OSStatus	X32USB_CreateDevice(AudioServerPlugInDriverRef inDriver, CFDictionaryRef inDescription, const AudioServerPlugInClientInfo* inClientInfo, AudioObjectID* outDeviceObjectID);
static OSStatus	X32USB_DestroyDevice(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID);
static OSStatus	X32USB_AddDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, const AudioServerPlugInClientInfo* inClientInfo);
static OSStatus	X32USB_RemoveDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, const AudioServerPlugInClientInfo* inClientInfo);
static OSStatus	X32USB_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt64 inChangeAction, void* inChangeInfo);
static OSStatus	X32USB_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt64 inChangeAction, void* inChangeInfo);
static Boolean	X32USB_HasProperty(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress);
static OSStatus	X32USB_IsPropertySettable(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable);
static OSStatus	X32USB_GetPropertyDataSize(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32* outDataSize);
static OSStatus	X32USB_GetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, UInt32* outDataSize, void* outData);
static OSStatus	X32USB_SetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, const void* inData);
static OSStatus	X32USB_StartIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID);
static OSStatus	X32USB_StopIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID);
static OSStatus	X32USB_GetZeroTimeStamp(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, Float64* outSampleTime, UInt64* outHostTime, UInt64* outSeed);
static OSStatus	X32USB_WillDoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, Boolean* outWillDo, Boolean* outWillDoInPlace);
static OSStatus	X32USB_BeginIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo);
static OSStatus	X32USB_DoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, AudioObjectID inStreamObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo, void* ioMainBuffer, void* ioSecondaryBuffer);
static OSStatus	X32USB_EndIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo);

static AudioServerPlugInDriverInterface gInterface = {
	NULL,
	X32USB_QueryInterface,
	X32USB_AddRef,
	X32USB_Release,
	X32USB_Initialize,
	X32USB_CreateDevice,
	X32USB_DestroyDevice,
	X32USB_AddDeviceClient,
	X32USB_RemoveDeviceClient,
	X32USB_PerformDeviceConfigurationChange,
	X32USB_AbortDeviceConfigurationChange,
	X32USB_HasProperty,
	X32USB_IsPropertySettable,
	X32USB_GetPropertyDataSize,
	X32USB_GetPropertyData,
	X32USB_SetPropertyData,
	X32USB_StartIO,
	X32USB_StopIO,
	X32USB_GetZeroTimeStamp,
	X32USB_WillDoIOOperation,
	X32USB_BeginIOOperation,
	X32USB_DoIOOperation,
	X32USB_EndIOOperation
};
static AudioServerPlugInDriverInterface*	gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef			gDriverRef = &gInterfacePtr;

//
// State
static pthread_mutex_t			gStateMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t			gIOMutex = PTHREAD_MUTEX_INITIALIZER;
static UInt32					gRefCount = 0;
static AudioServerPlugInHostRef	gHost = NULL;
static Float64					gSampleRate = 48000.0;
static UInt64					gIOCount = 0;
static UInt64					gAnchorHostTime = 0;
static UInt64					gNumberTimeStamps = 0;
static Float64					gHostTicksPerFrame = 0.0;
static Float64					gHostClockFrequency = 0.0;
static Boolean					gInputActive = true;
static Boolean					gOutputActive = true;
static XUT						gTransport;
static int						gTransportOK = 0;

//
// Factory, named in Info.plist (CFPlugInFactories)
void* X32USB_Create(CFAllocatorRef inAllocator, CFUUIDRef inRequestedTypeUUID);
void* X32USB_Create(CFAllocatorRef inAllocator, CFUUIDRef inRequestedTypeUUID) {
	(void)inAllocator;
	if (CFEqual(inRequestedTypeUUID, kAudioServerPlugInTypeUUID)) return gDriverRef;
	return NULL;
}

static HRESULT X32USB_QueryInterface(void* inDriver, REFIID inUUID, LPVOID* outInterface) {
	CFUUIDRef uuid;
	HRESULT result = E_NOINTERFACE;

	if (inDriver != gDriverRef || outInterface == NULL) return kAudioHardwareBadObjectError;
	uuid = CFUUIDCreateFromUUIDBytes(NULL, inUUID);
	if (uuid == NULL) return kAudioHardwareIllegalOperationError;
	if (CFEqual(uuid, IUnknownUUID) || CFEqual(uuid, kAudioServerPlugInDriverInterfaceUUID)) {
		pthread_mutex_lock(&gStateMutex);
		++gRefCount;
		pthread_mutex_unlock(&gStateMutex);
		*outInterface = gDriverRef;
		result = 0;
	}
	CFRelease(uuid);
	return result;
}

static ULONG X32USB_AddRef(void* inDriver) {
	ULONG r;
	if (inDriver != gDriverRef) return 0;
	pthread_mutex_lock(&gStateMutex);
	if (gRefCount < UINT32_MAX) ++gRefCount;
	r = gRefCount;
	pthread_mutex_unlock(&gStateMutex);
	return r;
}

static ULONG X32USB_Release(void* inDriver) {
	ULONG r;
	if (inDriver != gDriverRef) return 0;
	pthread_mutex_lock(&gStateMutex);
	if (gRefCount > 0) --gRefCount;
	r = gRefCount;
	pthread_mutex_unlock(&gStateMutex);
	// the driver is a static object, nothing to free
	return r;
}

//
// the emulator runs at another rate: ask the HAL to switch (any thread)
static void rate_from_emulator(void* ctx, uint32_t rate) {
	(void)ctx;
	Log("emulator clock is %u Hz, following", rate);
	if (gHost) gHost->RequestDeviceConfigurationChange(gHost, kObjectID_Device, (UInt64)rate, NULL);
}

//
// settings from Info.plist
static void get_settings(char* host, size_t hostlen, int* port) {
	CFBundleRef bundle = CFBundleGetBundleWithIdentifier(CFSTR(kBundleID));
	CFTypeRef v;

	strlcpy(host, "127.0.0.1", hostlen);
	*port = X32A_DEFAULT_PORT;
	if (bundle == NULL) return;
	v = CFBundleGetValueForInfoDictionaryKey(bundle, CFSTR("X32EmulatorHost"));
	if (v && CFGetTypeID(v) == CFStringGetTypeID()) CFStringGetCString((CFStringRef)v, host, (CFIndex)hostlen, kCFStringEncodingUTF8);
	v = CFBundleGetValueForInfoDictionaryKey(bundle, CFSTR("X32EmulatorPort"));
	if (v && CFGetTypeID(v) == CFNumberGetTypeID()) CFNumberGetValue((CFNumberRef)v, kCFNumberIntType, port);
}

static OSStatus X32USB_Initialize(AudioServerPlugInDriverRef inDriver, AudioServerPlugInHostRef inHost) {
	mach_timebase_info_data_t tb;
	char host[256];
	int port;

	if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
	gHost = inHost;
	mach_timebase_info(&tb);
	gHostClockFrequency = (Float64)tb.denom / (Float64)tb.numer * 1000000000.0;
	gHostTicksPerFrame = gHostClockFrequency / gSampleRate;
	get_settings(host, sizeof(host), &port);
	if (XUT_init(&gTransport, host, port, rate_from_emulator, NULL) == 0) {
		gTransportOK = 1;
		XUT_set_rate(&gTransport, (uint32_t)gSampleRate);
		Log("X32 emulator expected at %s:%d", host, port);
	} else {
		Log("cannot set up the connection to %s:%d", host, port);
	}
	return 0;
}

static OSStatus X32USB_CreateDevice(AudioServerPlugInDriverRef inDriver, CFDictionaryRef inDescription, const AudioServerPlugInClientInfo* inClientInfo, AudioObjectID* outDeviceObjectID) {
	(void)inDriver; (void)inDescription; (void)inClientInfo; (void)outDeviceObjectID;
	return kAudioHardwareUnsupportedOperationError;
}

static OSStatus X32USB_DestroyDevice(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID) {
	(void)inDriver; (void)inDeviceObjectID;
	return kAudioHardwareUnsupportedOperationError;
}

static OSStatus X32USB_AddDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, const AudioServerPlugInClientInfo* inClientInfo) {
	(void)inClientInfo;
	if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
	if (inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	return 0;
}

static OSStatus X32USB_RemoveDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, const AudioServerPlugInClientInfo* inClientInfo) {
	(void)inClientInfo;
	if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
	if (inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	return 0;
}

static int valid_rate(Float64 rate) {
	unsigned i;
	for (i = 0; i < kNumRates; i++) if (rate == kRates[i]) return 1;
	return 0;
}

static OSStatus X32USB_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt64 inChangeAction, void* inChangeInfo) {
	(void)inChangeInfo;
	if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
	if (inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	if (!valid_rate((Float64)inChangeAction)) return kAudioHardwareBadObjectError;
	// the HAL stops IO before calling this
	pthread_mutex_lock(&gStateMutex);
	gSampleRate = (Float64)inChangeAction;
	gHostTicksPerFrame = gHostClockFrequency / gSampleRate;
	pthread_mutex_unlock(&gStateMutex);
	if (gTransportOK) XUT_set_rate(&gTransport, (uint32_t)inChangeAction);
	Log("sample rate now %u Hz", (unsigned)inChangeAction);
	return 0;
}

static OSStatus X32USB_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt64 inChangeAction, void* inChangeInfo) {
	(void)inChangeAction; (void)inChangeInfo;
	if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
	if (inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	return 0;
}

//
// Properties
//
static void fill_format(AudioStreamBasicDescription* f, Float64 rate) {
	memset(f, 0, sizeof(*f));
	f->mSampleRate = rate;
	f->mFormatID = kAudioFormatLinearPCM;
	f->mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
	f->mBytesPerPacket = kBytesPerFrame;
	f->mFramesPerPacket = 1;
	f->mBytesPerFrame = kBytesPerFrame;
	f->mChannelsPerFrame = kChannels;
	f->mBitsPerChannel = 32;
}

static Float64 current_rate(void) {
	Float64 r;
	pthread_mutex_lock(&gStateMutex);
	r = gSampleRate;
	pthread_mutex_unlock(&gStateMutex);
	return r;
}

static Boolean is_stream(AudioObjectID id) {
	return id == kObjectID_Stream_Input || id == kObjectID_Stream_Output;
}

static Boolean X32USB_HasProperty(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress) {
	(void)inClientProcessID;
	if (inDriver != gDriverRef || inAddress == NULL) return false;
	if (inObjectID == kObjectID_PlugIn) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioPlugInPropertyBoxList:
		case kAudioPlugInPropertyDeviceList:
		case kAudioPlugInPropertyTranslateUIDToDevice:
		case kAudioPlugInPropertyResourceBundle:
			return true;
		}
	} else if (inObjectID == kObjectID_Device) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyName:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioDevicePropertyDeviceUID:
		case kAudioDevicePropertyModelUID:
		case kAudioDevicePropertyTransportType:
		case kAudioDevicePropertyRelatedDevices:
		case kAudioDevicePropertyClockDomain:
		case kAudioDevicePropertyDeviceIsAlive:
		case kAudioDevicePropertyDeviceIsRunning:
		case kAudioObjectPropertyControlList:
		case kAudioDevicePropertyNominalSampleRate:
		case kAudioDevicePropertyAvailableNominalSampleRates:
		case kAudioDevicePropertyIsHidden:
		case kAudioDevicePropertyZeroTimeStampPeriod:
		case kAudioDevicePropertyStreams:
			return true;
		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
		case kAudioDevicePropertyLatency:
		case kAudioDevicePropertySafetyOffset:
		case kAudioDevicePropertyPreferredChannelsForStereo:
		case kAudioDevicePropertyPreferredChannelLayout:
			return inAddress->mScope == kAudioObjectPropertyScopeInput || inAddress->mScope == kAudioObjectPropertyScopeOutput;
		case kAudioObjectPropertyElementName:
			return (inAddress->mScope == kAudioObjectPropertyScopeInput || inAddress->mScope == kAudioObjectPropertyScopeOutput)
					&& inAddress->mElement >= 1 && inAddress->mElement <= kChannels;
		}
	} else if (is_stream(inObjectID)) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioStreamPropertyIsActive:
		case kAudioStreamPropertyDirection:
		case kAudioStreamPropertyTerminalType:
		case kAudioStreamPropertyStartingChannel:
		case kAudioStreamPropertyLatency:
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
			return true;
		}
	}
	return false;
}

static OSStatus X32USB_IsPropertySettable(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable) {
	if (inDriver != gDriverRef || inAddress == NULL || outIsSettable == NULL) return kAudioHardwareIllegalOperationError;
	if (!X32USB_HasProperty(inDriver, inObjectID, inClientProcessID, inAddress)) return kAudioHardwareUnknownPropertyError;
	*outIsSettable = false;
	if (inObjectID == kObjectID_Device && inAddress->mSelector == kAudioDevicePropertyNominalSampleRate) *outIsSettable = true;
	if (is_stream(inObjectID)) {
		switch (inAddress->mSelector) {
		case kAudioStreamPropertyIsActive:
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
			*outIsSettable = true;
		}
	}
	return 0;
}

#define kLayoutSize	(offsetof(AudioChannelLayout, mChannelDescriptions) + kChannels * sizeof(AudioChannelDescription))

static OSStatus X32USB_GetPropertyDataSize(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32* outDataSize) {
	(void)inQualifierDataSize; (void)inQualifierData;
	if (inDriver != gDriverRef || inAddress == NULL || outDataSize == NULL) return kAudioHardwareIllegalOperationError;
	if (!X32USB_HasProperty(inDriver, inObjectID, inClientProcessID, inAddress)) return kAudioHardwareUnknownPropertyError;
	switch (inAddress->mSelector) {
	case kAudioObjectPropertyBaseClass:
	case kAudioObjectPropertyClass:
		*outDataSize = sizeof(AudioClassID);
		return 0;
	case kAudioObjectPropertyOwner:
		*outDataSize = sizeof(AudioObjectID);
		return 0;
	case kAudioObjectPropertyName:
	case kAudioObjectPropertyManufacturer:
	case kAudioObjectPropertyElementName:
	case kAudioPlugInPropertyResourceBundle:
	case kAudioDevicePropertyDeviceUID:
	case kAudioDevicePropertyModelUID:
		*outDataSize = sizeof(CFStringRef);
		return 0;
	}
	if (inObjectID == kObjectID_PlugIn) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyOwnedObjects:
		case kAudioPlugInPropertyDeviceList:
			*outDataSize = sizeof(AudioObjectID);
			return 0;
		case kAudioPlugInPropertyBoxList:
			*outDataSize = 0;
			return 0;
		case kAudioPlugInPropertyTranslateUIDToDevice:
			*outDataSize = sizeof(AudioObjectID);
			return 0;
		}
	} else if (inObjectID == kObjectID_Device) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyOwnedObjects:
		case kAudioDevicePropertyStreams:
			*outDataSize = (inAddress->mScope == kAudioObjectPropertyScopeGlobal ? 2 : 1) * sizeof(AudioObjectID);
			return 0;
		case kAudioObjectPropertyControlList:
			*outDataSize = 0;
			return 0;
		case kAudioDevicePropertyRelatedDevices:
			*outDataSize = sizeof(AudioObjectID);
			return 0;
		case kAudioDevicePropertyTransportType:
		case kAudioDevicePropertyClockDomain:
		case kAudioDevicePropertyDeviceIsAlive:
		case kAudioDevicePropertyDeviceIsRunning:
		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
		case kAudioDevicePropertyLatency:
		case kAudioDevicePropertySafetyOffset:
		case kAudioDevicePropertyIsHidden:
		case kAudioDevicePropertyZeroTimeStampPeriod:
			*outDataSize = sizeof(UInt32);
			return 0;
		case kAudioDevicePropertyNominalSampleRate:
			*outDataSize = sizeof(Float64);
			return 0;
		case kAudioDevicePropertyAvailableNominalSampleRates:
			*outDataSize = kNumRates * sizeof(AudioValueRange);
			return 0;
		case kAudioDevicePropertyPreferredChannelsForStereo:
			*outDataSize = 2 * sizeof(UInt32);
			return 0;
		case kAudioDevicePropertyPreferredChannelLayout:
			*outDataSize = (UInt32)kLayoutSize;
			return 0;
		}
	} else if (is_stream(inObjectID)) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyOwnedObjects:
			*outDataSize = 0;
			return 0;
		case kAudioStreamPropertyIsActive:
		case kAudioStreamPropertyDirection:
		case kAudioStreamPropertyTerminalType:
		case kAudioStreamPropertyStartingChannel:
		case kAudioStreamPropertyLatency:
			*outDataSize = sizeof(UInt32);
			return 0;
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
			*outDataSize = sizeof(AudioStreamBasicDescription);
			return 0;
		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
			*outDataSize = kNumRates * sizeof(AudioStreamRangedDescription);
			return 0;
		}
	}
	return kAudioHardwareUnknownPropertyError;
}

#define NEED(sz)	do { if (inDataSize < (sz)) return kAudioHardwareBadPropertySizeError; *outDataSize = (UInt32)(sz); } while (0)

static OSStatus X32USB_GetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, UInt32* outDataSize, void* outData) {
	UInt32 i, n;
	AudioObjectID* ids = (AudioObjectID*)outData;

	if (inDriver != gDriverRef || inAddress == NULL || outDataSize == NULL || outData == NULL) return kAudioHardwareIllegalOperationError;
	if (!X32USB_HasProperty(inDriver, inObjectID, inClientProcessID, inAddress)) return kAudioHardwareUnknownPropertyError;
	//
	// plug-in
	if (inObjectID == kObjectID_PlugIn) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
			NEED(sizeof(AudioClassID));
			*(AudioClassID*)outData = kAudioObjectClassID;
			return 0;
		case kAudioObjectPropertyClass:
			NEED(sizeof(AudioClassID));
			*(AudioClassID*)outData = kAudioPlugInClassID;
			return 0;
		case kAudioObjectPropertyOwner:
			NEED(sizeof(AudioObjectID));
			*(AudioObjectID*)outData = kAudioObjectUnknown;
			return 0;
		case kAudioObjectPropertyManufacturer:
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFSTR(kManufacturer);
			return 0;
		case kAudioObjectPropertyOwnedObjects:
		case kAudioPlugInPropertyDeviceList:
			*outDataSize = 0;
			if (inDataSize >= sizeof(AudioObjectID)) {
				ids[0] = kObjectID_Device;
				*outDataSize = sizeof(AudioObjectID);
			}
			return 0;
		case kAudioPlugInPropertyBoxList:
			*outDataSize = 0;
			return 0;
		case kAudioPlugInPropertyTranslateUIDToDevice:
			NEED(sizeof(AudioObjectID));
			if (inQualifierDataSize != sizeof(CFStringRef) || inQualifierData == NULL) return kAudioHardwareBadPropertySizeError;
			*(AudioObjectID*)outData = CFStringCompare(*(CFStringRef*)inQualifierData, CFSTR(kDeviceUID), 0) == kCFCompareEqualTo
									? kObjectID_Device : kAudioObjectUnknown;
			return 0;
		case kAudioPlugInPropertyResourceBundle:
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFSTR("");
			return 0;
		}
		return kAudioHardwareUnknownPropertyError;
	}
	//
	// device
	if (inObjectID == kObjectID_Device) {
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
			NEED(sizeof(AudioClassID));
			*(AudioClassID*)outData = kAudioObjectClassID;
			return 0;
		case kAudioObjectPropertyClass:
			NEED(sizeof(AudioClassID));
			*(AudioClassID*)outData = kAudioDeviceClassID;
			return 0;
		case kAudioObjectPropertyOwner:
			NEED(sizeof(AudioObjectID));
			*(AudioObjectID*)outData = kObjectID_PlugIn;
			return 0;
		case kAudioObjectPropertyName:
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFSTR(kDeviceName);
			return 0;
		case kAudioObjectPropertyManufacturer:
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFSTR(kManufacturer);
			return 0;
		case kAudioObjectPropertyElementName:
			// input channels carry the X32 card outputs, output channels feed the X32 card inputs
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFStringCreateWithFormat(NULL, NULL,
					inAddress->mScope == kAudioObjectPropertyScopeInput ? CFSTR("Card Out %u") : CFSTR("Card In %u"),
					(unsigned)inAddress->mElement);
			return 0;
		case kAudioObjectPropertyOwnedObjects:
		case kAudioDevicePropertyStreams:
			n = 0;
			if (inAddress->mScope != kAudioObjectPropertyScopeOutput && inDataSize >= (n + 1) * sizeof(AudioObjectID))
				ids[n++] = kObjectID_Stream_Input;
			if (inAddress->mScope != kAudioObjectPropertyScopeInput && inDataSize >= (n + 1) * sizeof(AudioObjectID))
				ids[n++] = kObjectID_Stream_Output;
			*outDataSize = n * sizeof(AudioObjectID);
			return 0;
		case kAudioObjectPropertyControlList:
			*outDataSize = 0;
			return 0;
		case kAudioDevicePropertyDeviceUID:
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFSTR(kDeviceUID);
			return 0;
		case kAudioDevicePropertyModelUID:
			NEED(sizeof(CFStringRef));
			*(CFStringRef*)outData = CFSTR(kDeviceModelUID);
			return 0;
		case kAudioDevicePropertyTransportType:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = kAudioDeviceTransportTypeVirtual;
			return 0;
		case kAudioDevicePropertyRelatedDevices:
			*outDataSize = 0;
			if (inDataSize >= sizeof(AudioObjectID)) {
				ids[0] = kObjectID_Device;
				*outDataSize = sizeof(AudioObjectID);
			}
			return 0;
		case kAudioDevicePropertyClockDomain:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = 0;
			return 0;
		case kAudioDevicePropertyDeviceIsAlive:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = 1;
			return 0;
		case kAudioDevicePropertyDeviceIsRunning:
			NEED(sizeof(UInt32));
			pthread_mutex_lock(&gIOMutex);
			*(UInt32*)outData = gIOCount > 0;
			pthread_mutex_unlock(&gIOMutex);
			return 0;
		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = 1;
			return 0;
		case kAudioDevicePropertyLatency:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = kLatencyFrames;
			return 0;
		case kAudioDevicePropertySafetyOffset:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = inAddress->mScope == kAudioObjectPropertyScopeInput ? XUT_SAFETY_FRAMES : kOutputSafetyFrames;
			return 0;
		case kAudioDevicePropertyNominalSampleRate:
			NEED(sizeof(Float64));
			*(Float64*)outData = current_rate();
			return 0;
		case kAudioDevicePropertyAvailableNominalSampleRates:
			n = inDataSize / sizeof(AudioValueRange);
			if (n > kNumRates) n = kNumRates;
			for (i = 0; i < n; i++) {
				((AudioValueRange*)outData)[i].mMinimum = kRates[i];
				((AudioValueRange*)outData)[i].mMaximum = kRates[i];
			}
			*outDataSize = n * sizeof(AudioValueRange);
			return 0;
		case kAudioDevicePropertyIsHidden:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = 0;
			return 0;
		case kAudioDevicePropertyPreferredChannelsForStereo:
			NEED(2 * sizeof(UInt32));
			((UInt32*)outData)[0] = 1;
			((UInt32*)outData)[1] = 2;
			return 0;
		case kAudioDevicePropertyPreferredChannelLayout: {
			AudioChannelLayout* layout = (AudioChannelLayout*)outData;
			NEED(kLayoutSize);
			memset(layout, 0, kLayoutSize);
			layout->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
			layout->mNumberChannelDescriptions = kChannels;
			for (i = 0; i < kChannels; i++) {
				layout->mChannelDescriptions[i].mChannelLabel =
						i == 0 ? kAudioChannelLabel_Left : i == 1 ? kAudioChannelLabel_Right : kAudioChannelLabel_Discrete_0 + i;
			}
			return 0;
		}
		case kAudioDevicePropertyZeroTimeStampPeriod:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = kZeroTimeStampPeriod;
			return 0;
		}
		return kAudioHardwareUnknownPropertyError;
	}
	//
	// streams
	if (is_stream(inObjectID)) {
		Boolean input = inObjectID == kObjectID_Stream_Input;
		switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
			NEED(sizeof(AudioClassID));
			*(AudioClassID*)outData = kAudioObjectClassID;
			return 0;
		case kAudioObjectPropertyClass:
			NEED(sizeof(AudioClassID));
			*(AudioClassID*)outData = kAudioStreamClassID;
			return 0;
		case kAudioObjectPropertyOwner:
			NEED(sizeof(AudioObjectID));
			*(AudioObjectID*)outData = kObjectID_Device;
			return 0;
		case kAudioObjectPropertyOwnedObjects:
			*outDataSize = 0;
			return 0;
		case kAudioStreamPropertyIsActive:
			NEED(sizeof(UInt32));
			pthread_mutex_lock(&gStateMutex);
			*(UInt32*)outData = input ? gInputActive : gOutputActive;
			pthread_mutex_unlock(&gStateMutex);
			return 0;
		case kAudioStreamPropertyDirection:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = input ? 1 : 0;
			return 0;
		case kAudioStreamPropertyTerminalType:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = kAudioStreamTerminalTypeLine;
			return 0;
		case kAudioStreamPropertyStartingChannel:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = 1;
			return 0;
		case kAudioStreamPropertyLatency:
			NEED(sizeof(UInt32));
			*(UInt32*)outData = 0;
			return 0;
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
			NEED(sizeof(AudioStreamBasicDescription));
			fill_format((AudioStreamBasicDescription*)outData, current_rate());
			return 0;
		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
			n = inDataSize / sizeof(AudioStreamRangedDescription);
			if (n > kNumRates) n = kNumRates;
			for (i = 0; i < n; i++) {
				AudioStreamRangedDescription* d = &((AudioStreamRangedDescription*)outData)[i];
				fill_format(&d->mFormat, kRates[i]);
				d->mSampleRateRange.mMinimum = kRates[i];
				d->mSampleRateRange.mMaximum = kRates[i];
			}
			*outDataSize = n * sizeof(AudioStreamRangedDescription);
			return 0;
		}
	}
	return kAudioHardwareUnknownPropertyError;
}

static OSStatus request_rate(Float64 rate) {
	if (!valid_rate(rate)) return kAudioDeviceUnsupportedFormatError;
	if (rate != current_rate() && gHost) {
		// change asked by the user (Audio MIDI Setup, an app); if the emulator
		// runs at another rate, the driver will follow the emulator again
		gHost->RequestDeviceConfigurationChange(gHost, kObjectID_Device, (UInt64)rate, NULL);
	}
	return 0;
}

static OSStatus X32USB_SetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, const void* inData) {
	(void)inQualifierDataSize; (void)inQualifierData;
	if (inDriver != gDriverRef || inAddress == NULL || inData == NULL) return kAudioHardwareIllegalOperationError;
	if (!X32USB_HasProperty(inDriver, inObjectID, inClientProcessID, inAddress)) return kAudioHardwareUnknownPropertyError;
	if (inObjectID == kObjectID_Device && inAddress->mSelector == kAudioDevicePropertyNominalSampleRate) {
		if (inDataSize != sizeof(Float64)) return kAudioHardwareBadPropertySizeError;
		return request_rate(*(const Float64*)inData);
	}
	if (is_stream(inObjectID)) {
		switch (inAddress->mSelector) {
		case kAudioStreamPropertyIsActive: {
			AudioObjectPropertyAddress changed = {kAudioStreamPropertyIsActive, kAudioObjectPropertyScopeGlobal, 0 /* main element */};
			Boolean active;
			if (inDataSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
			active = *(const UInt32*)inData != 0;
			pthread_mutex_lock(&gStateMutex);
			if (inObjectID == kObjectID_Stream_Input) gInputActive = active;
			else gOutputActive = active;
			pthread_mutex_unlock(&gStateMutex);
			if (gHost) gHost->PropertiesChanged(gHost, inObjectID, 1, &changed);
			return 0;
		}
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat: {
			const AudioStreamBasicDescription* f = (const AudioStreamBasicDescription*)inData;
			if (inDataSize != sizeof(AudioStreamBasicDescription)) return kAudioHardwareBadPropertySizeError;
			if (f->mFormatID != kAudioFormatLinearPCM || f->mChannelsPerFrame != kChannels || f->mBitsPerChannel != 32
					|| !(f->mFormatFlags & kAudioFormatFlagIsFloat)) return kAudioDeviceUnsupportedFormatError;
			return request_rate(f->mSampleRate);
		}
		}
	}
	return kAudioHardwareUnknownPropertyError;
}

//
// IO
//
static OSStatus X32USB_StartIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID) {
	(void)inClientID;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	pthread_mutex_lock(&gIOMutex);
	if (gIOCount == UINT64_MAX) {
		pthread_mutex_unlock(&gIOMutex);
		return kAudioHardwareIllegalOperationError;
	}
	if (gIOCount++ == 0) {
		gNumberTimeStamps = 0;
		gAnchorHostTime = mach_absolute_time();
		if (gTransportOK) XUT_start(&gTransport, gAnchorHostTime, (uint32_t)current_rate());
	}
	pthread_mutex_unlock(&gIOMutex);
	return 0;
}

static OSStatus X32USB_StopIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID) {
	(void)inClientID;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	pthread_mutex_lock(&gIOMutex);
	if (gIOCount == 0) {
		pthread_mutex_unlock(&gIOMutex);
		return kAudioHardwareIllegalOperationError;
	}
	if (--gIOCount == 0 && gTransportOK) XUT_stop(&gTransport);
	pthread_mutex_unlock(&gIOMutex);
	return 0;
}

static OSStatus X32USB_GetZeroTimeStamp(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, Float64* outSampleTime, UInt64* outHostTime, UInt64* outSeed) {
	UInt64 now, ticksPerPeriod, next;
	(void)inClientID;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	pthread_mutex_lock(&gIOMutex);
	now = mach_absolute_time();
	ticksPerPeriod = (UInt64)(gHostTicksPerFrame * kZeroTimeStampPeriod);
	next = gAnchorHostTime + (gNumberTimeStamps + 1) * ticksPerPeriod;
	if (next <= now) ++gNumberTimeStamps;
	*outSampleTime = (Float64)(gNumberTimeStamps * kZeroTimeStampPeriod);
	*outHostTime = gAnchorHostTime + gNumberTimeStamps * ticksPerPeriod;
	*outSeed = 1;
	pthread_mutex_unlock(&gIOMutex);
	return 0;
}

static OSStatus X32USB_WillDoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, Boolean* outWillDo, Boolean* outWillDoInPlace) {
	(void)inClientID;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	*outWillDo = inOperationID == kAudioServerPlugInIOOperationReadInput || inOperationID == kAudioServerPlugInIOOperationWriteMix;
	*outWillDoInPlace = true;
	return 0;
}

static OSStatus X32USB_BeginIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo) {
	(void)inClientID; (void)inOperationID; (void)inIOBufferFrameSize; (void)inIOCycleInfo;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	return 0;
}

static OSStatus X32USB_DoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, AudioObjectID inStreamObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo, void* ioMainBuffer, void* ioSecondaryBuffer) {
	(void)inClientID; (void)ioSecondaryBuffer;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	if (ioMainBuffer == NULL) return 0;
	if (inOperationID == kAudioServerPlugInIOOperationReadInput && inStreamObjectID == kObjectID_Stream_Input) {
		if (gTransportOK && gInputActive)
			XUT_read_input(&gTransport, (int64_t)inIOCycleInfo->mInputTime.mSampleTime, (float*)ioMainBuffer, inIOBufferFrameSize);
		else
			memset(ioMainBuffer, 0, inIOBufferFrameSize * kBytesPerFrame);
	} else if (inOperationID == kAudioServerPlugInIOOperationWriteMix && inStreamObjectID == kObjectID_Stream_Output) {
		if (gTransportOK && gOutputActive)
			XUT_write_output(&gTransport, (int64_t)inIOCycleInfo->mOutputTime.mSampleTime, (const float*)ioMainBuffer, inIOBufferFrameSize);
	}
	return 0;
}

static OSStatus X32USB_EndIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo) {
	(void)inClientID; (void)inOperationID; (void)inIOBufferFrameSize; (void)inIOCycleInfo;
	if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
	return 0;
}
