#include "ipffile.h"
#include "capsapi/Comtype.h"
#include "capsapi/CapsAPI.h"
#include "../../file_io.h"
#include <algorithm>
#include <utility>
#include <cstring>
#include <stdio.h>
#include <dlfcn.h>
#include "debug.h"
#define CAPSCALL



static bool ipfcapsAttempted = false;
void* libHandle = nullptr;

typedef SDWORD(CAPSCALL* CAPSINIT)(void);
static CAPSINIT pCAPSInit;
typedef SDWORD(CAPSCALL* CAPSADDIMAGE)(void);
static CAPSADDIMAGE pCAPSAddImage;
typedef SDWORD(CAPSCALL* CAPSREMIMAGE)(SDWORD);
static CAPSREMIMAGE pCAPSRemImage;
typedef SDWORD(CAPSCALL* CAPSLOCKIMAGE)(SDWORD, PCHAR);
static CAPSLOCKIMAGE pCAPSLockImage;
typedef SDWORD(CAPSCALL* CAPSUNLOCKIMAGE)(SDWORD);
static CAPSUNLOCKIMAGE pCAPSUnlockImage;
typedef SDWORD(CAPSCALL* CAPSLOADIMAGE)(SDWORD, UDWORD);
static CAPSLOADIMAGE pCAPSLoadImage;
typedef SDWORD(CAPSCALL* CAPSGETIMAGEINFO)(PCAPSIMAGEINFO, SDWORD);
static CAPSGETIMAGEINFO pCAPSGetImageInfo;
typedef SDWORD(CAPSCALL* CAPSLOCKTRACK)(void*, SDWORD, UDWORD, UDWORD, UDWORD);
static CAPSLOCKTRACK pCAPSLockTrack;
typedef SDWORD(CAPSCALL* CAPSUNLOCKTRACK)(SDWORD id, UDWORD cylinder, UDWORD head);
static CAPSUNLOCKTRACK pCAPSUnlockTrack;
typedef SDWORD(CAPSCALL* CAPSUNLOCKALLTRACKS)(SDWORD);
static CAPSUNLOCKALLTRACKS pCAPSUnlockAllTracks;


#define LOCK_FLAGS (DI_LOCK_INDEX    |DI_LOCK_DENVAR   |DI_LOCK_DENAUTO  |DI_LOCK_DENNOISE |DI_LOCK_NOISE    |DI_LOCK_NOISEREV |DI_LOCK_UPDATEFD |DI_LOCK_TYPE     |DI_LOCK_TRKBIT  )

#define DensityToNS(densityValue) ((2000ULL * densityValue) / 1000ULL)

void caps_free() {
	if (libHandle) dlclose(libHandle);
	libHandle = nullptr;
}

bool caps_init(void)
{
//	if ((ipfcapsAttempted) && (!libHandle)) return false;
	if (libHandle) return true;
	ipfcapsAttempted = true;
	                  

	libHandle = dlopen("/media/fat/linux/libcapsimg.so", RTLD_NOW);
	if (!libHandle) {
		fdd_debugf("Unable to load /media/fat/linux/libcapsimg.so error %s", dlerror());
		return false;
	}

	if (dlsym(libHandle, "CAPSLockImageMemory") == 0 || dlsym(libHandle, "CAPSGetVersionInfo") == 0) {
		caps_free();
		return false;
	}		
	pCAPSInit = (CAPSINIT)dlsym(libHandle, "CAPSInit");
	pCAPSAddImage = (CAPSADDIMAGE)dlsym(libHandle, "CAPSAddImage");
	pCAPSRemImage = (CAPSREMIMAGE)dlsym(libHandle, "CAPSRemImage");
	pCAPSLockImage = (CAPSLOCKIMAGE)dlsym(libHandle, "CAPSLockImage");
	pCAPSUnlockImage = (CAPSUNLOCKIMAGE)dlsym(libHandle, "CAPSUnlockImage");
	pCAPSLoadImage = (CAPSLOADIMAGE)dlsym(libHandle, "CAPSLoadImage");
	pCAPSGetImageInfo = (CAPSGETIMAGEINFO)dlsym(libHandle, "CAPSGetImageInfo");
	pCAPSLockTrack = (CAPSLOCKTRACK)dlsym(libHandle, "CAPSLockTrack");
	pCAPSUnlockTrack = (CAPSUNLOCKTRACK)dlsym(libHandle, "CAPSUnlockTrack");
	pCAPSUnlockAllTracks = (CAPSUNLOCKALLTRACKS)dlsym(libHandle, "CAPSUnlockAllTracks");

	if (pCAPSInit() != imgeOk) {
		caps_free();
		return false;
	}

	return true;
}


uint32_t IPFFile::firstTrack() { return m_minCylinder* numHeads(); }
uint32_t IPFFile::lastTrack() { return (m_maxCylinder * numHeads()) - 1; }
uint32_t IPFFile::numHeads() { return (m_maxHead - m_minHead) + 1; };


// This data just read wasn't used. We dont want to go out of sync.
void IPFFile::unFluxRead(const uint16_t* inputBuffer, uint32_t numWords) {
	if (!fluxReady()) return;
	m_rewindBuffer.resize(numWords);
	memcpy(m_rewindBuffer.data(), inputBuffer, numWords * sizeof(uint16_t));
}

// Open SCP file
bool IPFFile::openFile(const char* filename) {
	closeFile();

	if (!libHandle) return false;

	static char full_path[2100];
	sprintf(full_path, "%s/%s", getRootDir(), filename);

	SDWORD result = pCAPSLockImage(m_capsImageIndex, (PCHAR)full_path);
	if (result != imgeOk) {
		fdd_debugf("Unable to open %s (error %li)", filename, result);
		return false;
	}
	m_isOpen = true;

	CapsImageInfo info;
	if (pCAPSGetImageInfo(&info, m_capsImageIndex) != imgeOk) {
		closeFile();
		return false;
	}

	m_minHead = info.minhead;
	m_maxHead = info.maxhead;
	m_minCylinder = info.mincylinder;
	m_maxCylinder = info.maxcylinder;	
	if (pCAPSLoadImage(m_capsImageIndex, LOCK_FLAGS) != imgeOk) {
		closeFile();
		return false;
	}

	bool ret = selectTrack(firstTrack());
	if (!ret) {
		closeFile();
		return false;
	}
	return true;
}

// Change active track (this includes the head)
bool IPFFile::selectTrack(uint32_t track) {	
	if (track == m_currentTrack) return true;
	if (track < firstTrack()) return false;
	if (track >= lastTrack()) return false;
	m_currentTrack = 0xffff;
	m_rewindBuffer.clear();
	m_timeSoFar = 0;
	m_fluxTime = 0;
	m_timeSoFar = 0;
	uint32_t lastBufferPos = m_bufferPos;
	uint32_t lastBufferLength = m_trackBufferLength;

	// Decode the track
	if (decodeTrack(track)) {

		// Skip to a position within the data to simulate the fact that the disk is still spinning		
		uint64_t newSamplePos = lastBufferLength ? (lastBufferPos * m_trackBufferLength) / lastBufferLength : 0;
		if (newSamplePos >= m_trackBufferLength - 1) newSamplePos = 0;
		m_bufferPos = newSamplePos;
		m_markIndex = m_bufferPos == 0;
		m_currentTrack = track;
		return true;
	}
	return false;
}



// Provides a basic decoding. just enough
bool IPFFile::decodeTrack(uint32_t track) {
	CapsTrackInfoT2 trackInfo;
	memset(&trackInfo, 0, sizeof(trackInfo));
	trackInfo.type = 2;
	m_bufferPos = 0;
	
	// Close previous buffer	
	if (m_previousTrack>=0) pCAPSUnlockTrack(m_capsImageIndex, m_previousTrack / numHeads(), m_minHead + (m_previousTrack % numHeads()));
	m_previousTrack = -1;

	// Load track and convert to flux timing
	if (pCAPSLockTrack(&trackInfo, m_capsImageIndex, track / numHeads(), m_minHead + (track % numHeads()), LOCK_FLAGS) != imgeOk) return false;	
	m_previousTrack = track;

	m_trackBuffer = trackInfo.trackbuf;
	m_densityBuffer = trackInfo.timebuf;
	m_trackBufferLength = trackInfo.tracklen;
	m_densityBufferLength = trackInfo.timelen;

	if (trackInfo.trackbuf) {
		// Step 1: Work out the total time
		uint64_t timetotal = 0;
		if (trackInfo.timebuf && trackInfo.timelen) {
			for (UDWORD byte = 0; byte < trackInfo.timelen; byte++) timetotal += trackInfo.timebuf[byte];
			timetotal = DensityToNS(timetotal) * 8ULL;
		}
		else timetotal = 200000000ULL;
		m_densityCompensation = timetotal ? (200000000000ULL) / timetotal : 1000ULL;
	}
	else m_densityCompensation = 1000UL;

	m_markIndex = true;

	return (m_trackBufferLength>0) && (m_trackBuffer);
}

void IPFFile::closeFile() {
	m_bufferPos = 0;
	m_fluxTime = 0;
	m_markIndex = false;
	m_previousTrack = -1;
	m_trackBuffer= NULL;
	m_densityBuffer = NULL;
	m_trackBufferLength = 0;
	m_densityBufferLength = 0;
	m_timeSoFar = 0;

	if (m_isOpen) {
		pCAPSUnlockAllTracks(m_capsImageIndex);
		pCAPSUnlockImage(m_capsImageIndex);
		m_isOpen = false;
	}
	m_currentTrack = 0xffff;
	m_rewindBuffer.clear();
	m_minHead = 0;
	m_maxHead = 0;
	m_minCylinder = 0;
	m_maxCylinder = 0;
}

bool IPFFile::fluxReady() {
	return (m_currentTrack != 0xFFFF) && (m_trackBufferLength > 0) && (m_trackBuffer);
}

// Read some flux data from the file
bool IPFFile::fluxRead(uint16_t* outputBuffer, uint32_t numWords) {	
	if (m_currentTrack < firstTrack()) return false;
	if (m_currentTrack >= lastTrack()) return false;

	// Rewind!
	if (m_rewindBuffer.size() == numWords) {
		memcpy(outputBuffer, m_rewindBuffer.data(), numWords * sizeof(uint16_t));
		m_rewindBuffer.clear();
		return true;
	}
	
	uint8_t* bytesOut = (uint8_t*)outputBuffer;
	uint32_t bytesLeft = numWords * 2UL;

	while (bytesLeft) {

		// Encode flux transition information
		if (m_fluxTime) {
			while (m_fluxTime > 36370LL) {  // approx 255				
				if (!fluxReady()) return false;
				// 1 byte left, encode a single tick delay
				if (bytesLeft == 1) {
					*bytesOut = 1; bytesLeft--; bytesOut++;
					m_fluxTime -= 142LL;
					if (m_fluxTime < 1) m_fluxTime = 1;
					return true;
				}
				// 2 bytes left
				*bytesOut = 2;   // signal the next byte is a delay only, not transition
				bytesLeft--; bytesOut++;
				m_fluxTime -= 142LL;    // even this uses one tick
				uint8_t convertedTime = (uint8_t)std::min(std::max(3LL, ((m_fluxTime + 71LL) * 7LL) / 1000LL), 255LL);
				*bytesOut = convertedTime;
				bytesOut++; bytesLeft--;
				m_fluxTime -= (((int64_t)convertedTime) * 1000LL) / 7LL;
				if (m_fluxTime < 1) m_fluxTime = 1;
				if (bytesLeft < 1) return true;
			}
			// Encode flux transition			
			if (bytesLeft) {
				uint8_t convertedTime = (uint8_t)std::min(std::max(3LL, ((m_fluxTime + 71LL) * 7LL) / 1000LL), 255LL);
				m_fluxTime -= (((int64_t)convertedTime) * 1000LL) / 7LL;
				*bytesOut = convertedTime;
				bytesOut++; bytesLeft--;				
			}
			else return true;
			m_timeSoFar = m_fluxTime;
			m_fluxTime = 0;
		}

		// Need more data?
		if (m_bufferPos >= m_trackBufferLength) {
			if (!decodeTrack(m_previousTrack >= 0 ? m_previousTrack : m_currentTrack)) return false;
		}

		// INDEX
		if (m_markIndex) {
			// We need to send INDEX, but theres a residual from the previous loop. Needs encoding so INDEX appears at the right position
			if (m_timeSoFar) {
				while (m_timeSoFar > 142LL) {
					if (!fluxReady()) return false;
					// 1 byte left, encode a single tick delay
					if (bytesLeft == 1) {
						*bytesOut = 1; bytesLeft--; bytesOut++;
						m_timeSoFar -= 142LL;
						return true;
					}
					// More than 1 byte remaining!
					// Less than 2 ticks (1.5) worth of time remaining?
					if (m_timeSoFar < 245LL) {
						*bytesOut = 1;   // Single delay
						bytesLeft--; bytesOut++;
						m_timeSoFar = 0;
					}
					else {
						// Less than 3 ticks (2.5 worth)
						if (m_timeSoFar < 355LL) {
							*bytesOut = 1;    bytesOut++;
							*bytesOut = 1;    bytesOut++;
							bytesLeft -= 2;
							m_timeSoFar = 0;
						}
						else {
							// 3 or more ticks
							*bytesOut = 2;   // signal the next byte is a delay only, not transition
							bytesLeft--; bytesOut++;
							m_timeSoFar -= 142LL;    // even this uses one tick
							uint8_t convertedTime = (uint8_t)std::min(std::max(3LL, ((m_timeSoFar + 71LL) * 7LL) / 1000LL), 255LL);
							*bytesOut = convertedTime;
							bytesOut++; bytesLeft--;
							m_timeSoFar -= (((int64_t)convertedTime) * 1000LL) / 7LL;
							if (convertedTime <= 3) m_timeSoFar = 0; // just incase
						}
					}

					// Stop if out of data space
					if (bytesLeft < 1) return true;
				}
				m_timeSoFar = 0;
			}

			m_markIndex = false;
			*bytesOut = 0; // INDEX
			bytesOut++; bytesLeft--;
			if (bytesLeft < 1) return true;
		}
			
		// Bit-level
		const uint32_t bytePos = m_bufferPos / 8;
		UDWORD density = ((m_densityBuffer) && (m_densityBufferLength) && (bytePos < m_densityBufferLength)) ? m_densityBuffer[bytePos] : 1000ULL;
		m_timeSoFar += DensityToNS(density * m_densityCompensation / 1000ULL);

		// Bit detected
		if (m_trackBuffer[bytePos] & (1 << (7 - (m_bufferPos & 7)))) {
			m_fluxTime = std::max(1LL, m_timeSoFar);
			m_timeSoFar = 0LL;
		}
		m_bufferPos++;
	}

	return true;
}


IPFFile::IPFFile() {
	m_isOpen = false;
	if (caps_init()) m_capsImageIndex = pCAPSAddImage();
	closeFile();
}

IPFFile::~IPFFile() {
	closeFile();	
	if (libHandle) pCAPSRemImage(m_capsImageIndex);
}


