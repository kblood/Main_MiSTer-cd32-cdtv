#ifndef __MINIMIG_IPF_H__
#define __MINIMIG_IPF_H__

#include "../../file_io.h"
#include "minimig_fdd.h"
#include <stdint.h>
#include <vector>
#include <unordered_map>


class IPFFile : public FluxFile {
private:
	signed long m_capsImageIndex;
	bool m_isOpen;

	uint16_t m_minHead, m_maxHead, m_minCylinder, m_maxCylinder;	
	std::vector<uint16_t> m_rewindBuffer;	
	uint16_t m_currentTrack;    	
	uint32_t m_bufferPos;	
	uint8_t* m_trackBuffer;
	unsigned long* m_densityBuffer;
	uint32_t m_trackBufferLength;
	uint32_t m_densityBufferLength;
	uint64_t m_densityCompensation;
	int32_t m_previousTrack;
	int64_t m_fluxTime;
	int64_t m_timeSoFar;
	bool m_markIndex;

	// Provides a basic decoding. just enough
	bool decodeTrack(uint32_t track);

public:
	IPFFile();
    virtual ~IPFFile();

	// Open SCP file
	virtual bool openFile(const char* filename) override;
	virtual void closeFile() override;

	virtual bool fluxReady() override;
		
	// Change active track (this includes the head)
	virtual bool selectTrack(uint32_t track) override;

	virtual bool fluxRead(uint16_t* outputBuffer, uint32_t numWords) override;

	// This data just read wasn't used. We dont want to go out of sync.
	virtual void unFluxRead(const uint16_t* inputBuffer, uint32_t numWords) override;
	
	virtual uint32_t firstTrack() override;
	virtual uint32_t lastTrack() override;
	virtual uint32_t numHeads() override;
};

bool caps_init(void);

#endif // __MINIMIG_IPF_H__