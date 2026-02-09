#ifndef __MINIMIG_FDD_H__
#define __MINIMIG_FDD_H__

#include "../../file_io.h"

// floppy disk interface defs
#define CMD_RDTRK 0x01
#define CMD_WRTRK 0x02

// floppy status
#define DSK_INSERTED 0x001 /*disk is inserted*/
#define DSK_WRITABLE 0x010 /*disk is writable*/
#define DSK_FLUXMODE 0x100 /*disk data is in flux mode*/

#define FLUXTYPE_NONE 0x00
#define FLUXTYPE_IPF  0x01
#define FLUXTYPE_SCP  0x02

#define MAX_TRACKS (83*2)


class FluxFile {
public:
	// Open Flux based file
	virtual bool openFile(const char* filename) = 0;
	virtual void closeFile() = 0;

	// Is this available to be read from
	virtual bool fluxReady() = 0;

	// Change active track (this includes the head)
	virtual bool selectTrack(uint32_t track) = 0;

	// Read some flux data from the file
	virtual bool fluxRead(uint16_t* outputBuffer, uint32_t numWords) = 0;
	// This data just read wasn't used. We dont want to go out of sync.
	virtual void unFluxRead(const uint16_t* inputBuffer, uint32_t numWords) = 0;

	virtual uint32_t firstTrack() = 0;
	virtual uint32_t lastTrack() = 0;
	virtual uint32_t numHeads() = 0;

	virtual ~FluxFile() {};
};

typedef struct
{
	fileTYPE      file;
	uint16_t      status; /*status of floppy*/
	unsigned char tracks; /*number of tracks*/
	unsigned char sector_offset; /*sector offset to handle tricky loaders*/
	unsigned char track; /*current track*/
	unsigned char track_prev; /*previous track*/
	char          name[1024]; /*floppy name*/
	uint16_t	  lastBit;   /* last mfm word used for clock bit calculation */
	FluxFile*     fluxFile;  /* flux reader class */
} adfTYPE;

extern unsigned char drives;
extern adfTYPE df[4];

void UpdateDriveStatus(void);
void HandleFDD(unsigned char c1, unsigned char c2);
void InsertFloppy(adfTYPE *drive, char* path);

#endif

