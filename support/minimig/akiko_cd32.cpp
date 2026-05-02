// CD32 Akiko native-mode bridge — Main_MiSTer side (M3 MVP).
//
// Mirrors the WinUAE Akiko CD command interpreter (akiko.cpp @ SHA 2c7f8581)
// in just enough fidelity to boot Cannon Fodder. Talks to the FPGA bridge in
// rtl/akiko_hps_bridge.v via UIO class 0xF400 (io_din[15:9] == 7'b1111_010,
// see hps_ext.v:127). Polled once per frame from user_io.cpp.
//
// Protocol (per hps_ext.v):
//   Status poll: spi_w(0x63) → bit[11] = akiko_req (command framed & ready).
//   Read txn :   spi8(0x62); spi32_w(0xF400); { byte = spi_w(0); }*; DisableIO()
//                ends txn → FPGA pulses cmd_done (releases command framer).
//   Write txn:   spi8(0x61); spi32_w(0xF400); { spi_w(byte); }*; DisableIO()
//                ends txn → FPGA pulses result_done (kicks RX DMA).
// Read and write must NEVER be mixed in the same EnableIO/DisableIO scope.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdbool.h>

#include "../../spi.h"
#include "../../user_io.h"
#include "../../ide.h"
#include "../../ide_cdrom.h"
#include "akiko_cd32.h"

// -----------------------------------------------------------------------------
// Debug gating
// -----------------------------------------------------------------------------
// Forward declaration so the debug macro can use it before its definition.
static void akiko_diag(const char *fmt, ...);

// Temporarily forced on for M5 hardware boot diagnosis (Cannon Fodder no-CD).
// Revert by removing this define once the bridge is observed working.
#define AKIKO_CD32_DEBUG 1
#ifdef AKIKO_CD32_DEBUG
	// Route through akiko_diag so output reaches /tmp/akiko_dbg.log instead
	// of stdout, which Minimig redirects/silences after init.
	#define akiko_dbg(fmt, ...) akiko_diag("[akiko] " fmt, ##__VA_ARGS__)
#else
	#define akiko_dbg(...) do { } while (0)
#endif

// -----------------------------------------------------------------------------
// WinUAE-derived constants (akiko.cpp)
// -----------------------------------------------------------------------------

// Bridge address class (matches hps_ext.v:127 → akiko_cs).
#define AKIKO_BRIDGE_ADDR  0xF400

// Status-poll cmd byte. Returns one 16-bit word; bit[11] = akiko_req,
// bit[10] = akiko_sec_req (M4 PBX wants a sector pushed).
#define AKIKO_STATUS_CMD     0x63
#define AKIKO_STATUS_REQ     (1u << 11)
#define AKIKO_STATUS_SEC_REQ (1u << 10)

// Sub-channel selector inside the 0xF400 class: io_din[8] = 1 selects the
// sector channel (hps_ext.v:135, akiko_cs_sec). 0xF400 | 0x100 = 0xF500.
#define AKIKO_SECTOR_ADDR  0xF500

// Trace sub-channel: io_din[7] = 1 selects the akiko_bus_trace ring buffer
// (hps_ext.v: akiko_cs_trace). Each entry is 4 bytes:
//   byte 0: bit7 = 1 if write / 0 if read; bits6:0 = addr[7:1] within the
//           akiko window ($B80000-$B800FE in 2-byte stride)
//   byte 1: data[7:0]
//   byte 2: data[15:8]
//   byte 3: 0xFF if entry valid, 0x00 if ring empty (stop draining)
#define AKIKO_TRACE_ADDR  0xF480

// PBX sector size (raw Mode-1/Mode-2 frame).
#define AKIKO_SECTOR_BYTES 2352

// Per-opcode command lengths *excluding* the trailing checksum byte.
// akiko.cpp:1136 — entries < 0 are "reserved/unsupported".
static const int command_lengths[16] = {
	1, 2, 1, 1, 12, 2, 1, 1, 4, 1, 2, -1, -1, -1, -1, -1
};

// Status / error nibbles (akiko.cpp). Place-holder values — we don't model
// door open/close yet.
#define CH_ERR_OK          0x00
#define CH_ERR_BADCOMMAND  0x80
#define CH_ERR_NODISK      0x80
#define CH_ERR_CHECKSUM    0x88
#define CDS_PLAYING        0x80

// Drive firmware string returned by INFO (opcode 0x07). Exactly 18 chars,
// matches akiko.cpp:176 #define FIRMWAREVERSION "CHINON  O-658-2 24".
#define AKIKO_FIRMWARE     "CHINON  O-658-2 24"

// Max bytes we can possibly drain in a single command frame: largest table
// entry is 12 (opcode 0x04) plus the checksum = 13. Round up to 32 because
// the FPGA's buffer is 32 bytes and we're paranoid about tooling churn.
#define AKIKO_CMD_MAX      32

// -----------------------------------------------------------------------------
// Module state
// -----------------------------------------------------------------------------

// Auto-init handshake (akiko.cpp:1388-1397):
//   0 = drive cold, host hasn't seen media yet.
//   1 = we sent the {0x0a, present} media-status response.
//   2 = host sent INFO (0x07); fully initialised.
static uint8_t  cd_initialized      = 0;
static uint8_t  cd_paused           = 0;
static uint8_t  cd_playing          = 0;       // unused in M3, set by 0x04 (audio)
static uint8_t  cd_led_state        = 0;
// WinUAE akiko.cpp:500 defaults cdrom_door=1 and never clears it. The
// "door" byte is actually a status mask whose bit 0 = CHERR_DISKPRESENT
// (AROS chinon.h:63). With cd_door=0 every INFO/STATUS/STOP/PAUSE/MULTI
// response advertises "no disk present" -- the CD32 BIOS then halts at
// the spinning-CD splash polling CDINTREQ forever, which is exactly what
// we observed. Initial value MUST be 1.
static uint8_t  cd_door             = 1;
static uint32_t cd_play_start_lba   = 0;       // recorded for M4 (real audio/data)
static uint32_t cd_play_end_lba     = 0;

// M4 PBX state: cd_data_lba_base is set when cmd 0x04 is issued in DATA mode
// (cmd[7] bit 7 = 1). On each FPGA sec_req, we read the FPGA's sector_counter
// and fetch LBA = base + counter. -1 = no data read in progress (push zeros).
static int32_t  cd_data_lba_base    = -1;

// Last mounted state — used to re-arm the auto-init when a disc is swapped.
static bool     cd_last_mounted     = false;

// One-shot: push a fresh media-status frame on the next poll *after* INFO
// has completed. Mimics WinUAE's mediachanged-still-set-after-init quirk
// (akiko.cpp:1399 fires once cd_initialized reaches 2 if mediachanged was
// set at boot). Without this, BIOS gets the LED+INFO acks but never sees
// the second media_status that would unblock its post-init MULTI scan.
static uint8_t  cd_post_info_media_push_pending = 0;

// TOC streaming state. After cmd_info completes we proactively push TOC
// entries (cmd 0x06 / cdrom_return_toc_entry) one per poll until the BIOS
// has seen each point TOC_REPEAT times. WinUAE pushes one per video frame
// (akiko.cpp:1438-1440). Without this, CD32 BIOS sits at the spinning-CD
// splash forever — it never sends MULTI/READ until TOC is known.
//   AKIKO_TOC_MAX_POINTS = 0xA0 + 0xA1 + 0xA2 + up to 99 tracks; cap at 16
//   for our M5 use case (Cannon Fodder = 2 tracks → 5 points).
#define AKIKO_TOC_REPEAT       3
#define AKIKO_TOC_PUSH_PERIOD  4    // one entry every Nth poll = ~16ms-ish
#define AKIKO_TOC_MAX_POINTS   16
static uint8_t toc_buffer[AKIKO_TOC_MAX_POINTS * 13];
static uint8_t toc_point_count      = 0;
static int16_t toc_push_idx         = -1;   // -1 = idle; else next slot in 3x sequence
static int     toc_push_throttle    = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static inline uint32_t msf_to_lba(uint8_t m, uint8_t s, uint8_t f)
{
	// Standard CDDA MSF → LBA conversion with 2-second pre-gap.
	return (((uint32_t)m * 60u + s) * 75u + f) - 150u;
}

static inline uint8_t bcd_to_bin(uint8_t b)
{
	return (uint8_t)(((b >> 4) * 10u) + (b & 0x0fu));
}

static inline uint8_t bin_to_bcd(uint8_t v)
{
	return (uint8_t)(((v / 10u) << 4) | (v % 10u));
}

// CD mounted? Use the first IDE port/drive that has a CD. ide.h shows
// ide_inst[2].drive[2] with .present and .cd flags.
static bool cd_is_mounted(void)
{
	for (int p = 0; p < 2; p++) {
		for (int d = 0; d < 2; d++) {
			if (ide_inst[p].drive[d].present && ide_inst[p].drive[d].cd) {
				return true;
			}
		}
	}
	return false;
}

// Find the first mounted CD drive (used by M4 sector fetch). NULL if none.
static drive_t *cd_find_drive(void)
{
	for (int p = 0; p < 2; p++) {
		for (int d = 0; d < 2; d++) {
			if (ide_inst[p].drive[d].present && ide_inst[p].drive[d].cd) {
				return &ide_inst[p].drive[d];
			}
		}
	}
	return NULL;
}

// Read the bridge status word. Mirrors ide_check() but kept private to avoid
// clobbering the IDE poll's read of the same word.
static uint16_t akiko_read_status(void)
{
	uint16_t res;
	EnableIO();
	res = spi_w(AKIKO_STATUS_CMD);
	if (!res) res = (uint8_t)spi_w(0);
	DisableIO();
	return res;
}

// -----------------------------------------------------------------------------
// Bridge transactions
// -----------------------------------------------------------------------------

// Drain up to one framed command from the bridge. Returns the number of bytes
// read. The first byte (opcode) tells us how many bytes the rest of the frame
// is — we read exactly cmd_length+1 (incl. checksum). For unknown opcodes we
// fall back to reading the maximum to drain whatever the FPGA has buffered.
static int akiko_drain_command(uint8_t *buf)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(AKIKO_BRIDGE_ADDR);

	// First byte = opcode. We have to issue at least one read to know how
	// many follow.
	buf[0] = (uint8_t)spi_w(0);

	int op = buf[0] & 0x0f;
	int payload_len = command_lengths[op];   // bytes following the opcode? No:
	                                         // table is *total* command bytes
	                                         // INCLUDING opcode, EXCLUDING chk.
	int total;
	if (payload_len < 0) {
		// Unknown opcode — drain the full FPGA buffer so we don't leave stale
		// bytes around for the next frame. The bridge clamps rd_ptr at 32.
		total = AKIKO_CMD_MAX;
	} else {
		total = payload_len + 1;             // +1 for trailing checksum byte
	}
	if (total > AKIKO_CMD_MAX) total = AKIKO_CMD_MAX;

	for (int i = 1; i < total; i++) {
		buf[i] = (uint8_t)spi_w(0);
	}

	DisableIO();
	return total;
}

// Append the trailing one-byte checksum (akiko.cpp:838-842) and push the whole
// payload to the FPGA via UIO_DMA_WRITE. The FPGA's RX engine DMAs the bytes
// to chip RAM and asserts the CD32 IRQ when result_done strobes.
static void akiko_send_response(const uint8_t *payload, int len)
{
	if (len < 1 || len > AKIKO_CMD_MAX) {
		akiko_dbg("send_response: bad len %d\n", len);
		return;
	}

	uint8_t out[AKIKO_CMD_MAX + 1];
	memcpy(out, payload, (size_t)len);

	uint32_t sum = 0;
	for (int i = 0; i < len; i++) sum += out[i];
	out[len] = (uint8_t)(0xff - (sum & 0xff));

	int total = len + 1;

	EnableIO();
	spi8(UIO_DMA_WRITE);
	spi32_w(AKIKO_BRIDGE_ADDR);
	for (int i = 0; i < total; i++) {
		spi_w(out[i]);                       // low 8 bits = byte, high = 0
	}
	DisableIO();

#ifdef AKIKO_CD32_DEBUG
	akiko_dbg("TX %d bytes:", total);
	for (int i = 0; i < total; i++) printf(" %02x", out[i]);
	printf("\n");
#endif
}

// -----------------------------------------------------------------------------
// TOC build & push (akiko.cpp:758-786, 956-978, 1437-1440)
// -----------------------------------------------------------------------------

static void toc_pack_entry(int point, uint8_t control, uint32_t msf_or_track)
{
	if (toc_point_count >= AKIKO_TOC_MAX_POINTS) return;
	uint8_t *d = &toc_buffer[toc_point_count * 13];
	memset(d, 0, 13);
	d[1] = 1u | ((uint8_t)control << 4);              // adr=1, control in high nibble
	d[3] = (point < 100) ? bin_to_bcd((uint8_t)point) : (uint8_t)point;
	if (point == 0xA0 || point == 0xA1) {
		// "MSF" actually carries the first/last track number in the M slot.
		d[8] = bin_to_bcd((uint8_t)msf_or_track);
	} else {
		uint32_t lsn = msf_or_track + 150u;           // standard 2-second pre-gap
		uint32_t mins = (lsn / 75u) / 60u;
		uint32_t secs = (lsn / 75u) % 60u;
		uint32_t fr   = lsn % 75u;
		d[8]  = bin_to_bcd((uint8_t)mins);
		d[9]  = bin_to_bcd((uint8_t)secs);
		d[10] = bin_to_bcd((uint8_t)fr);
	}
	toc_point_count++;
}

static void akiko_build_toc(void)
{
	toc_point_count = 0;
	toc_push_idx    = -1;

	drive_t *drv = cd_find_drive();
	if (!drv || drv->track_cnt < 2) {
		akiko_diag("[akiko] TOC build SKIP (no drive or no tracks)");
		return;
	}

	int real_tracks = drv->track_cnt - 1;             // last entry is lead-out
	if (real_tracks < 1 || real_tracks > 99) {
		akiko_diag("[akiko] TOC build SKIP (real_tracks=%d)", real_tracks);
		return;
	}

	// Use track 1's data/audio attribute for the 0xA0/0xA1/0xA2 entries.
	uint8_t first_ctrl = (drv->track[0].attr & 0x40) ? 0x04 : 0x00;
	toc_pack_entry(0xA0, first_ctrl, 1);
	toc_pack_entry(0xA1, first_ctrl, real_tracks);
	toc_pack_entry(0xA2, first_ctrl, drv->track[real_tracks].start);

	for (int i = 0; i < real_tracks; i++) {
		uint8_t ctrl = (drv->track[i].attr & 0x40) ? 0x04 : 0x00;
		toc_pack_entry(drv->track[i].number, ctrl, drv->track[i].start);
	}

	toc_push_idx      = 0;
	toc_push_throttle = 0;
	akiko_diag("[akiko] TOC built: %u points (%d real tracks, lead-out lba=%u)",
	           toc_point_count, real_tracks, drv->track[real_tracks].start);
}

// Push one TOC entry frame. Returns true if a frame was sent (caller should
// then `return` from the poll so we don't double-action this tick).
static bool akiko_push_toc_entry(void)
{
	if (toc_push_idx < 0) return false;
	int point_idx = toc_push_idx / AKIKO_TOC_REPEAT;
	if (point_idx >= toc_point_count) {
		akiko_diag("[akiko] TOC push complete (%d frames sent)", toc_push_idx);
		toc_push_idx = -1;
		return false;
	}
	uint8_t r[15];
	memset(r, 0, sizeof(r));
	r[0] = 0x06;                                       // cmd opcode echo
	r[1] = 0x0a;                                       // "unknown but real CD32 sets it"
	memcpy(r + 2, &toc_buffer[point_idx * 13], 13);
	int counter = toc_push_idx;
	r[6] = bin_to_bcd(99);
	r[7] = bin_to_bcd((uint8_t)((24u + (uint32_t)counter / 75u) % 100u));
	r[8] = bin_to_bcd((uint8_t)((uint32_t)counter % 75u));
	akiko_send_response(r, 15);
	akiko_diag("[akiko] TOC push idx=%d point_idx=%d point=0x%02x ctrl=0x%02x msf=%02x:%02x:%02x",
	           toc_push_idx, point_idx, r[5], (r[3] >> 4) & 0x0f, r[10], r[11], r[12]);
	toc_push_idx++;
	return true;
}

// -----------------------------------------------------------------------------
// Per-opcode command handlers
// -----------------------------------------------------------------------------

// 0x07 — INFO/STATUS. akiko.cpp:940-954.
// Response = 20 bytes: { cmd, door, FIRMWAREVERSION[18] }.
static void cmd_info(const uint8_t *cmd)
{
	uint8_t r[20];
	r[0] = cmd[0];
	r[1] = cd_door;
	memcpy(&r[2], AKIKO_FIRMWARE, 18);       // exactly 18 chars, no NUL
	akiko_send_response(r, 20);
	cd_initialized = 2;
	akiko_dbg("INFO -> initialized=2\n");
	// Build TOC eagerly (cheap), but DON'T start streaming. WinUAE's BIOS
	// asks for TOC via MULTI in PLAY mode with seekpos<0 — see cmd_multi.
	akiko_build_toc();
	toc_push_idx = -1;                            // armed but not pushing yet
	// WinUAE leaves mediachanged=1 across boot, so akiko_handler pushes a
	// second media-status frame *after* cd_initialized hits 2 (lines 1388
	// and 1399 race). Mimic that: arm one more media-status push so the
	// BIOS gets the post-INFO "media is here, you can proceed" ping.
	cd_post_info_media_push_pending = 1;
}

// 0x01 — STOP. akiko.cpp:989-1003.
static void cmd_stop(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];
	r[1] = cd_is_mounted() ? 0x00 : (CH_ERR_NODISK | cd_door);
	cd_playing = 0;
	cd_paused = 0;
	cd_data_lba_base = -1;                       // cancel any data-mode read
	akiko_send_response(r, 2);
	akiko_dbg("STOP\n");
}

// 0x02 — PAUSE.
static void cmd_pause(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];
	r[1] = cd_playing ? CDS_PLAYING : 0;
	cd_paused = 1;
	akiko_send_response(r, 2);
	akiko_dbg("PAUSE (playing=%d)\n", cd_playing);
}

// 0x03 — UNPAUSE.
static void cmd_unpause(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];
	r[1] = cd_playing ? CDS_PLAYING : 0;
	cd_paused = 0;
	akiko_send_response(r, 2);
	akiko_dbg("UNPAUSE (playing=%d)\n", cd_playing);
}

// 0x04 — PLAY/READ. akiko.cpp:1257ff.
//   cmd[1..3] = start MSF (BCD), cmd[4..6] = end MSF (BCD).
//   cmd[7] bit 7: 1 = data read, 0 = audio play.
// We *record* the LBAs but do not actually start audio/data — that's M4.
static void cmd_multi(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];

	if (!cd_is_mounted()) {
		r[1] = 0x01;                         // "no disk" code in the play branch
		akiko_send_response(r, 2);
		akiko_dbg("PLAY: no disk\n");
		return;
	}

	// WinUAE uses signed msf2lsn — values before pre-gap (MSF 00:00:00)
	// produce a negative LSN which the BIOS uses as the "scan TOC" sentinel
	// in the PLAY branch. Match by checking the raw MSF before the lossy
	// uint subtract.
	uint32_t s_msf_total = (((uint32_t)bcd_to_bin(cmd[1]) * 60u
	                       + bcd_to_bin(cmd[2])) * 75u + bcd_to_bin(cmd[3]));
	bool seek_negative = (s_msf_total < 150u);
	uint32_t s_lba = s_msf_total - 150u;          // lossy when seek_negative
	uint32_t e_lba = msf_to_lba(bcd_to_bin(cmd[4]), bcd_to_bin(cmd[5]), bcd_to_bin(cmd[6]));

	cd_play_start_lba = s_lba;
	cd_play_end_lba   = e_lba;

	bool data_read = (cmd[7] & 0x80) != 0;
	if (data_read) {
		// M4: arm the PBX sector fetcher. cdrom_sector_counter on the FPGA
		// is reset to 0 on CDFLAG_ENABLE rising (akiko.cpp:1973-1976), so
		// LBA = base + counter holds across the full read pass.
		cd_data_lba_base = (int32_t)s_lba;
		r[1] = 0x02;
	} else if (seek_negative) {
		// PLAY with seekpos < 0 = "scan TOC" trigger (akiko.cpp:1095-1097).
		// Start streaming TOC entries to the BIOS one frame at a time.
		cd_data_lba_base = -1;
		cd_playing = 0;
		cd_paused = 0;
		toc_push_idx = (toc_point_count > 0) ? 0 : -1;
		toc_push_throttle = 0;
		r[1] = 0x42;                              // play-starting status
		akiko_diag("[akiko] MULTI scan-TOC trigger (points=%u)", toc_point_count);
	} else {
		// TODO(post-M4): start CDDA from cd_play_start_lba.
		cd_data_lba_base = -1;
		cd_playing = 1;
		cd_paused = 0;
		r[1] = 0x42;
	}

	akiko_send_response(r, 2);
	akiko_dbg("PLAY %s start=%u(%s) end=%u\n",
		data_read ? "DATA" : "AUDIO", s_lba, seek_negative ? "neg" : "pos", e_lba);
}

// 0x05 — LED control. cmd[1] bit 7 set means "respond with new state".
//   Form A (bit7 set): 2-byte response { cmd, led_state }.
//   Form B (bit7 clr): 1-byte response { cmd } (just an ack).
static void cmd_led(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];

	if (cmd[1] & 0x80) {
		cd_led_state = cmd[1] & 0x01;
		r[1] = cd_led_state;
		akiko_send_response(r, 2);
	} else {
		akiko_send_response(r, 1);
	}
	akiko_dbg("LED cmd1=%02x state=%d\n", cmd[1], cd_led_state);
}

// 0x06 — SUBQ. akiko.cpp returns 15 bytes of Q-channel data. For MVP we send
// all-zero. Real subcode synthesis is post-M4.
static void cmd_subq(const uint8_t *cmd)
{
	uint8_t r[15];
	memset(r, 0, sizeof(r));
	r[0] = cmd[0];
	akiko_send_response(r, 15);
	akiko_dbg("SUBQ (stub)\n");
}

// Catch-all for unknown / reserved opcodes. akiko.cpp:1161-1191 style.
static void cmd_bad(const uint8_t *cmd, uint8_t err_code)
{
	uint8_t r[2];
	r[0] = (uint8_t)((cmd[0] & 0xf0) | 5);   // top nibble preserved, low = 5
	r[1] = err_code | cd_door;
	akiko_send_response(r, 2);
	akiko_dbg("BAD cmd=%02x err=%02x\n", cmd[0], err_code);
}

// -----------------------------------------------------------------------------
// M4 PBX sector channel
// -----------------------------------------------------------------------------

// Read the FPGA's current cdrom_sector_counter (1-byte read on the sec
// sub-channel, 0xF500). The bridge presents hps_sec_status on every read
// strobe; one word is enough.
static uint8_t akiko_read_sec_counter(void)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(AKIKO_SECTOR_ADDR);
	uint16_t w = spi_w(0);
	DisableIO();
	return (uint8_t)(w & 0xff);
}

// Push 2352 bytes via UIO_DMA_WRITE on the sec sub-channel. The bridge
// pulses hps_sec_done on deselect, which latches sector_ready in the engine
// and unblocks the PBX state machine.
static void akiko_push_sector(const uint8_t *buf)
{
	EnableIO();
	spi8(UIO_DMA_WRITE);
	spi32_w(AKIKO_SECTOR_ADDR);
	for (int i = 0; i < AKIKO_SECTOR_BYTES; i++) {
		spi_w(buf[i]);
	}
	DisableIO();
}

// Service one akiko_sec_req. Reads the engine's current sector_counter,
// fetches LBA = base + counter from the CD image, and pushes the raw 2352-byte
// frame back. On any error (no disc, no data read armed, image read failure)
// we still push 2352 zeros so the engine doesn't stall — the PBX cycle will
// land but the resulting sector will fail Kickstart's data-checksum (returning
// junk is preferable to deadlocking the CD32 boot path on a transient).
static void akiko_handle_sec_req(void)
{
	uint8_t counter = akiko_read_sec_counter();

	uint8_t buf[AKIKO_SECTOR_BYTES];

	if (cd_data_lba_base < 0) {
		akiko_dbg("sec_req but no data read armed (counter=%u)\n", counter);
		memset(buf, 0, sizeof(buf));
		akiko_push_sector(buf);
		return;
	}

	drive_t *drv = cd_find_drive();
	if (!drv) {
		akiko_dbg("sec_req but no CD drive (counter=%u)\n", counter);
		memset(buf, 0, sizeof(buf));
		akiko_push_sector(buf);
		return;
	}

	uint32_t lba = (uint32_t)cd_data_lba_base + counter;
	if (cdrom_read_raw_sector(drv, lba, buf) != 0) {
		akiko_dbg("sec_req lba=%u read FAILED\n", lba);
		memset(buf, 0, sizeof(buf));
	} else {
		akiko_dbg("sec_req lba=%u counter=%u OK\n", lba, counter);
	}

	akiko_push_sector(buf);
}

// -----------------------------------------------------------------------------
// Bus trace drain (debug)
// -----------------------------------------------------------------------------

// Drain the akiko_bus_trace ring buffer (up to 32 entries). For each captured
// CPU access we emit one line: `[trace] R $B80012 = 0x4711` style. The ring
// is small, so we drain on every poll to avoid losing entries during a burst
// of firmware reads/writes.
static void akiko_drain_trace(void)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(AKIKO_TRACE_ADDR);

	// Cap at 32 entries (ring depth) so a runaway loop can't lock us up.
	for (int i = 0; i < 32; i++) {
		uint8_t b0 = (uint8_t)spi_w(0);  // {wr, addr[6:0]}
		uint8_t b1 = (uint8_t)spi_w(0);  // data[7:0]
		uint8_t b2 = (uint8_t)spi_w(0);  // data[15:8]
		uint8_t b3 = (uint8_t)spi_w(0);  // 0xFF valid, 0x00 empty

		if (b3 == 0) break;              // ring drained

		bool     is_wr = (b0 & 0x80) != 0;
		uint32_t addr  = 0xB80000u | ((uint32_t)(b0 & 0x7F) << 1);
		uint16_t data  = (uint16_t)b1 | ((uint16_t)b2 << 8);

		akiko_diag("[trace] %s $%06X = 0x%04X", is_wr ? "W" : "R", addr, data);
	}

	DisableIO();
}

// -----------------------------------------------------------------------------
// Top-level poll
// -----------------------------------------------------------------------------

void akiko_cd32_init(void)
{
	cd_initialized   = 0;
	cd_paused        = 0;
	cd_playing       = 0;
	cd_led_state     = 0;
	cd_door          = 1;                          // see static initializer above
	cd_play_start_lba = 0;
	cd_play_end_lba   = 0;
	cd_data_lba_base = -1;
	cd_last_mounted  = false;
	toc_point_count  = 0;
	toc_push_idx     = -1;
	toc_push_throttle = 0;
	akiko_dbg("init\n");
}

// Write a one-line diagnostic to /tmp/akiko_dbg.log, bypassing libc stdio
// buffering. Cheap enough for an event-driven loop. Always compiled in.
static void akiko_diag(const char *fmt, ...)
{
	FILE *f = fopen("/tmp/akiko_dbg.log", "a");
	if (!f) return;
	va_list ap; va_start(ap, fmt);
	vfprintf(f, fmt, ap);
	va_end(ap);
	fputc('\n', f);
	fflush(f);
	fclose(f);
}

void akiko_cd32_poll(void)
{
	bool mounted = cd_is_mounted();

	// Heartbeat: prove poll loop reached us at all. Writes to a dedicated
	// log file so it survives any stdout redirection MiSTer does after init.
	static bool first_poll = true;
	if (first_poll) {
		first_poll = false;
		akiko_diag("[akiko] poll alive (first call) mounted=%d", mounted);
	}

	// Drain bus trace ring first so we always log what the CPU did before
	// we react to it. Cheap when the ring is empty (4 spi reads + early-out).
	akiko_drain_trace();

	// While we don't think we're mounted, periodically dump the underlying
	// ide_inst flags so we can see what state the IDE subsystem is in.
	// This burns one log line per second until something flips.
	static int unmounted_dump_throttle = 0;
	if (!mounted && (unmounted_dump_throttle++ % 60) == 0 && unmounted_dump_throttle < 5*60) {
		akiko_diag(
			"[akiko] ide_inst dump: "
			"[0,0]p=%d/c=%d/chd=%p/f=%p  [0,1]p=%d/c=%d/chd=%p/f=%p",
			ide_inst[0].drive[0].present, ide_inst[0].drive[0].cd,
			(void*)ide_inst[0].drive[0].chd_f, (void*)ide_inst[0].drive[0].f,
			ide_inst[0].drive[1].present, ide_inst[0].drive[1].cd,
			(void*)ide_inst[0].drive[1].chd_f, (void*)ide_inst[0].drive[1].f
		);
	}

	// Re-arm auto-init if media was swapped or just inserted.
	if (mounted != cd_last_mounted) {
		cd_initialized   = 0;
		cd_data_lba_base = -1;               // any in-progress read is stale
		toc_point_count  = 0;                // force TOC rebuild after next INFO
		toc_push_idx     = -1;
		cd_last_mounted  = mounted;
		akiko_diag("[akiko] media change -> mounted=%d", mounted);
	}

	// 1. Auto-init: push media-status frame periodically until host echoes
	//    it back via INFO (cd_initialized -> 2). On real CD32 the boot menu
	//    sometimes opens cd.device long after first poll, so a one-shot push
	//    races with that. Re-pushing every ~1s costs nothing and recovers.
	//    akiko.cpp:1388-1397.
	static int media_push_throttle = 0;
	if (mounted && cd_initialized < 2) {
		bool first = (cd_initialized == 0);
		bool periodic = (cd_initialized == 1) && ((media_push_throttle++ % 60) == 0);
		if (first || periodic) {
			uint8_t r[2] = { 0x0a, 0x01 };   // matches WinUAE cdrom_command_media_status (akiko.cpp:932-936)
			akiko_send_response(r, 2);
			if (first) cd_initialized = 1;
			akiko_diag("[akiko] media-status push (init=%d, %s)",
			           cd_initialized, first ? "first" : "periodic");
			return;                          // one bridge action per poll
		}
	}

	// 1.4 Post-INFO media-status push. Initial one-shot (immediately after
	//     INFO completes) plus continued periodic pushes every ~2s. The
	//     CD32 BIOS appears to wait for an ongoing drive-ready ping after
	//     identification before issuing its first MULTI scan -- a single
	//     post-INFO push isn't enough on hardware.
	static int post_info_throttle = 0;
	if (cd_initialized == 2) {
		bool first_post = (cd_post_info_media_push_pending != 0);
		bool periodic = (post_info_throttle++ % 120) == 0; // ~2s @ 60Hz poll
		if (first_post || periodic) {
			cd_post_info_media_push_pending = 0;
			uint8_t r[2] = { 0x0a, 0x01 };
			akiko_send_response(r, 2);
			akiko_diag("[akiko] post-INFO media-status push (%s)",
			           first_post ? "first" : "periodic");
			return;
		}
	}

	// 1.5 Stream TOC entries to the BIOS (cmd 0x06 / cdrom_return_toc_entry).
	//     The CD32 BIOS scans for TOC via these proactive pushes. Without
	//     them it never sends MULTI/READ — it sits forever on the AMIGA CD32
	//     spinning-CD splash polling CDINTREQ. Throttled so we don't flood
	//     the bridge while still much faster than WinUAE's 60Hz framesync.
	if (cd_initialized == 2 && toc_push_idx >= 0) {
		if ((toc_push_throttle++ % AKIKO_TOC_PUSH_PERIOD) == 0) {
			if (akiko_push_toc_entry()) {
				return;                      // one bridge action per poll
			}
		}
	}

	// 2. Status poll. sec_req is checked first because PBX has tighter timing
	//    requirements than the command stream — Kickstart's data-read loop
	//    expects sectors to land within a few frames of the slot bit being
	//    written. Both bits can be set concurrently; we'll get the cmd next
	//    frame.
	uint16_t status = akiko_read_status();
	static uint16_t last_status = 0xffff;
	static int status_log_count = 0;
	if (status != last_status && status_log_count < 200) {
		akiko_diag("[akiko] status=0x%04x (was 0x%04x)", status, last_status);
		last_status = status;
		status_log_count++;
	}

	if (status & AKIKO_STATUS_SEC_REQ) {
		akiko_handle_sec_req();
		return;                              // one bridge action per poll
	}

	if (!(status & AKIKO_STATUS_REQ)) return;

	// 3. Drain the framed command.
	uint8_t cmd[AKIKO_CMD_MAX];
	int     n = akiko_drain_command(cmd);
	if (n < 1) {
		akiko_dbg("drain returned 0 bytes\n");
		return;
	}

#ifdef AKIKO_CD32_DEBUG
	akiko_dbg("RX %d bytes:", n);
	for (int i = 0; i < n; i++) printf(" %02x", cmd[i]);
	printf("\n");
#endif

	int op = cmd[0] & 0x0f;
	int expected_len = command_lengths[op];

	// 4. Validate input checksum (akiko.cpp:1161-1191): sum of all bytes incl.
	//    checksum byte must equal 0xff. Only meaningful for known opcodes.
	if (expected_len > 0) {
		int chk_total = expected_len + 1;    // bytes summed = cmd + chk
		if (n < chk_total) {
			akiko_dbg("short frame: n=%d expected=%d\n", n, chk_total);
			cmd_bad(cmd, CH_ERR_CHECKSUM);
			return;
		}
		uint32_t sum = 0;
		for (int i = 0; i < chk_total; i++) sum += cmd[i];
		if ((sum & 0xff) != 0xff) {
			akiko_dbg("checksum FAIL sum=%02x\n", sum & 0xff);
			cmd_bad(cmd, CH_ERR_CHECKSUM);
			return;
		}
	}

	// 5. Dispatch.
	switch (op) {
		case 0x01: cmd_stop(cmd);    break;
		case 0x02: cmd_pause(cmd);   break;
		case 0x03: cmd_unpause(cmd); break;
		case 0x04: cmd_multi(cmd);   break;
		case 0x05: cmd_led(cmd);     break;
		case 0x06: cmd_subq(cmd);    break;
		case 0x07: cmd_info(cmd);    break;
		default:
			cmd_bad(cmd, CH_ERR_BADCOMMAND);
			break;
	}
}
