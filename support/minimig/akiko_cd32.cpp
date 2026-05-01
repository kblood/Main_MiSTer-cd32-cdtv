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
#ifdef AKIKO_CD32_DEBUG
	#define akiko_dbg(...) do { printf("[akiko] "); printf(__VA_ARGS__); } while (0)
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
static uint8_t  cd_door             = 0;       // door always "closed" for now
static uint32_t cd_play_start_lba   = 0;       // recorded for M4 (real audio/data)
static uint32_t cd_play_end_lba     = 0;

// M4 PBX state: cd_data_lba_base is set when cmd 0x04 is issued in DATA mode
// (cmd[7] bit 7 = 1). On each FPGA sec_req, we read the FPGA's sector_counter
// and fetch LBA = base + counter. -1 = no data read in progress (push zeros).
static int32_t  cd_data_lba_base    = -1;

// Last mounted state — used to re-arm the auto-init when a disc is swapped.
static bool     cd_last_mounted     = false;

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

	uint32_t s_lba = msf_to_lba(bcd_to_bin(cmd[1]), bcd_to_bin(cmd[2]), bcd_to_bin(cmd[3]));
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
	} else {
		// TODO(post-M4): start CDDA from cd_play_start_lba.
		cd_data_lba_base = -1;
		cd_playing = 1;
		cd_paused = 0;
		r[1] = 0x42;
	}

	akiko_send_response(r, 2);
	akiko_dbg("PLAY %s start=%u end=%u\n",
		data_read ? "DATA" : "AUDIO", s_lba, e_lba);
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
// Top-level poll
// -----------------------------------------------------------------------------

void akiko_cd32_init(void)
{
	cd_initialized   = 0;
	cd_paused        = 0;
	cd_playing       = 0;
	cd_led_state     = 0;
	cd_door          = 0;
	cd_play_start_lba = 0;
	cd_play_end_lba   = 0;
	cd_data_lba_base = -1;
	cd_last_mounted  = false;
	akiko_dbg("init\n");
}

void akiko_cd32_poll(void)
{
	bool mounted = cd_is_mounted();

	// Re-arm auto-init if media was swapped or just inserted.
	if (mounted != cd_last_mounted) {
		cd_initialized   = 0;
		cd_data_lba_base = -1;               // any in-progress read is stale
		cd_last_mounted  = mounted;
		akiko_dbg("media change -> mounted=%d\n", mounted);
	}

	// 1. Auto-init: first poll after media is mounted, push media-status frame
	//    *before* the host sends anything. akiko.cpp:1388-1397.
	if (mounted && cd_initialized == 0) {
		uint8_t r[2] = { 0x0a, 0x01 };       // 0x01 = media present
		akiko_send_response(r, 2);
		cd_initialized = 1;
		akiko_dbg("auto media-status -> initialized=1\n");
		return;                              // one bridge action per poll
	}

	// 2. Status poll. sec_req is checked first because PBX has tighter timing
	//    requirements than the command stream — Kickstart's data-read loop
	//    expects sectors to land within a few frames of the slot bit being
	//    written. Both bits can be set concurrently; we'll get the cmd next
	//    frame.
	uint16_t status = akiko_read_status();

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
