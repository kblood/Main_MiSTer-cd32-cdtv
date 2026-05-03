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
#include <sys/stat.h>     // mkdir() for /media/fat/saves/Minimig
#include <errno.h>
#include <unistd.h>       // unlink, fsync

#include "../../spi.h"
#include "../../user_io.h"
#include "../../ide.h"
#include "../../ide_cdrom.h"
#include "../../hardware.h"   // GetTimer / CheckTimer
#include "akiko_cd32.h"

// -----------------------------------------------------------------------------
// Debug gating
// -----------------------------------------------------------------------------
// Forward declaration so the debug macro can use it before its definition.
static void akiko_diag(const char *fmt, ...);

// Verbose per-command/per-status diag (TX/RX byte dumps, status change
// chatter). Costly: every line is a synchronous fopen/fwrite/fclose to
// tmpfs, and once gameplay starts CF generates 200+ of these per second.
// Off in production. (The poll-loop throttle in akiko_cd32_poll is what
// was historically masked by this logging — see comment there.)
#define AKIKO_CD32_DEBUG 0
#if AKIKO_CD32_DEBUG
	// Route through akiko_diag so output reaches /tmp/akiko_dbg.log instead
	// of stdout, which Minimig redirects/silences after init.
	#define akiko_dbg(fmt, ...) akiko_diag("[akiko] " fmt, ##__VA_ARGS__)
#else
	#define akiko_dbg(...) do { } while (0)
#endif

// Bus-trace ring drain. Independent flag because the per-entry logging
// dwarfs everything else (>90% of log volume during gameplay). HOWEVER —
// the drain itself MUST run every poll regardless: the trace ring in
// akiko_hps_bridge appears to back-pressure the CPU when full, so an
// undrained ring stalls CF on dense bus activity (e.g. C2P traffic
// during early boot — observed: CF stuck at lba 33 vs lba 25k+ with
// drain enabled). When this flag is 0 we still drain, just silently.
#define AKIKO_BUS_TRACE 0

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
// Phase 18: bit[9] = akiko_rx_busy = (cdrom_receive_length != 0). Mirror of
// WinUAE's cdrom_can_return_data() gate: when set, the FPGA RX engine still
// has a queued/in-flight response and we must NOT push another frame, or it
// gets dropped (overwritten in result_buffer before the framer drains it).
#define AKIKO_STATUS_RX_BUSY (1u << 9)

// Sub-channel selector inside the 0xF400 class: io_din[8] = 1 selects the
// sector channel (hps_ext.v:135, akiko_cs_sec). 0xF400 | 0x100 = 0xF500.
#define AKIKO_SECTOR_ADDR  0xF500

// Phase 32: NVRAM save-dump sub-channel. io_din[6] = 1 selects the NVRAM
// host port (akiko_hps_bridge.v + akiko_nvram.v). 0xF400 | 0x40 = 0xF440.
// Reading streams 1024 bytes (auto-incrementing internal addr counter,
// resets on cs rise). Any write here pulses host_clear_dirty.
#define AKIKO_NVRAM_ADDR              0xF440
#define AKIKO_NVRAM_BYTES             1024
#define AKIKO_NVRAM_DIR               "/media/fat/saves/Minimig"
// Pre-Phase-32.5.1 single-file fallback. Used only when the dirty-debounce
// poll path tries to save and we have no per-game CD path active (e.g. the
// player wrote to EEPROM before any CD got mounted, or the path-setter hook
// hasn't fired yet). After 32.5.1 normal use writes to per-game files
// `cd32-<hash>.nvr` in AKIKO_NVRAM_DIR; legacy `cd32.nvr` is read on first
// load only as a one-shot migration when no per-game file exists.
#define AKIKO_NVRAM_FILE_LEGACY       "/media/fat/saves/Minimig/cd32.nvr"
#define AKIKO_STATUS_NVR_DIRTY        (1u << 7)   // hps_ext bit 7
// Throttle: poll the dirty bit at most once per second; once seen, wait
// this long for the dirty state to stabilize before saving (debounces
// bursty BIOS FlashFile commits, which may take multiple I2C writes).
#define AKIKO_NVRAM_POLL_PERIOD_MS    1000
#define AKIKO_NVRAM_DIRTY_DEBOUNCE_MS 5000

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

// Status / error nibbles. Values are byte-exact mirrors of WinUAE
// akiko.cpp:433-449 — getting these wrong is what kept the no-CD splash
// from rendering: BIOS distinguishes "no disc, drive ready" (0xF8|door =
// 0xF9) from "bad command" (0x80) by exact bit pattern. Returning 0x81
// for no-disc (the wrong NODISK value 0x80 | door 0x01) made BIOS think
// the drive was reporting an unknown command and re-poll forever.
#define CH_ERR_OK          0x00
#define CH_ERR_BADCOMMAND  0x80
#define CH_ERR_NODISK      0xf8
#define CH_ERR_CHECKSUM    0x88
#define CDS_ERROR          0x80
#define CDS_PLAYING        0x08
#define CDS_PLAYEND        0x00

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

// Audio-play notification state (mirror of WinUAE cdrom_audiotimeout,
// akiko.cpp:1411-1435). cmd_multi acks PLAY AUDIO synchronously with 0x42
// ("play starting"), but BIOS won't advance past the audio-cued state until
// it sees the asynchronous opcode-0x04 follow-up frame. Values:
//    2,1 -> countdown to "play started" emission (CDS_PLAYING|2)
//   -1   -> stop audio engine (placeholder until Phase 33), advance to -2
//   -2   -> emit "play ended" (CDS_PLAYEND)
//   -3   -> emit "play failed" (CDS_ERROR)
static int8_t   cd_audio_timeout    = 0;

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
#define AKIKO_TOC_PUSH_PERIOD  1000 // ~70Hz at our ~70kHz poll = matches WinUAE framesync (60Hz)
#define AKIKO_TOC_MAX_POINTS   16
static uint8_t toc_buffer[AKIKO_TOC_MAX_POINTS * 13];
static uint8_t toc_point_count      = 0;
static int16_t toc_push_idx         = -1;   // -1 = idle; else next slot in 3x sequence
static int     toc_push_throttle    = 0;

// Phase 32.5.1: per-game NVRAM. cd_save_path_active is the full path of the
// per-CD save file currently mirrored in FPGA BRAM (`""` when no CD is
// mounted). cd_save_load_pending is set by akiko_cd32_set_cd_path() (CD
// swap) or akiko_cd32_init() (Minimig core reconfig wiped BRAM) and cleared
// by the poll loop after the load actually fires. cd_save_dirty_observed
// tracks whether we've ever seen the FPGA dirty flag asserted since the
// current save was loaded — used to skip the pre-swap flush when the player
// never touched EEPROM (avoids overwriting a real save with the synthesized
// FlashFile init from a different CD).
static char cd_save_path_active[256] = {0};
static bool cd_save_load_pending     = false;
static bool cd_save_dirty_observed   = false;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

// FNV-1a 64-bit. Stable across runs/builds, no endian concerns. Used to map
// a CD basename to a 16-hex-char save filename slot.
static uint64_t fnv1a_64(const char *s)
{
	uint64_t h = 0xcbf29ce484222325ULL;
	while (*s) {
		h ^= (uint64_t)(uint8_t)*s++;
		h *= 0x100000001b3ULL;
	}
	return h;
}

// Build `<AKIKO_NVRAM_DIR>/cd32-<hash>.nvr` from the basename of `cd_path`.
// We hash the basename (not the full path) so the same CHD in a different
// folder maps to the same save slot. Comparison is case-sensitive — Linux
// FS, and CD32 CHDs are typically named in a stable case anyway.
static void compute_save_path(const char *cd_path, char *out, size_t outsz)
{
	const char *base = strrchr(cd_path, '/');
	base = base ? base + 1 : cd_path;
	uint64_t h = fnv1a_64(base);
	snprintf(out, outsz, "%s/cd32-%016" PRIx64 ".nvr", AKIKO_NVRAM_DIR, h);
}

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

// "Is real CD media inserted?" — matches WinUAE's `cdrom_disk != NULL` semantic.
// A placeholder drive (cfg=2, no filename) has present=0/cd=1 with chd_f=NULL,
// f=NULL — that's "drive ready, no media", which WinUAE treats as no-disk for
// cmd_multi/pause/unpause. Checking present+cd alone would let MULTI fall into
// the scan-TOC branch and then PLAY AUDIO with a junk seekpos, trapping BIOS
// in the UNPAUSE polling loop and never letting it render the splash.
static bool cd_is_mounted(void)
{
	for (int p = 0; p < 2; p++) {
		for (int d = 0; d < 2; d++) {
			drive_t *drv = &ide_inst[p].drive[d];
			if (drv->cd && (drv->chd_f || drv->f)) {
				return true;
			}
		}
	}
	return false;
}

// Find the first CD drive that has real media. NULL if no drive has media.
static drive_t *cd_find_drive(void)
{
	for (int p = 0; p < 2; p++) {
		for (int d = 0; d < 2; d++) {
			drive_t *drv = &ide_inst[p].drive[d];
			if (drv->cd && (drv->chd_f || drv->f)) {
				return drv;
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


#if AKIKO_CD32_DEBUG
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

	// Phase 24 diag: dump every track's raw fields so we can see what the
	// CHD parser actually populated.
	for (int i = 0; i <= real_tracks; i++) {
		akiko_diag("[akiko] TRACK[%d] num=%u attr=0x%02x start=%u length=%u chd_off=%u",
		           i, drv->track[i].number, drv->track[i].attr,
		           drv->track[i].start, drv->track[i].length,
		           drv->track[i].chd_offset);
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

	// Phase 19: do NOT auto-arm the drip. WinUAE's akiko_handler only emits
	// TOC frames when cdrom_toc_counter >= 0, and that counter is set to 0
	// only inside cdrom_command_multi() at line 1096 — i.e. after BIOS issues
	// MULTI cmd 0x04 with the negative-MSF "scan TOC" sentinel. Auto-arming
	// here (and in cmd_info) caused the drip to run forever, BIOS to consume
	// 700k entries, and never frame LED=1/PLAY because it was stuck in TOC-
	// scan mode. We keep the build (it's cheap) but leave toc_push_idx = -1.
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
	// Phase 19: hard-stop after points*REPEAT pushes, matching WinUAE
	// akiko.cpp:974-976. Looping forever kept BIOS in TOC-scan mode and
	// blocked progression to LED=1/PLAY — BIOS treats counter=-1 as "TOC
	// transmission complete" and only then advances state.
	if (point_idx >= toc_point_count) {
		toc_push_idx = -1;
		akiko_diag("[akiko] TOC drip complete (%u points * %d repeat = %d frames)",
		           toc_point_count, AKIKO_TOC_REPEAT,
		           toc_point_count * AKIKO_TOC_REPEAT);
		return false;
	}
	uint8_t r[15];
	memset(r, 0, sizeof(r));
	r[0] = 0x06;                                       // cmd opcode echo
	r[1] = 0x0a;                                       // "unknown but real CD32 sets it"
	memcpy(r + 2, &toc_buffer[point_idx * 13], 13);
	int counter = toc_push_idx;
	// Phase 19: match WinUAE akiko.cpp:971-973 byte-for-byte. WinUAE does
	// NOT take r[7] mod 100 — it lets BCD wrap naturally past 99. Removed
	// the %100 to avoid drifting after counter/75 >= 76.
	r[6] = bin_to_bcd(99);
	r[7] = bin_to_bcd((uint8_t)(24u + (uint32_t)counter / 75u));
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
	// Phase 21: removed auto-arm of TOC drip and post-INFO media push.
	// Subagent diff vs WinUAE akiko.cpp:940-954 (cdrom_command_status / INFO)
	// shows WinUAE does NOTHING after building the TOC — it just bumps
	// cd_initialized to 2 and waits for BIOS's next command. The TOC drip
	// counter is set to 0 in EXACTLY ONE place (akiko.cpp:1096), inside
	// cdrom_command_multi when MULTI 0x04 is issued with seekpos<0 (the
	// "scan TOC" sentinel). Our cmd_multi at line 502 already does this.
	// Phase 19.5's auto-arm sent BIOS 15 unsolicited cmd 0x06 frames it
	// never asked for, which corrupted its rxinx/rxcmp FSM and caused it
	// to go silent after the drip completed. Removing it should let BIOS
	// proceed straight from INFO to MULTI 0x04 (data read of boot sector).
}

// Async play-state notification — WinUAE cdrom_playend_notify (akiko.cpp:
// 1110-1121). Emitted by the poll loop after a MULTI audio command, never
// in synchronous response to a command frame.
//   status  0 = started   -> CDS_PLAYING | 0x02 | door
//   status  1 = ended     -> CDS_PLAYEND | door  (==door since CDS_PLAYEND=0)
//   status -1 = failed    -> CDS_ERROR   | door
static void emit_playend_notify(int status)
{
	uint8_t r[2];
	r[0] = 0x04;
	if (status < 0)        r[1] = CDS_ERROR;
	else if (status == 0)  r[1] = CDS_PLAYING | 0x02;
	else                   r[1] = CDS_PLAYEND;
	r[1] |= cd_door;
	akiko_send_response(r, 2);
	akiko_dbg("PLAYEND_NOTIFY status=%d -> %02x\n", status, r[1]);
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
	// If a play-started ack is still pending from a prior MULTI audio that
	// just got STOPped, swallow it. Conversely, a play-ended timeout (-2/-1)
	// is a legitimate "engine just finished" signal and stays armed so BIOS
	// sees the natural end-of-play frame.
	if (cd_audio_timeout > 0) cd_audio_timeout = 0;
	akiko_send_response(r, 2);
	akiko_dbg("STOP\n");
}

// 0x02 — PAUSE. WinUAE akiko.cpp:1005-1024. checkerr() at line 1014 returns
// CH_ERR_NODISK | cdrom_door if no disc. We were returning 0 in that case,
// telling BIOS "drive idle, fine" instead of "no disc, waiting for media".
static void cmd_pause(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];
	if (!cd_is_mounted()) {
		r[1] = CH_ERR_NODISK | cd_door;
	} else {
		r[1] = (cd_playing ? CDS_PLAYING : 0) | cd_door;
	}
	cd_paused = 1;
	akiko_send_response(r, 2);
	akiko_dbg("PAUSE (playing=%d, mounted=%d)\n", cd_playing, cd_is_mounted());
}

// 0x03 — UNPAUSE. WinUAE akiko.cpp:1027-1044. Same checkerr() pattern.
static void cmd_unpause(const uint8_t *cmd)
{
	uint8_t r[2];
	r[0] = cmd[0];
	if (!cd_is_mounted()) {
		r[1] = CH_ERR_NODISK | cd_door;
	} else {
		r[1] = (cd_playing ? CDS_PLAYING : 0) | cd_door;
	}
	cd_paused = 0;
	akiko_send_response(r, 2);
	akiko_dbg("UNPAUSE (playing=%d, mounted=%d)\n", cd_playing, cd_is_mounted());
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
		r[1] = 0x01;                         // "no disk" code in the play branch (matches WinUAE akiko.cpp:1059)
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
		// Phase 23: WinUAE sets r[1] = 0 for scan-TOC (default from line 1057),
		// not 0x42. The 0x42 "play started" status is reserved for actual
		// audio play (seekpos >= 0, line 1099). Our 0x42 here was telling
		// BIOS "play has started" which conflicts with the TOC scan that
		// follows — likely keeping BIOS in audio-play state machine instead
		// of letting it transition to data-read after the TOC.
		cd_data_lba_base = -1;
		cd_playing = 0;
		cd_paused = 0;
		toc_push_idx = (toc_point_count > 0) ? 0 : -1;
		toc_push_throttle = 0;
		r[1] = 0x00;                              // scan-TOC: no play-started flag
		akiko_diag("[akiko] MULTI scan-TOC trigger (points=%u)", toc_point_count);
	} else {
		// PLAY AUDIO. Synchronous ack first (0x42 = "play starting"), then
		// the poll loop emits the asynchronous "play started" follow-up via
		// the cd_audio_timeout state machine. BIOS gates audio-cued game
		// progress on the async frame — without it, titles that start a
		// CDDA cue (Banshee, Speris Legacy, JP3) hang. Real audio output
		// arrives in Phase 33 (CDDA streaming).
		// TODO(Phase 33): replace with CDDA pump arming using
		// cd_play_start_lba/cd_play_end_lba (already captured above).
		cd_data_lba_base = -1;
		cd_playing = 1;
		cd_paused = 0;
		r[1] = 0x42;
		cd_audio_timeout = 2;
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
		// Phase 21: WinUAE akiko.cpp:917-922 returns 0 here, dispatcher at
		// line 1288 calls set_status(DRIVEXMIT) which is "not used by ROM,
		// PIO mode" (akiko.cpp:416). Net effect = no buffer push, no IRQ.
		// We were pushing 1 byte + checksum = 2 bytes that BIOS read as
		// part of the next response, putting our RX buffer 2 bytes ahead
		// of BIOS's RXCMP tracking. Drop the response entirely.
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

// -----------------------------------------------------------------------------
// Phase 32: NVRAM save-dump (UIO 0xF440)
// -----------------------------------------------------------------------------
// Drain all 1024 NVRAM bytes from the FPGA via UIO_DMA_READ on the nvr
// sub-channel. The bridge presents bytes from an internal address counter
// that resets to 0 on cs rising and increments per uio_rd. host_dout has
// 1-cycle BRAM latency; cs is held many cycles before the first read so
// the first byte is valid by then.
static void akiko_nvram_dump(uint8_t *out_1024)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(AKIKO_NVRAM_ADDR);
	for (int i = 0; i < AKIKO_NVRAM_BYTES; i++) {
		out_1024[i] = (uint8_t)spi_w(0);
	}
	DisableIO();
}

// Phase 32.5: load 1024 bytes from a save file into FPGA NVRAM BRAM via
// the host write port (UIO_DMA_WRITE on the nvr sub-channel). Returns true
// if the file existed at the right size and was streamed; false (silently)
// otherwise — in which case BRAM keeps its synthesized FlashFile-magic
// init, behaving as a fresh empty EEPROM (the pre-Phase-32.5 cold-boot
// behavior). The bridge does NOT touch the dirty flag on writes, so
// loading does not provoke an immediate re-save on the next poll.
static bool akiko_nvram_load_from_path(const char *path)
{
	struct stat st;
	if (stat(path, &st) != 0) {
		akiko_diag("[akiko] NVR load skip: no file at %s (errno=%d)",
		           path, errno);
		return false;
	}
	if (st.st_size != AKIKO_NVRAM_BYTES) {
		akiko_diag("[akiko] NVR load skip: %s wrong size %lld (want %d)",
		           path, (long long)st.st_size, AKIKO_NVRAM_BYTES);
		return false;
	}

	uint8_t buf[AKIKO_NVRAM_BYTES];
	FILE *f = fopen(path, "rb");
	if (!f) {
		akiko_diag("[akiko] NVR fopen(%s, rb) failed: errno=%d",
		           path, errno);
		return false;
	}
	size_t got = fread(buf, 1, AKIKO_NVRAM_BYTES, f);
	fclose(f);
	if (got != AKIKO_NVRAM_BYTES) {
		akiko_diag("[akiko] NVR fread short: got %zu want %d",
		           got, AKIKO_NVRAM_BYTES);
		return false;
	}

	EnableIO();
	spi8(UIO_DMA_WRITE);
	spi32_w(AKIKO_NVRAM_ADDR);
	for (int i = 0; i < AKIKO_NVRAM_BYTES; i++) {
		spi_w(buf[i]);
	}
	DisableIO();

	akiko_diag("[akiko] NVR loaded %d bytes from %s", AKIKO_NVRAM_BYTES, path);
	return true;
}

// Atomic save to a specific path: dump → write to .tmp → fsync → rename →
// unlink stale .tmp. Returns true on success. Failure paths log via
// akiko_diag and leave the dirty flag set (next poll re-tries). Note that
// akiko_nvram_dump on the bridge auto-clears the dirty flag at end of the
// read burst, so a successful save is naturally idempotent.
static bool akiko_nvram_save_to_path(const char *path)
{
	uint8_t buf[AKIKO_NVRAM_BYTES];
	akiko_nvram_dump(buf);

	// Ensure target directory exists. mkdir is fine if it already does
	// (EEXIST). Other errors surface during fopen.
	if (mkdir(AKIKO_NVRAM_DIR, 0755) != 0 && errno != EEXIST) {
		akiko_diag("[akiko] NVR mkdir(%s) failed: errno=%d", AKIKO_NVRAM_DIR, errno);
		// fall through — fopen below will report the real failure
	}

	char tmp_path[300];
	snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

	FILE *f = fopen(tmp_path, "wb");
	if (!f) {
		akiko_diag("[akiko] NVR fopen(%s) failed: errno=%d", tmp_path, errno);
		return false;
	}
	size_t wrote = fwrite(buf, 1, AKIKO_NVRAM_BYTES, f);
	int    flush = fflush(f);
	int    fsy   = fsync(fileno(f));
	int    cls   = fclose(f);
	if (wrote != AKIKO_NVRAM_BYTES || flush != 0 || fsy != 0 || cls != 0) {
		akiko_diag("[akiko] NVR write incomplete: wrote=%zu flush=%d fsync=%d close=%d",
		           wrote, flush, fsy, cls);
		unlink(tmp_path);
		return false;
	}

	if (rename(tmp_path, path) != 0) {
		akiko_diag("[akiko] NVR rename(%s -> %s) failed: errno=%d",
		           tmp_path, path, errno);
		unlink(tmp_path);
		return false;
	}

	akiko_diag("[akiko] NVR saved %d bytes to %s", AKIKO_NVRAM_BYTES, path);
	return true;
}

// Save wrapper used by the dirty-debounce path in poll. Resolves to the
// per-game save filename if a CD path is active; otherwise falls back to
// the legacy single-file slot so writes that happen with no CD context
// (shouldn't normally occur, but cheap insurance) aren't silently dropped.
static bool akiko_nvram_save_to_disk(void)
{
	const char *target = (cd_save_path_active[0])
		? cd_save_path_active
		: AKIKO_NVRAM_FILE_LEGACY;
	return akiko_nvram_save_to_path(target);
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
		// Lightweight per-sector log throttled to one line every 64 sectors,
		// so the log volume stays low (~1500 lines for a full ~95k-sector
		// CD) while still letting us observe progress and detect stalls
		// (long gap between consecutive sec_req entries = CF stuck reading).
		if ((counter & 0x3F) == 0) {
			akiko_diag("[akiko] sec_req lba=%u counter=%u OK", lba, counter);
		}
		// TEMP DIAG: dump first 16 bytes of every read in low-LBA region
		// (CDFS bootstrap area) so we can see what CF is being fed.
		if (lba < 200) {
			akiko_diag("[akiko] sec_req lba=%u bytes[0..15]: "
				"%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
				lba, buf[0],buf[1],buf[2],buf[3], buf[4],buf[5],buf[6],buf[7],
				buf[8],buf[9],buf[10],buf[11], buf[12],buf[13],buf[14],buf[15]);
		}
	}

	akiko_push_sector(buf);
}

// -----------------------------------------------------------------------------
// Bus trace drain (debug)
// -----------------------------------------------------------------------------

// Drain the akiko_bus_trace ring buffer. ALWAYS COMPILED IN — the ring
// back-pressures the CPU when full, and an undrained ring stalls CF on
// dense bus activity (root cause of "stuck at lba 33"). Per-entry logging
// is gated on AKIKO_BUS_TRACE because at >90% of log volume during
// gameplay it dominates I/O cost; the bytes are still consumed off the
// ring, just discarded silently.
static void akiko_drain_trace(void)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(AKIKO_TRACE_ADDR);

	// Cap at 128 entries (ring depth, matches akiko_bus_trace.v) so a runaway
	// loop can't lock us up.
	for (int i = 0; i < 128; i++) {
		uint8_t b0 = (uint8_t)spi_w(0);  // {wr, addr[6:0]}
		uint8_t b1 = (uint8_t)spi_w(0);  // data[7:0]
		uint8_t b2 = (uint8_t)spi_w(0);  // data[15:8]
		uint8_t b3 = (uint8_t)spi_w(0);  // 0xFF valid, 0x00 empty

		if (b3 == 0) break;              // ring drained

#if AKIKO_BUS_TRACE
		bool     is_wr = (b0 & 0x80) != 0;
		uint32_t addr  = 0xB80000u | ((uint32_t)(b0 & 0x7F) << 1);
		uint16_t data  = (uint16_t)b1 | ((uint16_t)b2 << 8);

		akiko_diag("[trace] %s $%06X = 0x%04X", is_wr ? "W" : "R", addr, data);
#else
		(void)b0; (void)b1; (void)b2;
#endif
	}

	DisableIO();
}

// -----------------------------------------------------------------------------
// Top-level poll
// -----------------------------------------------------------------------------

// Phase 32.5.1 entrypoint: register the currently-mounted CD image's path
// so we can pick the right per-game save slot. Called from
// ide_cdrom.cpp::cdrom_parse on every mount/unmount. Empty string =
// unmount. Safe to call before akiko_cd32_init() — state is plain static
// memory and the actual SPI load is deferred to the poll loop.
//
// On a CD swap (different basename hash) we synchronously flush any dirty
// EEPROM contents to the OLD save file before swapping, so `mount A → write
// → mount B → write → mount A` preserves both saves correctly. The flush
// is skipped when we never observed dirty for the old slot — that avoids
// overwriting a real save with the synthesized FlashFile init from a
// different CD when the player just browsed past one image without
// running it.
void akiko_cd32_set_cd_path(const char *path)
{
	char new_path[256] = {0};
	if (path && *path) {
		compute_save_path(path, new_path, sizeof(new_path));
	}

	if (strcmp(new_path, cd_save_path_active) == 0) {
		// Same hash slot (or both empty) — nothing to do. Either the
		// user remounted the same CD or this is a pre-init no-op.
		return;
	}

	// Flush old save before swap, only if there's a chance writes happened
	// since the last load (the dirty-debounce poll path may have already
	// caught a write in flight; either way, dirty-observed gates this).
	if (cd_save_path_active[0] && cd_save_dirty_observed) {
		akiko_diag("[akiko] set_cd_path: flushing old save %s before swap",
		           cd_save_path_active);
		akiko_nvram_save_to_path(cd_save_path_active);
	}

	// Swap to the new slot.
	strncpy(cd_save_path_active, new_path, sizeof(cd_save_path_active) - 1);
	cd_save_path_active[sizeof(cd_save_path_active) - 1] = 0;
	cd_save_dirty_observed = false;

	if (!cd_save_path_active[0]) {
		cd_save_load_pending = false;
		akiko_diag("[akiko] set_cd_path: CD unmounted, no save file active");
		return;
	}

	// Arm deferred load. The poll path's first-mounted handler will load
	// the file into BRAM after the SPI_RST_USR pulse has finished wiping
	// it but before BIOS issues its first EEPROM I2C read. Sync load from
	// here is unsafe — it both gets wiped by the post-set_cd_path reset
	// AND its 1024-byte spi_w stream perturbs bridge state in a way that
	// breaks CF boot.
	cd_save_load_pending = true;
	akiko_diag("[akiko] set_cd_path: cd=%s -> save=%s (deferred load armed)",
	           path, cd_save_path_active);
}

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
	cd_audio_timeout = 0;
	cd_last_mounted  = false;
	toc_point_count  = 0;
	toc_push_idx     = -1;
	toc_push_throttle = 0;

	// Phase 32.5.1: a Minimig core reset wipes FPGA BRAM. If we already
	// have an active per-game save file (set by cdrom_parse before or
	// after this init), schedule a reload so soft `load_core` of Minimig
	// behaves the same as a hardware reset for save persistence. Without
	// this, the user sees "save vanished" after every core-reload loop.
	cd_save_dirty_observed = false;
	if (cd_save_path_active[0]) {
		cd_save_load_pending = true;
		akiko_diag("[akiko] init: scheduling reload of %s after BRAM wipe",
		           cd_save_path_active);
	}

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
	// Phase 32.5 — REQUIRED throttle. Without this, the poll runs faster
	// than the FPGA bridge can update its status word, and we see stale
	// rx_busy/sec_req bits. CF then either pushes responses on top of
	// each other or misses sec_req cycles, and stalls in a tight
	// LED→PAUSE→PLAY loop reading lba 5-33 forever. Empirical: 200µs
	// (~5000 polls/sec) reliably gets CF past the filesystem area into
	// game data (lba 25k+); the heavier per-cmd akiko_dbg logging in
	// pre-Phase-32.5 builds happened to add ~150µs/cmd which masked the
	// problem. Don't remove without first proving the bridge can drive
	// status updates at <200µs latency.
	usleep(200);

	bool mounted = cd_is_mounted();

	// Heartbeat: prove poll loop reached us at all. Writes to a dedicated
	// log file so it survives any stdout redirection MiSTer does after init.
	// Phase 32.5.1: NVRAM load is no longer driven from here — it's
	// scheduled by akiko_cd32_set_cd_path() (CD mount/swap) and
	// akiko_cd32_init() (post-reconfig BRAM wipe), and serviced below
	// once `mounted` is true. That way each Minimig load_core re-loads,
	// not just hardware boots, and per-game saves work correctly.
	static bool first_poll = true;
	if (first_poll) {
		first_poll = false;
		akiko_diag("[akiko] poll alive (first call) mounted=%d", mounted);
	}

	// Phase 32.5.1 deferred load. Triggered by set_cd_path() or init().
	// Only fires once mounted=1 so we don't load before the CHD is open
	// (early load would still work — it's purely a BRAM write — but the
	// log line is more useful when correlated with the mount event).
	if (cd_save_load_pending && mounted) {
		cd_save_load_pending = false;
		if (cd_save_path_active[0]) {
			// One-shot legacy migration: if the per-game file doesn't
			// exist but the legacy single-file save does, load the
			// legacy data into the per-game slot for this CD. Useful
			// for the user's existing cd32.nvr that pre-dates the
			// per-game scheme.
			struct stat pst;
			if (stat(cd_save_path_active, &pst) != 0 &&
			    stat(AKIKO_NVRAM_FILE_LEGACY, &pst) == 0 &&
			    pst.st_size == AKIKO_NVRAM_BYTES) {
				akiko_diag("[akiko] migrating legacy save %s -> %s",
				           AKIKO_NVRAM_FILE_LEGACY, cd_save_path_active);
				akiko_nvram_load_from_path(AKIKO_NVRAM_FILE_LEGACY);
			} else {
				akiko_nvram_load_from_path(cd_save_path_active);
			}
		}
	}

	// Always drain — the ring back-pressures the CPU when full. Per-entry
	// logging is gated inside the function on AKIKO_BUS_TRACE.
	akiko_drain_trace();

#if AKIKO_CD32_DEBUG
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
#endif

	// Re-arm auto-init if media was swapped or just inserted.
	if (mounted != cd_last_mounted) {
		cd_initialized   = 0;
		cd_data_lba_base = -1;               // any in-progress read is stale
		toc_point_count  = 0;                // force TOC rebuild after next INFO
		toc_push_idx     = -1;
		cd_last_mounted  = mounted;
		akiko_diag("[akiko] media change -> mounted=%d", mounted);
	}

	// 0. Status poll moved up so unsolicited pushes (auto-init, post-INFO,
	// TOC drip) can be gated on FPGA rx_busy. Phase 18: bit[9] = rx_busy =
	// (cdrom_receive_length != 0). When set, the RX engine still has a
	// queued/in-flight response in result_buffer; pushing now would
	// overwrite it and lose data. Mirror of WinUAE's
	// cdrom_can_return_data() gate.
	uint16_t status = akiko_read_status();
#if AKIKO_CD32_DEBUG
	static uint16_t last_status = 0xffff;
	static int status_log_count = 0;
	if (status != last_status && status_log_count < 200) {
		akiko_diag("[akiko] status=0x%04x (was 0x%04x)", status, last_status);
		last_status = status;
		status_log_count++;
	}
#endif
	const bool rx_idle = !(status & AKIKO_STATUS_RX_BUSY);

	// 1. Auto-init: WinUAE akiko.cpp:1388-1392 pushes a media-status frame
	// when cd_initialized == 0 && media-present. BIOS uses this to learn
	// "media is here" before issuing LED/INFO. Phase 16: re-enabled now
	// that Phase 14 NVRAM and Phase 15 partial-RX delivery are working.
	// Without this, our trace shows BIOS doing LED+INFO drains via the
	// partial-RX path (RXCMP bumped 4 → 30) but never issuing PLAY/MULTI:
	// it's still in "expecting media-status" mode after INFO.
	if (mounted && cd_initialized == 0 && rx_idle) {
		uint8_t r[2];
		r[0] = 0x0a;                                         // CDS_MEDIA_STATUS opcode
		r[1] = 0x01;                                         // media present
		akiko_send_response(r, 2);
		cd_initialized = 1;
		akiko_diag("[akiko] auto-init media-status push: opcode=0x0a status=0x01 (cd_initialized=1)");
	}

	// Phase 20: post-INFO media-status push DISABLED (was Phase 15-19.x).
	// Subagent diff vs WinUAE akiko.cpp:1399-1407 shows the post-INFO push
	// is GATED on `mediachanged == 1`, which the auto-init push at
	// cd_initialized==0 already CONSUMED (akiko.cpp:1390-1392). On a cold
	// boot WinUAE therefore fires post-INFO ZERO times. Our unconditional
	// `cd_post_info_media_push_pending=1` in cmd_info was sending a
	// duplicate/spurious frame after BIOS had already accepted INFO,
	// likely racing the BIOS state machine and preventing it from
	// advancing to LED(1)/PLAY/MULTI. Drop it entirely to match WinUAE's
	// cold-boot behavior; only the auto-init push above should fire.
	(void)cd_post_info_media_push_pending;

	// 1.5 Auto-TOC drip: re-enabled in Phase 17, gated on rx_idle in Phase 18.
	// BIOS RXCMP after INFO+post-INFO goes 4,6,7,...,15,30 — the +15 jump from
	// RXCMP=15 to 30 indicates BIOS is waiting for a 15-byte TOC frame to land
	// at offset 30. WinUAE pushes one TOC entry per video frame
	// (akiko.cpp:1438-1440). Throttle accumulates only when the engine is
	// idle — pushing into a busy buffer would silently drop the entry.
	if (cd_initialized == 2 && toc_push_idx >= 0 && rx_idle) {
		// Throttle: WinUAE pushes one TOC entry per video frame (~50 Hz).
		// Phase 19.6: prior /20 throttle bursted all 15 frames in ~1s
		// (Phase 18 measured ~60 kHz effective poll rate). /1200 brings
		// us roughly back to 50 Hz and gives BIOS time to process each
		// frame's RXDMADONE before the next overwrite.
		if (++toc_push_throttle >= 1200) {
			toc_push_throttle = 0;
			akiko_push_toc_entry();
		}
	}

	// Phase 32: NVRAM persistence. RTL latches nvr_dirty whenever BIOS
	// writes a byte through the I2C path; userspace dumps + saves once
	// the dirty state has held steady for the debounce window (BIOS
	// FlashFile commits are bursty — 16+ writes back-to-back). Gate the
	// dump itself on rx_idle so a save doesn't interleave with a queued
	// MULTI response, but the timestamp tracking runs unconditionally.
	{
		static uint32_t nvr_dirty_seen_at = 0;     // 0 = not currently dirty
		const bool nvr_dirty_now = (status & AKIKO_STATUS_NVR_DIRTY) != 0;
		if (nvr_dirty_now) {
			cd_save_dirty_observed = true;     // arms set_cd_path flush
			uint32_t now_ms = (uint32_t)GetTimer(0);
			if (!nvr_dirty_seen_at) {
				nvr_dirty_seen_at = now_ms;
				akiko_diag("[akiko] NVR dirty observed at t=%u", now_ms);
			} else if (rx_idle &&
			           (now_ms - nvr_dirty_seen_at) >= AKIKO_NVRAM_DIRTY_DEBOUNCE_MS) {
				// Phase 32.5: bridge auto-clears dirty at end of the
				// READ burst inside save_to_disk's nvram_dump, so no
				// explicit clear-dirty SPI write is needed (and would
				// in fact corrupt byte 0 under the new write-as-load
				// bridge semantics).
				akiko_nvram_save_to_disk();
				nvr_dirty_seen_at = 0;
			}
		} else {
			// Flag dropped (RTL reset, or we just cleared it). Re-arm.
			nvr_dirty_seen_at = 0;
		}
	}

	// Audio-play notification state machine (mirrors WinUAE akiko_handler,
	// akiko.cpp:1411-1435). Only steps when the RX engine is idle so we
	// never overwrite an in-flight reply with the play-state frame.
	// Decrements through 2->1 then emits "play started"; advances -1->-2
	// then emits "play ended"; emits "play failed" on -3.
	if (cd_audio_timeout != 0 && rx_idle) {
		if (cd_audio_timeout > 1) {
			cd_audio_timeout--;
		} else if (cd_audio_timeout == 1) {
			emit_playend_notify(0);
			cd_audio_timeout = 0;
		} else if (cd_audio_timeout == -1) {
			// Phase 33 will tear down the CDDA pump here. For now we
			// just advance to the play-ended emission.
			cd_audio_timeout = -2;
		} else if (cd_audio_timeout == -2) {
			emit_playend_notify(1);
			cd_playing = 0;
			cd_audio_timeout = 0;
		} else if (cd_audio_timeout == -3) {
			emit_playend_notify(-1);
			cd_playing = 0;
			cd_audio_timeout = 0;
		}
		return;                              // one bridge action per poll
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

#if AKIKO_CD32_DEBUG
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
