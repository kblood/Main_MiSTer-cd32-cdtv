#include "chipset_trace.h"

#include <cstdio>
#include <cstdint>
#include <sys/stat.h>

#include "../../spi.h"
#include "../../user_io.h"
#include "../../hardware.h"

// io_din command word selecting chipset_bus_trace's dedicated UIO
// sub-channel: class 7'b1111011, i.e. io_din[15:9] == 7'b1111011 with the
// rest of the 16-bit word zero -> 0xF600. See hps_ext.v and
// rtl/chipset_bus_trace.v's header comment (this class is NOT shared with
// cpu_trace/channelb_trace's 0xFE00/0xFF00 class).
static constexpr uint32_t CHIPSET_TRACE_ADDR = 0xF600;
static constexpr int      RING_DEPTH         = 1024;
static const char *OUT_PATH   = "/tmp/chipset_trace.csv";
static const char *ARM_MARKER = "/tmp/chipset_trace_on";

// Poll period. This is BOTH the drain cadence and the capture's only wall
// clock, so it sets analysis resolution as well as how much the ring has to
// absorb between drains. It was 200 ms, which is 10 PAL frames -- far longer
// than the ring can hold at this title's event rate, so most batches ended up
// dropping. 50 ms is 2.5 PAL frames and gives 4x the drain opportunities at
// the same SPI cost per event. 20 ms is one PAL frame.
static constexpr unsigned SAMPLE_PERIOD_MS = 20;

// Entries to pull in one drain, distinct from the ring's depth. Reading at
// most RING_DEPTH per poll caps sustained throughput at RING_DEPTH /
// SAMPLE_PERIOD_MS regardless of how fast SPI actually is -- 1024 per 50 ms
// was 20,480 events/s, and the row-signature instrument offers about 22,800,
// so every capture bled a steady 23% into GAP records. The loop already stops
// on the ring-empty sentinel, so this is only a runaway bound: the drain now
// keeps reading until the ring is actually empty.
static constexpr int MAX_DRAIN_PER_POLL = 65536;

static const char *SRC_LABELS[8] = {
    "cpu", "cop", "blt", "spr", "bpl", "dsk", "aud", "ref",
};

static FILE *g_csv = nullptr;
static bool  g_armed = false;
static uint32_t g_seq = 0;
static uint32_t g_batches = 0;
static uint32_t g_dropped_total = 0;
static unsigned long g_next_ms = 0;

static bool marker_present(void)
{
    struct stat st;
    return stat(ARM_MARKER, &st) == 0;
}

void chipset_trace_drain(void)
{
    bool want = marker_present();

    if (want && !g_armed) {
        g_armed   = true;
        g_next_ms = GetTimer(0);
        // These are file-scope statics; without this a second capture in the
        // same MiSTer process resumes the previous batch/seq numbering and
        // batch*SAMPLE_PERIOD_MS stops being elapsed time.
        g_seq           = 0;
        g_batches       = 0;
        g_dropped_total = 0;
    }
    if (!want && g_armed) {
        g_armed = false;
        if (g_csv) {
            fprintf(g_csv, "# done batches=%u dropped_total=%u\n",
                g_batches, g_dropped_total);
            fclose(g_csv);
            g_csv = nullptr;
        }
    }
    if (!g_armed) return;
    if (!CheckTimer(g_next_ms)) return;
    g_next_ms = GetTimer(SAMPLE_PERIOD_MS);

    if (!g_csv) {
        g_csv = fopen(OUT_PATH, "w");
        if (!g_csv) return;
        // Self-describing header. batch*period is the capture's only
        // wall clock, so a consumer that assumes the wrong period
        // silently rescales every timestamp. Emit it rather than let
        // the analysis tools hardcode a value that drifts out of sync
        // with this constant.
        fprintf(g_csv, "# period_ms=%u\n", SAMPLE_PERIOD_MS);
        fprintf(g_csv, "seq,vpos,hpos,src,reg,data,dbwe\n");
    }

    g_batches++;

    EnableIO();
    spi8(UIO_DMA_READ);
    spi32_w(CHIPSET_TRACE_ADDR);

    int got = 0;
    int dropped_this_batch = 0;
    for (int i = 0; i < MAX_DRAIN_PER_POLL; i++) {
        uint8_t b[8];
        for (int j = 0; j < 8; j++) b[j] = (uint8_t)spi_w(0);

        // Byte 7 is the RTL's own sentinel: 0xFF a normal entry, 0xFE a GAP
        // record, anything else (0x00) means the ring was empty at drain
        // time. Unlike channelb_trace this protocol needs no all-zero
        // heuristic.
        if (b[7] != 0xFF && b[7] != 0xFE) break;

        if (b[7] == 0xFE) {
            // The ring filled and refused writes rather than overwriting
            // unread entries. data[15:0] is the exact number of events lost
            // (saturating at 65535); vpos/hpos are where it recovered.
            uint32_t lost  = (uint32_t)b[0] | ((uint32_t)b[1] << 8);
            uint32_t gvpos = ((uint32_t)(b[3] & 0x7) << 8) | b[4];
            uint32_t ghpos = ((uint32_t)(b[6] & 0x1) << 8) | b[5];
            fprintf(g_csv, "# gap seq=%u dropped=%u vpos=%u hpos=%u\n",
                    g_seq, lost, gvpos, ghpos);
            dropped_this_batch += (int)lost;
            g_dropped_total    += lost;
            continue;
        }

        uint32_t data    = (uint32_t)b[0] | ((uint32_t)b[1] << 8);
        uint32_t reg     = b[2];
        uint8_t  ctxhi   = b[3];   // {dbwe, src[2:0], 1'b0, vpos[10:8]}
        uint32_t dbwe    = (ctxhi >> 7) & 0x1;
        uint32_t src     = (ctxhi >> 4) & 0x7;
        uint32_t vpos_hi = ctxhi & 0x7;
        uint32_t vpos    = (vpos_hi << 8) | b[4];
        uint32_t hpos_hi = b[6] & 0x1;
        uint32_t hpos    = (hpos_hi << 8) | b[5];

        fprintf(g_csv, "%u,%u,%u,%s,0x%02X,0x%04X,%u\n",
                g_seq++, vpos, hpos, SRC_LABELS[src], reg, data, dbwe);
        got++;
    }

    DisableIO();

    fprintf(g_csv, "# batch=%u rows=%d dropped=%d\n",
            g_batches, got, dropped_this_batch);
    fflush(g_csv);
}
