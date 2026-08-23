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

static constexpr unsigned SAMPLE_PERIOD_MS = 200;

static const char *SRC_LABELS[8] = {
    "cpu", "cop", "blt", "spr", "bpl", "dsk", "aud", "ref",
};

static FILE *g_csv = nullptr;
static bool  g_armed = false;
static uint32_t g_seq = 0;
static uint32_t g_batches = 0;
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
    }
    if (!want && g_armed) {
        g_armed = false;
        if (g_csv) {
            fprintf(g_csv, "# done batches=%u\n", g_batches);
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
        fprintf(g_csv, "seq,vpos,hpos,src,reg,data,dbwe\n");
    }

    g_batches++;

    EnableIO();
    spi8(UIO_DMA_READ);
    spi32_w(CHIPSET_TRACE_ADDR);

    int got = 0;
    for (int i = 0; i < RING_DEPTH; i++) {
        uint8_t b[8];
        for (int j = 0; j < 8; j++) b[j] = (uint8_t)spi_w(0);

        // Byte 7 is the RTL's own valid sentinel (0xFF valid, 0x00 -> ring
        // was empty at drain time); unlike channelb_trace this protocol
        // does not need an all-zero heuristic.
        if (b[7] != 0xFF) break;

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

    fprintf(g_csv, "# batch=%u rows=%d\n", g_batches, got);
    fflush(g_csv);
}
