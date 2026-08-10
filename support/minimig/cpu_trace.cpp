#include "cpu_trace.h"

#include <cstdio>
#include <cstdint>
#include <sys/stat.h>

#include "../../spi.h"
#include "../../user_io.h"

static constexpr uint32_t CPU_TRACE_ADDR = 0xFE00;
static constexpr int      RING_DEPTH     = 512;
static const char *GATE_PATH = "/tmp/cpu_trace_on";
static const char *OUT_PATH  = "/tmp/cpu_trace.csv";

static const char *kEv[16] = {
    "fetch", "stop_enter", "stop_exit", "ipl", "heartbeat", "window",
    "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv"
};

static FILE *g_csv = nullptr;
static uint32_t g_seq = 0;
static bool g_gate_prev = false;
static bool g_armed = false;
static unsigned g_rows_left = 0;

void cpu_trace_arm(unsigned max_rows)
{
    if (g_armed) return;
    g_armed = true;
    g_rows_left = max_rows;
}

void cpu_trace_drain(void)
{
    struct stat st;
    bool on = g_armed ? (g_rows_left > 0) : (stat(GATE_PATH, &st) == 0);

    if (!on && g_gate_prev && g_csv) {
        fclose(g_csv);
        g_csv = nullptr;
    }
    g_gate_prev = on;
    if (g_armed && !g_rows_left && g_csv) {
        fclose(g_csv);
        g_csv = nullptr;
    }

    EnableIO();
    spi8(UIO_DMA_READ);
    spi32_w(CPU_TRACE_ADDR);

    for (int i = 0; i < RING_DEPTH; i++) {
        uint8_t b[16];
        for (int j = 0; j < 16; j++) b[j] = (uint8_t)spi_w(0);

        uint32_t all_or = 0;
        for (int j = 0; j < 16; j++) all_or |= b[j];
        if (!all_or) break;

        if (!on) continue;

        if (!g_csv) {
            g_csv = fopen(OUT_PATH, "w");
            if (g_csv) {
                fprintf(g_csv,
                        "seq,ts,ev,pc,stop,sv,cpustate,ipl_raw,ipl_lvl,is_l2,int2,skipfetch,win\n");
                g_seq = 0;
            }
        }
        if (!g_csv) break;

        uint32_t ts = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                      ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
        uint32_t pc = (uint32_t)b[4] | ((uint32_t)b[5] << 8) |
                      ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
        uint8_t e8 = b[8], e9 = b[9];
        uint8_t ev       = (e8 >> 4) & 0xF;
        uint8_t stop     = (e8 >> 3) & 0x1;
        uint8_t sv       = (e8 >> 2) & 0x1;
        uint8_t cpustate =  e8       & 0x3;
        uint8_t ipl_raw  = (e9 >> 5) & 0x7;
        uint8_t int2     = (e9 >> 4) & 0x1;
        uint8_t skipf    = (e9 >> 3) & 0x1;
        uint8_t is_l2    =  e9       & 0x1;
        uint8_t ipl_lvl  = (~ipl_raw) & 0x7;
        uint16_t win     = (uint16_t)b[10] | ((uint16_t)b[11] << 8);

        fprintf(g_csv,
                "%u,%u,%s,0x%08X,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                g_seq++, ts, kEv[ev], pc, stop, sv, cpustate,
                ipl_raw, ipl_lvl, is_l2, int2, skipf, win);
    }

    DisableIO();
    if (g_csv) fflush(g_csv);
}
