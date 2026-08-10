#include "cpu_trace.h"

#include <cstdio>
#include <cstdint>
#include <sys/stat.h>

#include "../../spi.h"
#include "../../user_io.h"
#include "../../hardware.h"

static constexpr uint32_t CPU_TRACE_ADDR = 0xFE00;
static constexpr int      RING_DEPTH     = 512;
static const char *OUT_PATH  = "/tmp/cpu_trace.csv";

static const char *kEv[16] = {
    "fetch", "stop_enter", "stop_exit", "ipl", "heartbeat", "window",
    "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv", "rsv"
};

static constexpr unsigned SAMPLE_PERIOD_MS = 500;

static FILE *g_csv = nullptr;
static uint32_t g_seq = 0;
static uint32_t g_skipped = 0;
static uint32_t g_batches = 0;
static uint32_t g_stale = 0;
static uint32_t g_last_first_ts = 0;
static uint32_t g_last_last_ts = 0;
static unsigned g_budget = 0;
static unsigned long g_next_ms = 0;

void cpu_trace_arm(unsigned max_batches)
{
    if (g_budget) return;
    g_budget  = max_batches ? max_batches : 1;
    g_next_ms = GetTimer(0);
}

void cpu_trace_drain(void)
{
    if (!g_budget) return;
    if (!CheckTimer(g_next_ms)) return;
    g_next_ms = GetTimer(SAMPLE_PERIOD_MS);

    if (!g_csv) {
        g_csv = fopen(OUT_PATH, "w");
        if (!g_csv) return;
        fprintf(g_csv,
                "seq,batch,ts,ev,pc,stop,sv,cpustate,ipl_raw,ipl_lvl,is_l2,tg68k,opcode,stoplen_hi\n");
    }

    g_batches++;

    EnableIO();
    spi8(UIO_DMA_READ);
    spi32_w(CPU_TRACE_ADDR);

    int got = 0;
    uint32_t first_ts = 0, last_ts = 0;
    for (int i = 0; i < RING_DEPTH; i++) {
        uint8_t b[16];
        for (int j = 0; j < 16; j++) b[j] = (uint8_t)spi_w(0);

        uint32_t all_or = 0;
        for (int j = 0; j < 16; j++) all_or |= b[j];
        if (!all_or) break;

        uint8_t ev_nib = (b[8] >> 4) & 0xF;
        bool sane = (ev_nib == 1 || ev_nib == 5) && !b[14] && !b[15];
        if (!sane) {
            g_skipped++;
            break;
        }

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
        uint8_t tg68k    = (e9 >> 4) & 0x1;
        uint8_t is_l2    =  e9       & 0x1;
        uint8_t ipl_lvl  = (~ipl_raw) & 0x7;
        uint16_t slen    = (uint16_t)b[10] | ((uint16_t)b[11] << 8);
        uint16_t opc     = (uint16_t)b[12] | ((uint16_t)b[13] << 8);

        if (!got) first_ts = ts;
        last_ts = ts;

        fprintf(g_csv,
                "%u,%u,%u,%s,0x%08X,%u,%u,%u,%u,%u,%u,%u,0x%04X,%u\n",
                g_seq++, g_batches, ts, kEv[ev], pc, stop, sv, cpustate,
                ipl_raw, ipl_lvl, is_l2, tg68k, opc, slen);
        got++;
    }

    DisableIO();

    bool stale = got && (first_ts == g_last_first_ts) && (last_ts == g_last_last_ts);
    if (stale) g_stale++;
    g_last_first_ts = first_ts;
    g_last_last_ts  = last_ts;

    fprintf(g_csv, "# batch=%u rows=%d stale=%u resync_breaks=%u\n",
            g_batches, got, stale ? 1u : 0u, g_skipped);
    fflush(g_csv);

    if (!--g_budget) {
        fprintf(g_csv, "# done batches=%u stale=%u resync_breaks=%u\n",
                g_batches, g_stale, g_skipped);
        fclose(g_csv);
        g_csv = nullptr;
    }
}
