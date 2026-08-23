#pragma once

// Chipset bus trace host-side drain (2026-08-23 cycle-diff matrix goal).
// See rtl/chipset_bus_trace.v (Minimig-AGA_MiSTer, worktree
// Minimig-AGA_MiSTer-wt-hybris-trace, branch hybris-blit-vpos-trace) for the
// RTL side. Mirrors the channelb_trace.h/.cpp marker-armed drain pattern.

#ifdef __cplusplus
extern "C" {
#endif

// Called once per user_io poll (see the channelb_trace_drain() call site in
// user_io.cpp). No-op unless /tmp/chipset_trace_on exists -- creating that
// marker file arms continuous draining to /tmp/chipset_trace.csv in the
// canonical schema (seq,vpos,hpos,src,reg,data,dbwe) shared with
// tools/winuae_chipset_log_parse.py and tools/chipset_trace_compare.py;
// removing the marker closes the CSV.
void chipset_trace_drain(void);

#ifdef __cplusplus
}
#endif
