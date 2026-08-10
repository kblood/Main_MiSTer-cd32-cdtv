#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void cpu_trace_drain(void);
void cpu_trace_arm(unsigned max_rows);

#ifdef __cplusplus
}
#endif
