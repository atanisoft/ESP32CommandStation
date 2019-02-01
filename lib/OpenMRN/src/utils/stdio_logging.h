/// Include this file into main.cxx to log stuff to stdio on FreeRTOS.

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__linux__) || defined(__MACH__) || defined(__EMSCRIPTEN__)
#define LOGWEAK __attribute__((weak))
#else
#define LOGWEAK
#endif

LOGWEAK void log_output(char* buf, int size) {
    if (size <= 0) return;
    fwrite(buf, size, 1, stderr);
    fwrite("\n", 1, 1, stderr);
}

#ifdef __cplusplus
}
#endif
