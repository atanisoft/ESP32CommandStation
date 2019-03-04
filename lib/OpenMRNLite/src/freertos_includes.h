#ifdef ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define NSEC_TO_TICK(ns)                                                       \
    (((((long long)(ns)) / 1000 * configTICK_RATE_HZ) + 999999) / 1000000)

#else

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define NSEC_TO_TICK(ns) ((ns) >> NSEC_TO_TICK_SHIFT)

#endif
