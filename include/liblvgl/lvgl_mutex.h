#ifndef LVGL_MUTEX_H
#define LVGL_MUTEX_H

#include "pros/rtos.h"

#ifdef __cplusplus
extern "C" {
namespace pros {
namespace c {
#endif

// mutex to allow interacting with lvgl in a thread safe manner
extern mutex_t _lvgl_mutex;

#ifdef __cplusplus
} // namespace c
} // namespace pros
}
#endif

#endif /*LVGL_H*/

