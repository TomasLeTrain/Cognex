#pragma once

#include "apis.h"
//

#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/misc/lv_anim.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "pros/apix.h" // IWYU pragma: keep
#include <math.h>
#include <stdio.h>

LV_IMAGE_DECLARE(dvd_img);
LV_IMAGE_DECLARE(monkey_img);
LV_IMAGE_DECLARE(monkey_4x_img);
LV_IMAGE_DECLARE(monkey_75x_img);

#define SCREEN(name) \
    namespace name { \
    void init();     \
    }

namespace screen {
extern lv_obj_t* auton_select_screen;
extern lv_obj_t* dvd_screen;

extern lv_obj_t** curr_screen;

void change_screen(lv_event_t* e);
void setScreen(lv_obj_t** new_screen);

SCREEN(bouncing_dvd_screen)
SCREEN(auton_select)

void init();
} // namespace screen
