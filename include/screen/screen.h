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

LV_IMAGE_DECLARE(monkey_75x_img);

namespace screen {
extern lv_obj_t** curr_screen;

void change_screen(lv_event_t* e);
void setScreen(lv_obj_t** new_screen);

namespace tabs {
extern lv_obj_t* screen;
inline const int num_tabs = 4;
extern lv_obj_t* tabs[num_tabs];
void init(lv_obj_t* parent_screen);

} // namespace tabs

namespace health {
extern lv_obj_t* screen;

enum notification_severity_t {
    critical,
    warn,
    succeed
};

void add_notification(
  std::string title_text,
  std::string detail_text,
  notification_severity_t severity = notification_severity_t::warn);

// returns the index of the notification
int add_init_notif(
  std::string title_text,
  notification_severity_t severity = notification_severity_t::warn);

void update_init_notif_severity(int index,
                                notification_severity_t new_severity);

void set_console_text(std::string text);
void console_println(std::string text);

void init(lv_obj_t* error_parent_screen,
          lv_obj_t* status_parent_screen,
          lv_obj_t* console_parent_screen);

} // namespace health

namespace bouncing_dvd_screen {
extern lv_obj_t* screen;
void init(lv_obj_t* parent_screen);
}

namespace auton_select {
extern lv_obj_t* screen;
void init(lv_obj_t* parent_screen);
} // namespace auton_select

void init();
} // namespace screen
