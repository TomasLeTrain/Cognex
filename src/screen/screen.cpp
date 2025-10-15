#include "apis.h"
//

#include "screen.h"

namespace screen {
// screens

lv_obj_t** curr_screen = &tabs::screen;

void change_screen(lv_event_t* e) {
    lv_obj_t* next_screen = (lv_obj_t*)lv_event_get_user_data(e);
    if (next_screen != nullptr) {
        lv_obj_add_flag(*curr_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(next_screen, LV_OBJ_FLAG_HIDDEN);
        curr_screen = &next_screen;
    }
}

void setScreen(lv_obj_t** new_screen) {
    if (new_screen != nullptr) {
        lv_obj_add_flag(*curr_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(*new_screen, LV_OBJ_FLAG_HIDDEN);
        curr_screen = new_screen;
    }
}

void makeCurrentActive() {
    lv_obj_remove_flag(*curr_screen, LV_OBJ_FLAG_HIDDEN);
}

// initializes all the screens
void init() {
    tabs::init(lv_screen_active());
    bouncing_dvd_screen::init(lv_screen_active());

    health::init(tabs::tabs[0], tabs::tabs[1], tabs::tabs[2]);
    auton_select::init(tabs::tabs[3]);
	// also show auton_select
    lv_obj_remove_flag(auton_select::screen, LV_OBJ_FLAG_HIDDEN);

    makeCurrentActive();
}
} // namespace screen
