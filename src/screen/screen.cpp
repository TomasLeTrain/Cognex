#include "screen.h"

namespace screen {
// screens
lv_obj_t* auton_select_screen;
lv_obj_t* debug_screen;
lv_obj_t* dvd_screen;

lv_obj_t ** curr_screen = &auton_select_screen;

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
    bouncing_dvd_screen::init();
    auton_select::init();

    makeCurrentActive();
}
} // namespace screen
