#include "apis.h"
#include "liblvgl/lvgl_mutex.h"
#include "pros/rtos.h"
//

#include "screen.h"

namespace screen {
// screens

lv_obj_t** curr_screen = &tabs::screen;

void get_screen_mutex() {
    pros::c::mutex_recursive_take(pros::c::_lvgl_mutex, TIMEOUT_MAX);
}

void give_screen_mutex() {
    pros::c::mutex_recursive_give(pros::c::_lvgl_mutex);
}

void change_screen(lv_event_t* e) {
    get_screen_mutex();
    lv_obj_t* next_screen = (lv_obj_t*)lv_event_get_user_data(e);
    if (next_screen != nullptr) {
        lv_obj_add_flag(*curr_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(next_screen, LV_OBJ_FLAG_HIDDEN);
        curr_screen = &next_screen;
    }
    give_screen_mutex();
}

void setScreen(lv_obj_t** new_screen) {
    get_screen_mutex();
    if (new_screen != nullptr) {
        lv_obj_add_flag(*curr_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(*new_screen, LV_OBJ_FLAG_HIDDEN);
        curr_screen = new_screen;
    }
    give_screen_mutex();
}

void makeCurrentActive() {
    get_screen_mutex();
    lv_obj_remove_flag(*curr_screen, LV_OBJ_FLAG_HIDDEN);
    give_screen_mutex();
}

// initializes all the screens
void init() {
    get_screen_mutex();

    tabs::init(lv_screen_active());

    bouncing_dvd_screen::init(lv_screen_active());
    std::cout << "bouncing done" << std::endl;

    health::init(tabs::tabs[0], tabs::tabs[1], tabs::tabs[2]);
    std::cout << "health done" << std::endl;
    auton_select::init(tabs::tabs[3]);
    std::cout << "auton done" << std::endl;
    // // also show auton_select
    lv_obj_remove_flag(auton_select::screen, LV_OBJ_FLAG_HIDDEN);

    makeCurrentActive();

    give_screen_mutex();
}
} // namespace screen
