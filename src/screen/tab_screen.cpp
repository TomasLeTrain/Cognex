#include "apis.h"
//

#include "liblvgl/lvgl.h"
#include "screen.h"

namespace screen {
namespace tabs {
lv_obj_t* screen;

lv_obj_t* tabs[num_tabs];

std::string tab_names[num_tabs] = {
    "Status",
    "Errors",
    "Console",
    "Selector",
};

// used for each tab object
static lv_style_t zero_padding;
static lv_style_t no_round;

// called once inside init, no need to make thread safe
void initStyles() {
    // zero padding style
    lv_style_init(&zero_padding);
    lv_style_set_pad_all(&zero_padding, 0);

    lv_style_init(&no_round);
    lv_style_set_radius(&no_round, 0);
}

void create_tabs(lv_obj_t* parent_screen) {
    get_screen_mutex();

    lv_obj_t* tabview;

    tabview = lv_tabview_create(parent_screen);
    lv_tabview_set_tab_bar_position(tabview, LV_DIR_RIGHT);
    lv_tabview_set_tab_bar_size(tabview, 60);

    lv_obj_t* tab_buttons = lv_tabview_get_tab_bar(tabview);
    lv_obj_set_style_bg_color(tab_buttons,
                              lv_palette_darken(LV_PALETTE_GREY, 3),
                              0);
    lv_obj_set_style_text_color(tab_buttons,
                                lv_palette_lighten(LV_PALETTE_GREY, 5),
                                0);

    /*Add 5 tabs (the tabs are page (lv_page) and can be scrolled*/
    for (int i = 0; i < num_tabs; i++) {
        tabs[i] = lv_tabview_add_tab(tabview, tab_names[i].c_str());
        lv_obj_add_style(tabs[i], &zero_padding, 0);
        lv_obj_add_style(tabs[i], &no_round, 0);
    }

    // don't make scrollable
    lv_obj_remove_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

    give_screen_mutex();
}

void init(lv_obj_t* parent_screen) {
    get_screen_mutex();

    initStyles();

    // main screen object
    screen = lv_obj_create(parent_screen);

    // makes object take up the full screen and have no styling
    lv_obj_remove_style_all(screen);
    lv_obj_set_size(screen,
                    lv_display_get_horizontal_resolution(NULL),
                    lv_display_get_vertical_resolution(NULL));
    lv_obj_center(screen);

    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_100, 0);

    // hidden by default
    lv_obj_add_flag(screen, LV_OBJ_FLAG_HIDDEN);

    create_tabs(screen);

    give_screen_mutex();
}

} // namespace tabs
} // namespace screen
