#include "autos.h"
#include "globals.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/draw/lv_draw_rect.h"
#include "liblvgl/font/lv_font.h"
#include "liblvgl/font/lv_symbol_def.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/misc/lv_palette.h"
#include "map"
#include "screen.h"

namespace screen {
namespace auton_select {
// this is the screen that becomes active after this one is done
lv_obj_t* next_screen = nullptr;

std::map<lv_obj_t*, std::string> radio_to_auton_mode;

// auton screen objects
lv_obj_t* field_btns[4];

uint32_t last_selected_checkbox = -1;

void setAuton(std::string new_auton) {
    selected_auton = new_auton;
}

void setFieldSide(field_side_t new_side) {
    auto_side = new_side;
}

void setAlliance(alliance_t new_alliance) {
    auto_alliance = new_alliance;
}

void update_fields() {
    static lv_style_t selected_style, not_selected_style;
    lv_style_init(&selected_style);
    lv_style_init(&not_selected_style);
    lv_style_set_bg_opa(&selected_style, LV_OPA_100);
    lv_style_set_bg_opa(&not_selected_style, LV_OPA_40);

    for (int i = 0; i < 4; i++) {
        lv_obj_add_style(field_btns[i], &not_selected_style, 0);
    }

    if (auto_alliance == alliance_t::blue) {
        lv_obj_add_style(field_btns[0], &selected_style, 0);
    } else if (auto_alliance == alliance_t::red) {
        lv_obj_add_style(field_btns[1], &selected_style, 0);
    }

    if (auto_side == field_side_t::left) {
        lv_obj_add_style(field_btns[2], &selected_style, 0);
    } else if (auto_side == field_side_t::right) {
        lv_obj_add_style(field_btns[3], &selected_style, 0);
    }
}

void left_cb(lv_event_t* e) {
    setFieldSide(field_side_t::left);
    update_fields();
}

void right_cb(lv_event_t* e) {
    setFieldSide(field_side_t::right);
    update_fields();
}

void red_cb(lv_event_t* e) {
    setAlliance(alliance_t::red);
    update_fields();
}

void blue_cb(lv_event_t* e) {
    setAlliance(alliance_t::blue);
    update_fields();
}

lv_obj_t* make_button(lv_obj_t* holder,
                      int posX,
                      int posY,
                      int width,
                      int height,
                      std::string s,
                      alliance_t alliance,
                      field_side_t side,
                      void (*callback)(lv_event_t*)) {
    lv_obj_t* btn = lv_button_create(holder);

    lv_obj_set_pos(btn, posX, posY); /*Set its position*/
    lv_obj_set_size(btn, width, height); /*Set its size*/
    lv_obj_add_event_cb(btn,
                        callback,
                        LV_EVENT_CLICKED,
                        NULL); /*Assign a callback to the button*/

    lv_obj_t* label = lv_label_create(btn); /*Add a label to the button*/
    lv_label_set_text(label, s.c_str()); /*Set the labels text*/
    lv_obj_center(label); /*Align the label to the center*/

    lv_color_t red_col = lv_palette_darken(LV_PALETTE_RED, 1);
    lv_color_t blue_col = lv_palette_darken(LV_PALETTE_BLUE,1);
    lv_color_t side_col = lv_palette_darken(LV_PALETTE_ORANGE,2);
    lv_color_t side_text_col = lv_color_black();

    static lv_style_t red_style, blue_style, side_style;
    lv_style_init(&red_style);
    lv_style_init(&blue_style);
    lv_style_init(&side_style);
    lv_style_set_bg_color(&red_style, red_col);
    lv_style_set_bg_color(&blue_style, blue_col);

    lv_style_set_bg_color(&side_style, side_col);
    lv_style_set_text_color(&side_style, side_text_col);

    if (side != field_side_t::unset) {
        lv_obj_add_style(btn, &side_style, 0);
    } else {
        if (alliance == alliance_t::blue) {
            lv_obj_add_style(btn, &blue_style, 0);
        } else if (alliance == alliance_t::red) {
            lv_obj_add_style(btn, &red_style, 0);
        }
    }

    // sets the font to be visible
    lv_obj_set_style_text_font(btn, &lv_font_montserrat_36, 0);
    return btn;
}

// responsible for updating selected auton every time a checkbox is set
static void auton_radio_event_handler(lv_event_t* e) {
    lv_obj_t* cont = (lv_obj_t*)lv_event_get_current_target(e);
    lv_obj_t* act_cb = (lv_obj_t*)lv_event_get_target(e);

    // Do nothing if the container was clicked
    if (act_cb == cont) return;

    // Uncheck the previous radio button
    if (last_selected_checkbox != -1) {
        lv_obj_t* old_cb = lv_obj_get_child(cont, last_selected_checkbox);
        lv_obj_remove_state(old_cb, LV_STATE_CHECKED);
    }
    lv_obj_add_state(act_cb,
                     LV_STATE_CHECKED); // Uncheck the current radio button

    last_selected_checkbox = lv_obj_get_index(act_cb);

    // update the auton mode to the correct one
    setAuton(radio_to_auton_mode[act_cb]);
}

// sets up the screen
void init() {
    // main screen
    auton_select_screen = lv_obj_create(lv_screen_active());

    // makes object take up the full screen and have no styling
    lv_obj_remove_style_all(auton_select_screen);
    lv_obj_set_size(auton_select_screen,
                    lv_display_get_horizontal_resolution(NULL),
                    lv_display_get_vertical_resolution(NULL));
    lv_obj_center(auton_select_screen);
    lv_obj_set_style_bg_opa(auton_select_screen, LV_OPA_COVER, 0);

    // background color of the screen
    lv_obj_set_style_bg_color(auton_select_screen,
                              lv_color_black(),
                              0);

    // hidden by default
    lv_obj_add_flag(auton_select_screen, LV_OBJ_FLAG_HIDDEN);

    // offset from top left of screen at which the buttons start
    int startX = 5;
    int startY = 5;

    // both buttons + padding should take 50% of the screen
    int corner_btn_width =
      // (lv_display_get_horizontal_resolution(NULL) * 0.5 - startX * 2) * 0.5;
      // (lv_display_get_horizontal_resolution(NULL) / 2 - startX * 2) / 2;
      (lv_display_get_horizontal_resolution(NULL) / 2) / 2 - startX;

    int corner_btn_height =
      // (lv_display_get_vertical_resolution(NULL) - startY * 2) / 2;
      lv_display_get_vertical_resolution(NULL) / 2 - startY;

    // [ blue, red, left, right ]

    // blue
    field_btns[0] = make_button(auton_select_screen,
                                startX,
                                startY,
                                corner_btn_width,
                                corner_btn_height,
                                "",
                                alliance_t::blue,
                                field_side_t::unset,
                                blue_cb);
    // red
    field_btns[1] = make_button(auton_select_screen,
                                startX + corner_btn_width,
                                startY,
                                corner_btn_width,
                                corner_btn_height,
                                "",
                                alliance_t::red,
                                field_side_t::unset,
                                red_cb);
    // left
    field_btns[2] = make_button(auton_select_screen,
                                startX,
                                startY + corner_btn_height,
                                corner_btn_width,
                                corner_btn_height,
                                LV_SYMBOL_LEFT,
                                alliance_t::unset,
                                field_side_t::left,
                                left_cb);
    // right
    field_btns[3] = make_button(auton_select_screen,
                                startX + corner_btn_width,
                                startY + corner_btn_height,
                                corner_btn_width,
                                corner_btn_height,
                                LV_SYMBOL_RIGHT,
                                alliance_t::unset,
                                field_side_t::right,
                                right_cb);

    static lv_style_t style_radio, style_radio_chk;

    lv_style_init(&style_radio);
    lv_style_init(&style_radio_chk);

    lv_style_set_radius(&style_radio, LV_RADIUS_CIRCLE);
    lv_style_set_bg_image_src(&style_radio_chk, NULL);

    // auton checkboxes
    lv_obj_t* checkbox_container = lv_obj_create(auton_select_screen);
    lv_obj_set_flex_flow(checkbox_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(checkbox_container, lv_pct(48), lv_pct(100));
    lv_obj_set_x(checkbox_container, lv_pct(51));
    lv_obj_set_y(checkbox_container, lv_pct(0));

    lv_obj_add_event_cb(checkbox_container,
                        auton_radio_event_handler,
                        LV_EVENT_CLICKED,
                        NULL);

    for (auto& pair : auton_list) {
        lv_obj_t* checkbox = lv_checkbox_create(checkbox_container);

        lv_checkbox_set_text(checkbox, pair.first.c_str());

        // make sure the events reach the parent
        lv_obj_add_flag(checkbox, LV_OBJ_FLAG_EVENT_BUBBLE);

        lv_obj_add_style(checkbox, &style_radio, LV_PART_INDICATOR);

        lv_obj_add_style(checkbox, &style_radio_chk, LV_PART_INDICATOR);
        lv_obj_add_style(checkbox, &style_radio_chk, LV_STATE_CHECKED);
        lv_obj_set_style_text_font(checkbox, &lv_font_montserrat_24, 0);

        radio_to_auton_mode[checkbox] = pair.first;
    }
}
} // namespace auton_select
} // namespace screen
