#include "apis.h"
//

#include "screen.h"
#include "autons_list.h"
#include "auton_globals.h"
#include <map>

namespace screen {
namespace auton_select {
lv_obj_t* screen;

// this is the screen that becomes active after this one is done
lv_obj_t* next_screen = nullptr;

std::map<lv_obj_t*, std::string> radio_to_auton_mode;

// auton screen objects
lv_obj_t* field_btns[4];

std::optional<lv_obj_t*> last_selected_checkbox = std::nullopt;

static lv_style_t selected_style, not_selected_style;
static lv_style_t red_style, blue_style, side_style;
static lv_style_t style_radio, style_radio_chk;
static lv_style_t button_container_style;
static lv_style_t no_round;

static lv_style_t corner_style_default;

// called once inside init, no need to make thread safe
void initStyles() {
    lv_color_t red_col = lv_palette_darken(LV_PALETTE_RED, 1);
    lv_color_t blue_col = lv_palette_darken(LV_PALETTE_BLUE, 1);
    lv_color_t side_col = lv_palette_darken(LV_PALETTE_ORANGE, 2);
    lv_color_t side_text_col = lv_color_black();

    lv_style_init(&red_style);
    lv_style_init(&blue_style);
    lv_style_init(&side_style);
    lv_style_set_bg_color(&red_style, red_col);
    lv_style_set_bg_color(&blue_style, blue_col);

    lv_style_set_bg_color(&side_style, side_col);
    lv_style_set_text_color(&side_style, side_text_col);

    lv_style_init(&style_radio);
    lv_style_init(&style_radio_chk);

    lv_style_set_radius(&style_radio, LV_RADIUS_CIRCLE);
    lv_style_set_bg_image_src(&style_radio_chk, NULL);

    lv_style_init(&no_round);
    lv_style_set_radius(&no_round, 0);

    lv_style_init(&button_container_style);

    lv_style_set_radius(&button_container_style, 0);
    lv_style_set_pad_gap(&button_container_style, 0);
    lv_style_set_pad_all(&button_container_style, 0);

    /*Properties to transition*/
    static lv_style_prop_t props[] = { LV_STYLE_TRANSFORM_WIDTH,
                                       LV_STYLE_TRANSFORM_HEIGHT,
                                       0 };

    /*Transition descriptor when going back to the default state.
     *Add some delay to be sure the press transition is visible even if the
     * press was very short*/
    static lv_style_transition_dsc_t transition_dsc_default;
    lv_style_transition_dsc_init(&transition_dsc_default,
                                 props,
                                 lv_anim_path_ease_in_out,
                                 250,
                                 50,
                                 NULL);

    /*Transition descriptor when going to pressed state.
     *No delay, go to presses state immediately*/
    static lv_style_transition_dsc_t transition_dsc_focus;
    lv_style_transition_dsc_init(&transition_dsc_focus,
                                 props,
                                 lv_anim_path_ease_in_out,
                                 250,
                                 0,
                                 NULL);

    /*Add only the new transition to he default state*/
    lv_style_init(&corner_style_default);
    lv_style_set_transition(&corner_style_default, &transition_dsc_default);

    /*Add the transition and some transformation to the presses state.*/

    lv_style_init(&selected_style);
    lv_style_set_bg_opa(&selected_style, LV_OPA_100);
    lv_style_set_transform_width(&selected_style, 1);
    lv_style_set_transform_height(&selected_style, 1);
    lv_style_set_transition(&selected_style, &transition_dsc_focus);

    lv_style_init(&not_selected_style);
    lv_style_set_bg_opa(&not_selected_style, LV_OPA_40);
    lv_style_set_transform_width(&not_selected_style, -1);
    lv_style_set_transform_height(&not_selected_style, -1);
    lv_style_set_transition(&not_selected_style, &transition_dsc_focus);
}

// only called from callbacks, also thread safe
void update_fields() {
    for (int i = 0; i < 4; i++) {
        lv_obj_add_state(field_btns[i], LV_STATE_USER_2);
    }

    auto activate_field = [](lv_obj_t* obj) {
        lv_obj_remove_state(obj, LV_STATE_USER_2);
        lv_obj_add_state(obj, LV_STATE_USER_1);
    };

    if (auto_alliance == alliance_t::blue) {
        activate_field(field_btns[0]);
    } else if (auto_alliance == alliance_t::red) {
        activate_field(field_btns[1]);
    }

    if (auto_side == field_side_t::left) {
        activate_field(field_btns[2]);
    } else if (auto_side == field_side_t::right) {
        activate_field(field_btns[3]);
    }
}

// callbacks are thread safe
void left_cb(lv_event_t* e) {
    setFieldSide(field_side_t::left);
}

void right_cb(lv_event_t* e) {
    setFieldSide(field_side_t::right);
}

void red_cb(lv_event_t* e) {
    setAlliance(alliance_t::red);
}

void blue_cb(lv_event_t* e) {
    setAlliance(alliance_t::blue);
}

// only called inside init, assumed to be thread-safe
lv_obj_t* make_btn(lv_obj_t* holder,
                   std::string s,
                   alliance_t alliance,
                   field_side_t side,
                   void (*callback)(lv_event_t*)) {
    lv_obj_t* btn = lv_button_create(holder);
    lv_obj_set_size(btn, lv_pct(45), lv_pct(45));
    lv_obj_add_event_cb(btn, callback, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(btn, &no_round, 0);

    lv_obj_add_style(btn, &selected_style, LV_STATE_USER_1);
    lv_obj_add_style(btn, &not_selected_style, LV_STATE_USER_2);
    lv_obj_add_style(btn, &corner_style_default, 0);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, s.c_str());
    lv_obj_center(label);

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
    if (side != field_side_t::unset) {
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_48, 0);
    } else {
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_30, 0);
    }

    return btn;
}

// responsible for updating selected auton every time a checkbox is set
// callback so its thread safe
static void auton_radio_event_handler(lv_event_t* e) {
    lv_obj_t* cont = (lv_obj_t*)lv_event_get_current_target(e);
    lv_obj_t* act_cb = (lv_obj_t*)lv_event_get_target(e);

    // Do nothing if the container was clicked
    if (act_cb == cont) return;

    // TODO: this is already done by setAuton?
    // check current button
    // lv_obj_add_state(act_cb, LV_STATE_CHECKED);
    //
    // // Uncheck the previous radio button
    // if (last_selected_checkbox.has_value()) {
    //     lv_obj_remove_state(last_selected_checkbox.value(),
    //     LV_STATE_CHECKED);
    // }
    //
    // last_selected_checkbox = act_cb;

    // update the auton mode to the correct one
    setAuton(radio_to_auton_mode[act_cb]);
}

void update_auton_radio_ui() {
    auto curr_auton = getAuton();

    lv_obj_t* new_selected = NULL;

    // search for c
    for (auto& it : radio_to_auton_mode) {
        if (it.second == curr_auton) {
            new_selected = it.first;
            break;
        }
    }

    if (new_selected != NULL) {
        // check current button
        lv_obj_add_state(new_selected, LV_STATE_CHECKED);

        // Uncheck the previous radio button
        if (last_selected_checkbox.has_value()) {
            lv_obj_remove_state(last_selected_checkbox.value(),
                                LV_STATE_CHECKED);
        }

        last_selected_checkbox = new_selected;
    }
}

// TODO: getting screen mutex is fine even if called from callback?
void ui_update() {
    get_screen_mutex();
    // update field buttons
    update_fields();

    // update radio buttons
    update_auton_radio_ui();

    give_screen_mutex();
}

// sets up the screen
void init(lv_obj_t* parent_screen) {
    get_screen_mutex();

    initStyles();

    // main screen
    screen = lv_obj_create(parent_screen);

    // makes object take up the full screen and have no styling
    lv_obj_remove_style_all(screen);
    lv_obj_set_size(screen, lv_pct(100), lv_pct(100));

    lv_obj_center(screen);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    // background color of the screen
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    // don't use elastic scroll
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // hidden by default
    lv_obj_add_flag(screen, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* button_container = lv_obj_create(screen);
    lv_obj_set_size(button_container, lv_pct(45), lv_pct(100));
    lv_obj_set_pos(button_container, lv_pct(0), lv_pct(0));

    lv_obj_set_flex_flow(button_container, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(button_container,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_SPACE_EVENLY);
    lv_obj_add_style(button_container, &button_container_style, 0);

    // button layout:
    // blue, red
    // left, right

    // blue
    field_btns[0] = make_btn(button_container,
                             "blue",
                             alliance_t::blue,
                             field_side_t::unset,
                             blue_cb);
    field_btns[1] = make_btn(button_container,
                             "red",
                             alliance_t::red,
                             field_side_t::unset,
                             red_cb);
    field_btns[2] = make_btn(button_container,
                             LV_SYMBOL_LEFT,
                             alliance_t::unset,
                             field_side_t::left,
                             left_cb);
    field_btns[3] = make_btn(button_container,
                             LV_SYMBOL_RIGHT,
                             alliance_t::unset,
                             field_side_t::right,
                             right_cb);

    // auton checkboxes
    lv_obj_t* checkbox_container = lv_obj_create(screen);
    lv_obj_set_flex_flow(checkbox_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(checkbox_container, lv_pct(75), lv_pct(100));
    lv_obj_set_pos(checkbox_container, lv_pct(45), lv_pct(0));
    lv_obj_add_style(checkbox_container, &no_round, 0);

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
        lv_obj_set_style_text_font(checkbox, &lv_font_montserrat_20, 0);

        radio_to_auton_mode[checkbox] = pair.first;
    }

    give_screen_mutex();
}
} // namespace auton_select
} // namespace screen
