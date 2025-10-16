#include "apis.h"
//

#include "liblvgl/lvgl.h"
#include "screen.h"

namespace screen {
namespace health {
lv_obj_t* screen;

static lv_obj_t* notification_lists[2];
static lv_obj_t* console_textarea;

// used for each tab object
static lv_style_t zero_padding;
static lv_style_t shared_notif_style;
static lv_style_t small_shared_notif_style;
static lv_style_t warn_style, critical_style, succeed_style;
static lv_style_t no_round;
static lv_style_t notification_list_style;

void initStyles() {
    lv_style_init(&shared_notif_style);
    lv_style_set_pad_row(&shared_notif_style, 5);
    lv_style_set_pad_all(&shared_notif_style, 10);

    lv_style_init(&small_shared_notif_style);
    lv_style_set_pad_row(&small_shared_notif_style, 0);
    lv_style_set_pad_all(&small_shared_notif_style, 5);

    lv_style_init(&critical_style);
    lv_style_set_bg_color(&critical_style,
                          lv_palette_lighten(LV_PALETTE_RED, 1));

    lv_style_init(&warn_style);
    lv_style_set_bg_color(&warn_style, lv_palette_darken(LV_PALETTE_ORANGE, 2));

    lv_style_init(&succeed_style);
    lv_style_set_bg_color(&succeed_style,
                          lv_palette_darken(LV_PALETTE_GREEN, 1));

    // zero padding style
    lv_style_init(&zero_padding);
    lv_style_set_pad_all(&zero_padding, 0);

    lv_style_init(&no_round);
    lv_style_set_radius(&no_round, 0);

    lv_style_init(&notification_list_style);
    lv_style_set_radius(&notification_list_style, 0);
    lv_style_set_pad_row(&notification_list_style, 3);
    lv_style_set_pad_all(&notification_list_style, 3);
}

void add_notification(std::string title_text,
                      std::string detail_text,
                      notification_severity_t severity) {
    // done as a hacky way to get thread safety with liblvgl
    get_screen_mutex();

    lv_obj_t* parent = notification_lists[1];
    lv_obj_t* notif;
    notif = lv_obj_create(parent);
    lv_obj_set_size(notif, lv_pct(100), lv_pct(40));
    lv_obj_set_flex_flow(notif, LV_FLEX_FLOW_COLUMN);
    // make it not scrollable - makes it annoying to use on screen
    lv_obj_remove_flag(notif, LV_OBJ_FLAG_SCROLLABLE);

    // add styles
    lv_obj_add_style(notif, &shared_notif_style, 0);

    if (severity == notification_severity_t::critical) {
        lv_obj_add_style(notif, &critical_style, 0);
    } else if (severity == notification_severity_t::warn) {
        lv_obj_add_style(notif, &warn_style, 0);
    } else if (severity == notification_severity_t::succeed) {
        lv_obj_add_style(notif, &succeed_style, 0);
    }

    lv_obj_t* title = lv_label_create(notif);
    lv_label_set_text(title, title_text.c_str());
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);

    lv_obj_t* detail = lv_label_create(notif);
    lv_label_set_text(detail, detail_text.c_str());

    lv_obj_scroll_to_view(notif, LV_ANIM_ON);
}

// returns the index of the notification
int add_init_notif(std::string title_text, notification_severity_t severity) {
    // done as a hacky way to get thread safety with liblvgl
    get_screen_mutex();

    lv_obj_t* parent = notification_lists[0];
    lv_obj_t* notif;
    notif = lv_obj_create(parent);

    lv_obj_set_size(notif, lv_pct(100), lv_pct(15));
    lv_obj_set_flex_flow(notif, LV_FLEX_FLOW_COLUMN);

    // add styles
    lv_obj_add_style(notif, &small_shared_notif_style, 0);

    if (severity == notification_severity_t::critical) {
        lv_obj_add_style(notif, &critical_style, 0);
    } else if (severity == notification_severity_t::warn) {
        lv_obj_add_style(notif, &warn_style, 0);
    } else if (severity == notification_severity_t::succeed) {
        lv_obj_add_style(notif, &succeed_style, 0);
    }

    lv_obj_t* title = lv_label_create(notif);
    lv_label_set_text(title, title_text.c_str());

    lv_obj_scroll_to_view(notif, LV_ANIM_ON);

    return lv_obj_get_index(notif);
}

void update_init_notif_severity(int index,
                                notification_severity_t new_severity) {
    get_screen_mutex();

    if (index < 0) {
        printf("invalid notif severity index!\n");
        return;
    }
    lv_obj_t* child = lv_obj_get_child(notification_lists[0], index);
    if (child == NULL) {
        printf("notif severity: child is null!\n");
        return;
    }
    if (new_severity == notification_severity_t::critical) {
        lv_obj_add_style(child, &critical_style, 0);
    } else if (new_severity == notification_severity_t::warn) {
        lv_obj_add_style(child, &warn_style, 0);
    } else if (new_severity == notification_severity_t::succeed) {
        lv_obj_add_style(child, &succeed_style, 0);
    }
}

void init_notification_list(lv_obj_t* parent_screen, int notification_index) {
    notification_lists[notification_index] = lv_list_create(parent_screen);
    // set to be 70% screen
    lv_obj_set_size(notification_lists[notification_index],
                    lv_pct(100),
                    lv_pct(100));

    lv_obj_add_style(notification_lists[notification_index],
                     &notification_list_style,
                     0);
}

void console_screen(lv_obj_t* parent_obj) {
    console_textarea = lv_textarea_create(parent_obj);
    lv_obj_add_style(console_textarea, &no_round, 0);
    lv_obj_set_size(console_textarea, lv_pct(100), lv_pct(100));
}

void set_console_text(std::string text) {
    // done as a hacky way to get thread safety with liblvgl
    get_screen_mutex();

    lv_textarea_set_text(console_textarea, text.c_str());
}

void console_println(std::string text) {
    get_screen_mutex();
    text += "\n";
    lv_textarea_add_text(console_textarea, text.c_str());
}

void init(lv_obj_t* error_parent_screen,
          lv_obj_t* status_parent_screen,
          lv_obj_t* console_parent_screen) {
	get_screen_mutex();
    initStyles();

    init_notification_list(error_parent_screen, 0);
    init_notification_list(status_parent_screen, 1);
    console_screen(console_parent_screen);
}

} // namespace health
} // namespace screen
