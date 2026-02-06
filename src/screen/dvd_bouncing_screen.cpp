#include "apis.h"
//

#include "screen.h"

namespace screen {
namespace bouncing_dvd_screen {
lv_obj_t* screen;

lv_obj_t* bouncing_object;
lv_obj_t* dvd_image;

int prev_time = 0;
float x = 40;
float y = 40;
// px / sec
float vx = -3; // -2
float vy = -3; // -2

// container bounds
float cx = 0;
float cy = 0;
// bot w / h
float w = 0;
float h = 0;

// callback, thread safe
void rectangle_update(void* obj, int t) {
    int time = t;
    float dt = time - prev_time;
    // here the animation repeated, making
    if (dt < 0) {
        dt = 0;
        prev_time = time;
    }
    // make dt from milliseconds to seconds
    dt /= 1000;
    prev_time = time;
    // calulate the change in x and y, check for bounds, update vel if neccesary
    // v (pix/sec) * dt (sec) = d (pix)
    float dx = vx * dt;
    float dy = vy * dt;
    float nx = x + dx;
    float ny = y + dy;

    // default postioning is top left
    // meaning:
    // top:    check y coord
    // bottom: check y+h coord
    // left:   check x coord
    // right:  check x+w coord

    bool left = nx < 0 || nx > cx;
    bool right = nx + w < 0 || nx + w > cx;
    bool top = ny < 0 || ny > cy;
    bool bottom = ny + h < 0 || ny + h > cy;

    // int nvx = vx, nvy = vy;

    // if both or none intersect dont change anything
    if (left ^ right) {
        if (left) vx = fabs(vx);
        if (right) vx = -fabs(vx);
    }

    if (top ^ bottom) {
        if (top) vy = fabs(vy);
        if (bottom) vy = -fabs(vy);
    }

    x = nx;
    y = ny;
    lv_obj_set_x((lv_obj_t*)obj, x);
    lv_obj_set_y((lv_obj_t*)obj, y);
}

void init(lv_obj_t* parent_screen) {
    get_screen_mutex();

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

    // bounds of the screen
    cx = lv_display_get_horizontal_resolution(NULL);
    cy = lv_display_get_vertical_resolution(NULL);

    // width and height of image
    w = 194.0;
    h = 152.0;

    lv_obj_add_flag(screen, LV_OBJ_FLAG_HIDDEN);

    // rectangle that gets animated
    bouncing_object = lv_obj_create(screen);
    lv_obj_remove_style_all(bouncing_object);
    lv_obj_set_size(bouncing_object, w, h);

    lv_obj_set_style_bg_opa(bouncing_object, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bouncing_object,
                              lv_palette_darken(LV_PALETTE_BLUE, 3),
                              0);

    dvd_image = lv_image_create(bouncing_object);
    lv_image_set_src(dvd_image, &monkey_75x_img);

    lv_obj_align(dvd_image, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_anim_t bounce;
    lv_anim_init(&bounce);

    lv_anim_set_var(&bounce, bouncing_object);
    lv_anim_set_exec_cb(&bounce, (lv_anim_exec_xcb_t)rectangle_update);
    // in seconds
    int duration = 10;
    int updates_per_second = 24;
    // value = duration (sec) * 24 (updates / sec)
    int value = duration * updates_per_second;

    lv_anim_set_duration(&bounce, duration * 1000);
    lv_anim_set_values(&bounce, 0, value * 1000);
    lv_anim_set_repeat_count(&bounce, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&bounce);

    give_screen_mutex();
}
} // namespace bouncing_dvd_screen
} // namespace screen
