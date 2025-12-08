#ifndef STATES_H
#define STATES_H

#include "engine/state_machine.h"

#include "components/ssd1306.h"

#include "face_state.h"
#include "shared.h"

typedef enum {
    TOUCH_TAP,
    TOUCH_DOUBLE_TAP,
    TOUCH_MULTI_TAP,
    TOUCH_LONG_TAP,
    TOUCH_SUPER_LONG_TAP,

    THROWN_UP,
} event_type_t;

typedef struct {
    event_type_t type;

    union {
        int multi_tap;
    } as;
} event_t;

event_t* create_touch_event(event_type_t type, int multi_tap) {
    event_t* event = malloc(sizeof(event_t));
    event->type = type;
    event->as.multi_tap = multi_tap;
    return event;
}

event_t* create_throw_up_event() {
    event_t* event = malloc(sizeof(event_t));
    event->type = THROWN_UP;
    return event;
}

void add_all_states(state_machine_t* state_machine) {
    add_state(state_machine, create_state("IDLE_STRAIGHT_EYES_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 5,
        .left_eye_width = 25,
        .left_eye_height = 40,

        .right_eye_x_position = SSD1306_WIDTH - 10 - 25,
        .right_eye_y_position = 5,
        .right_eye_width = 25,
        .right_eye_height = 40,

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = 64,
        .mouth_angle_y_position = 32,
        .mouth_width = 6,

        .decoration_width = 15,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 55,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 10,
        .decoration_right_y_position = 55
    })));

    add_state(state_machine, create_state("IDLE_CLOSED_EYES_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 25,
        .left_eye_width = 25,
        .left_eye_height = 3,

        .right_eye_x_position = 93, 
        .right_eye_y_position = 25,
        .right_eye_width = 25,
        .right_eye_height = 3,

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = 64,
        .mouth_angle_y_position = 32,
        .mouth_width = 6,

        .decoration_width = 15,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 55,

        .decoration_right_x_position = 108,
        .decoration_right_y_position = 55
    })));

    add_state(state_machine, create_state("IDLE_SLEEP_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 40,
        .left_eye_width = 30,
        .left_eye_height = 5,

        .right_eye_x_position = SSD1306_WIDTH - 10 - 25,
        .right_eye_y_position = 40,
        .right_eye_width = 30,
        .right_eye_height = 5,

        .mouth_angle_start = 80,
        .mouth_angle_end = 100,
        .mouth_angle_x_position = 64,
        .mouth_angle_y_position = 32,
        .mouth_width = 6,

        .decoration_width = 15,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 55,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 5,
        .decoration_right_y_position = 55
    })));

    add_state(state_machine, create_state("CONFUSED_CLOSED_EYES_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 25,
        .left_eye_width = 25,
        .left_eye_height = 3,

        .right_eye_x_position = 93, 
        .right_eye_y_position = 25,
        .right_eye_width = 25,
        .right_eye_height = 3,

        .mouth_angle_start = 90,
        .mouth_angle_end = 90,
        .mouth_angle_x_position = 64,
        .mouth_angle_y_position = 32,
        .mouth_width = 6,

        .decoration_width = 15,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 52,

        .decoration_right_x_position = 108,
        .decoration_right_y_position = 52
    })));

    /*
    add_state(state_machine, create_state("IDLE_EYES_RIGHT_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 5,
        .left_eye_width = 25,
        .left_eye_height = 40, 

        .right_eye_x_position = 10 + 25 + 5,
        .right_eye_y_position = 5,
        .right_eye_width = 25,
        .right_eye_height = 40, 

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = 10 + 25 + 2,
        .mouth_angle_y_position = 32,
        .mouth_width = 5,

        .decoration_width = 9,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 49,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 10,
        .decoration_right_y_position = 49
    })));

    add_state(state_machine, create_state("IDLE_EYES_LEFT_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = SSD1306_WIDTH - 10 - 25 - 5 - 25,
        .left_eye_y_position = 5,
        .left_eye_width = 25,
        .left_eye_height = 40, 

        .right_eye_x_position = SSD1306_WIDTH - 10 - 25,
        .right_eye_y_position = 5,
        .right_eye_width = 25,
        .right_eye_height = 40, 

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = SSD1306_WIDTH - 10 - 25 - 2,
        .mouth_angle_y_position = 32,
        .mouth_width = 2,

        .decoration_width = 9,
        .decoration_left_x_position = SSD1306_WIDTH - 10 - 8 - 45,
        .decoration_left_y_position = 47,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 8,
        .decoration_right_y_position = 47
    })));
    */
} 

#endif
