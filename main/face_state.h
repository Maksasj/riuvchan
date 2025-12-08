#ifndef FACE_STATE_H
#define FACE_STATE_H

#include "easings.h"

typedef struct face_state_t {
    uint8_t left_eye_x_position;
    uint8_t left_eye_y_position;
    uint8_t left_eye_width;
    uint8_t left_eye_height; 

    uint8_t right_eye_x_position;
    uint8_t right_eye_y_position;
    uint8_t right_eye_width;
    uint8_t right_eye_height; 

    uint8_t mouth_angle_start;
    uint8_t mouth_angle_end;
    uint8_t mouth_angle_x_position;
    uint8_t mouth_angle_y_position;
    uint8_t mouth_width;

    uint8_t decoration_width;
    uint8_t decoration_left_x_position;
    uint8_t decoration_left_y_position;

    uint8_t decoration_right_x_position;
    uint8_t decoration_right_y_position;
} face_state_t;

face_state_t calculate_face_state(face_state_t* current_face_state, face_state_t* previous_face_state, float transition);

face_state_t* create_face_state(face_state_t state) {
    face_state_t* new_state = malloc(sizeof(face_state_t));
    *new_state = state;
    return new_state;
}

face_state_t calculate_face_state(face_state_t* current_face_state, face_state_t* previous_face_state, float transition) {
    if(previous_face_state == NULL)
        return *current_face_state;
        
    face_state_t face_state;

    face_state.left_eye_x_position = ease_in_out_cubic(previous_face_state->left_eye_x_position, current_face_state->left_eye_x_position, transition);
    face_state.left_eye_y_position = ease_in_out_cubic(previous_face_state->left_eye_y_position, current_face_state->left_eye_y_position, transition);
    face_state.left_eye_width = ease_in_out_cubic(previous_face_state->left_eye_width, current_face_state->left_eye_width, transition);
    face_state.left_eye_height = ease_in_out_cubic(previous_face_state->left_eye_height, current_face_state->left_eye_height, transition); 

    face_state.right_eye_x_position = ease_in_out_cubic(previous_face_state->right_eye_x_position, current_face_state->right_eye_x_position, transition);
    face_state.right_eye_y_position = ease_in_out_cubic(previous_face_state->right_eye_y_position, current_face_state->right_eye_y_position, transition);
    face_state.right_eye_width = ease_in_out_cubic(previous_face_state->right_eye_width, current_face_state->right_eye_width, transition);
    face_state.right_eye_height = ease_in_out_cubic(previous_face_state->right_eye_height, current_face_state->right_eye_height, transition);

    face_state.mouth_angle_start = ease_in_out_cubic(previous_face_state->mouth_angle_start, current_face_state->mouth_angle_start, transition);
    face_state.mouth_angle_end = ease_in_out_cubic(previous_face_state->mouth_angle_end, current_face_state->mouth_angle_end, transition);
    face_state.mouth_angle_x_position = ease_in_out_cubic(previous_face_state->mouth_angle_x_position, current_face_state->mouth_angle_x_position, transition);
    face_state.mouth_angle_y_position = ease_in_out_cubic(previous_face_state->mouth_angle_y_position, current_face_state->mouth_angle_y_position, transition);
    face_state.mouth_width = ease_in_out_cubic(previous_face_state->mouth_width, current_face_state->mouth_width, transition);


    face_state.decoration_width = ease_in_out_cubic(previous_face_state->decoration_width, current_face_state->decoration_width, transition);
    face_state.decoration_left_x_position = ease_in_out_cubic(previous_face_state->decoration_left_x_position, current_face_state->decoration_left_x_position, transition);
    face_state.decoration_left_y_position = ease_in_out_cubic(previous_face_state->decoration_left_y_position, current_face_state->decoration_left_y_position, transition);

    face_state.decoration_right_x_position = ease_in_out_cubic(previous_face_state->decoration_right_x_position, current_face_state->decoration_right_x_position, transition);
    face_state.decoration_right_y_position = ease_in_out_cubic(previous_face_state->decoration_right_y_position, current_face_state->decoration_right_y_position, transition);

    return face_state;
}

#endif