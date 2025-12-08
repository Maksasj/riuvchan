#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <string.h>
#include <math.h>

#include "emotion_state.h"

#define STATE_MACHINE_MAX_STATES 16

typedef struct state_t {
    char* state_name;
    
    void* user_data;
} state_t;

state_t* create_state_empty(int id, const char* name) {
    state_t *state = malloc(sizeof(state_t));

    state->state_name = strdup(name);
    state->user_data = NULL;

    return state;
}

state_t* create_state(const char* name, void* user_data) {
    state_t *state = malloc(sizeof(state_t));

    state->state_name = strdup(name);
    state->user_data = user_data;

    return state;
}

typedef struct state_machine_t {
    int current_state_index;
    int previous_state_index;
    float transition;

    int stored;
    state_t** states;
} state_machine_t;

void create_state_machine(state_machine_t* state_machine) {
    state_machine->current_state_index = -1;
    state_machine->previous_state_index = -1;
    state_machine->transition = 0.0f;

    state_machine->stored = 0;
    state_machine->states = calloc(STATE_MACHINE_MAX_STATES, sizeof(state_t*));
    memset(state_machine->states, 0, STATE_MACHINE_MAX_STATES*sizeof(state_t*));
}

void add_state(state_machine_t* state_machine, state_t* state) {
    if(state_machine->stored >= STATE_MACHINE_MAX_STATES)
        assert(false && "Exceeded max states in state machine");

    state_machine->states[state_machine->stored] = state;
    state_machine->stored++;
}

state_t* get_current_state(state_machine_t* state_machine) {
    if(state_machine->current_state_index < 0 || state_machine->current_state_index >= state_machine->stored)
        return NULL;

    return state_machine->states[state_machine->current_state_index];
}

state_t* get_previous_state(state_machine_t* state_machine) {
    if(state_machine->previous_state_index < 0 || state_machine->previous_state_index >= state_machine->stored)
        return NULL;

    return state_machine->states[state_machine->previous_state_index];
}

void update_state_machine(state_machine_t* state_machine, emotion_state_t* emotion_state) {
    emotion_state->happiness = fmin(fmax(emotion_state->happiness, 0.0f), 100.0f);
    emotion_state->sadness = fmin(fmax(emotion_state->sadness, 0.0f), 100.0f);
    emotion_state->fear = fmin(fmax(emotion_state->fear, 0.0f), 100.0f);
    emotion_state->anger = fmin(fmax(emotion_state->anger, 0.0f), 100.0f);
    emotion_state->disgust = fmin(fmax(emotion_state->disgust, 0.0f), 100.0f);
    emotion_state->surprise = fmin(fmax(emotion_state->surprise, 0.0f), 100.0f);
}

void free_state_machine(state_machine_t* state_machine) {
    free(state_machine->states);
}

void free_state_machine_states(state_machine_t* state_machine) {
    for(int i = 0; i < STATE_MACHINE_MAX_STATES; ++i) {
        state_t* state = state_machine->states[i];

        if(state != NULL)
            free(state_machine->states[i]);
    }
}

#endif