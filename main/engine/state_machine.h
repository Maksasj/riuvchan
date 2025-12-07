#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef struct state_t {
    int id;
} state_t;

typedef struct state_machine_t {
    int current_state;

    state_t* states;
} state_machine_t;

void init_state_machine(state_machine_t* sm, state_t* states, int initial_state) {
    sm->states = states;
    sm->current_state = initial_state;
}

#endif