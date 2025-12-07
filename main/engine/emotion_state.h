#ifndef EMOTION_STATE_H
#define EMOTION_STATE_H

typedef enum {
    HAPPY,
    SAD,
    FEARFUL,
    ANGRY,
    DISGUSTED,
    SURPRISED
} emotion_t;

typedef struct emotion_state_t {
    float happiness;
    float sadness;
    float fear;
    float anger;
    float disgust;
    float surprise;
} emotion_state_t;

void init_emotion_state(emotion_state_t *state) {
    state->happiness = 10.0f;
    state->sadness = 10.0f;
    state->fear = 10.0f;
    state->anger = 10.0f;
    state->disgust = 10.0f;
    state->surprise = 10.0f;
}

float get_emotion_level(const emotion_state_t *state, emotion_t emotion) {
    switch (emotion) {
        case HAPPY:
            return state->happiness;
        case SAD:
            return state->sadness;
        case FEARFUL:
            return state->fear;
        case ANGRY:
            return state->anger;
        case DISGUSTED:
            return state->disgust;
        case SURPRISED:
            return state->surprise;
        default:
            return -1.0f;
    }
}

#endif