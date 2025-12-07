#ifndef EASINGS_H
#define EASINGS_H   

int ease_in_out_cubic(int start, int finish, float t) {
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }

    int delta = finish - start;

    float eased_t;

    if (t < 0.5f) {
        eased_t = 4.0f * t * t * t;
    } else {
        float u = 2.0f * t - 2.0f;
        eased_t = 0.5f * (u * u * u + 2.0f);
    }

    return start + (int)(delta * eased_t);
}

#endif