#include "Encoders.h"

#include "HAL.h"

void encoders_init() {
    hal_encoders_init();
    hal_encoders_zero();
}

int32_t encoder_left_counts() {
    return ENC_LEFT_SIGN * hal_encoder_left();
}

int32_t encoder_right_counts() {
    return ENC_RIGHT_SIGN * hal_encoder_right();
}

void encoders_zero() {
    hal_encoders_zero();
}
