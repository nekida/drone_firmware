#include "wind_parameters.h"

static uint32_t dummy_val = 0;
static speed_motors_st speed_motors;

void *get_motor_speed (MOTORS_NUM_et num_motor)
{
    void *speed_motor = NULL;

    switch (num_motor) {
        case 1: speed_motor = speed_motors.motor_1; break;

        case 2: speed_motor = speed_motors.motor_2; break;

        case 3: speed_motor = speed_motors.motor_3; break;

        case 4: speed_motor = speed_motors.motor_4; break;

        default: return &dummy_val;
    }

    return speed_motor;
}

void set_motor_speed (void *speed, const MOTORS_NUM_et num_motor)
{
    switch (num_motor) {
        case 1: speed_motors.motor_1 = speed;    break;
        
        case 2: speed_motors.motor_2 = speed;    break;

        case 3: speed_motors.motor_3 = speed;    break;

        case 4: speed_motors.motor_4 = speed;    break;
    }
}