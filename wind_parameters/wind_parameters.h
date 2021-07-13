#ifndef _WIND_PARAMETERS_H
#define _WIND_PARAMETERS_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    MOTOR_1 = 1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4
} MOTORS_NUM_et;

typedef struct {
    void *motor_1;
    void *motor_2;
    void *motor_3;
    void *motor_4;
} speed_motors_st;

void *get_motor_speed (const MOTORS_NUM_et num_motor);
void set_motor_speed (void *speed, const MOTORS_NUM_et num_motor);

#endif // _WIND_PARAMETERS_H