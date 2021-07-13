#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "wind_parameters.h"

int main (int arg_c, char *arg_v[])
{
    if (arg_c == 1)
        return -1;

    int intgr_1, intgr_2, intgr_3, intgr_4;
    float flt_1, flt_2, flt_3, flt_4;

    switch (*arg_v[5]) {
        case 'i':   // int
            intgr_1 = atoi(arg_v[1]);
            set_motor_speed((void *)&intgr_1, MOTOR_1);
            intgr_2 = atoi(arg_v[2]);
            set_motor_speed((void *)&intgr_2, MOTOR_2);
            intgr_3 = atoi(arg_v[3]);
            set_motor_speed((void *)&intgr_3, MOTOR_3);
            intgr_4 = atoi(arg_v[4]);
            set_motor_speed((void *)&intgr_4, MOTOR_4);

            printf("%d %d %d %d\n", *(int *)get_motor_speed(MOTOR_1), *(int *)get_motor_speed(MOTOR_2), *(int *)get_motor_speed(MOTOR_3), *(int *)get_motor_speed(MOTOR_4));
            break;

        case 'f':   // float
            flt_1 = atof(arg_v[1]);
            set_motor_speed((void *)&flt_1, MOTOR_1);
            flt_2 = atof(arg_v[2]);
            set_motor_speed((void *)&flt_2, MOTOR_2);
            flt_3 = atof(arg_v[3]);
            set_motor_speed((void *)&flt_3, MOTOR_3);
            flt_4 = atof(arg_v[4]);
            set_motor_speed((void *)&flt_4, MOTOR_4);

            printf("%.1f %.1f %.1f %.1f\n", *(float *)get_motor_speed(MOTOR_1), *(float *)get_motor_speed(MOTOR_2), *(float *)get_motor_speed(MOTOR_3), *(float *)get_motor_speed(MOTOR_4));
            break;
    }
    
    return 0;
}


