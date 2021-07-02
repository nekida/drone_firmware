#include "scheduler.h"
#include <stdio.h>

void task_print_1 (uint64_t current_time_in_us)
{
    UNUSED(current_time_in_us);

    printf("This is task the PRINT 1\n");
}

void task_print_2 (uint64_t current_time_in_us)
{
    UNUSED(current_time_in_us);

    printf("This is task the PRINT 2\n");
}

void task_print_3 (uint64_t current_time_in_us)
{
    UNUSED(current_time_in_us);

    printf("This is task the PRINT 3\n");
}

task_st tasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .task_name = "system",
        .task_func = task_system,
        .desired_period = TASK_PERIOD_MS(1000),
        .static_priority = TASK_PRIORITY_MIDDLE,
    },

    [TASK_PRINT_1] = {
        .task_name = "print 1",
        .task_func = task_print_1,
        .desired_period = TASK_PERIOD_MS(1000),
        .static_priority = TASK_PRIORITY_MIDDLE,
    },

    [TASK_PRINT_2] = {
        .task_name = "print 2",
        .task_func = task_print_2,
        .desired_period = TASK_PERIOD_MS(1000),
        .static_priority = TASK_PRIORITY_MIDDLE,
    },

    [TASK_PRINT_3] = {
        .task_name = "print 3",
        .task_func = task_print_3,
        .desired_period = TASK_PERIOD_MS(1),
        .static_priority = TASK_PRIORITY_MIDDLE,
    }
};