#ifndef PASO_H
#define PASO_H

#include "stm32f4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t delay;
    GPIO_TypeDef *dir_port;
    uint32_t dir_pin;
    GPIO_TypeDef *step_port;
    uint32_t step_pin;
    float speed_rad_s;
    uint32_t steps_left;
    uint32_t step_state;
    GPIO_PinState dir;

    // temporary variables for isr for each stepper
    uint32_t accel_steps;
    uint32_t start_delay;
    uint32_t target_delay;
    uint32_t init_steps;

    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
} stepper;

typedef struct {
    float wheel_radius; //m
    float wheel_base;   //m
    float distance;     //m
    stepper *left;
    stepper *right;
} robot;

void computeNewSpeed(stepper *s);
uint32_t speed_to_delay_us(float speed_rad_s);
uint32_t distance_to_steps(float distance, float wheel_radius);
void move_stepper(stepper *s, uint32_t steps, GPIO_PinState direction);
void isr_stepper_all(stepper *s1, stepper *s2);
void isr_stepper(stepper *s);
void move_two_steppers(stepper *s1, stepper *s2, uint32_t steps, GPIO_PinState direction);
void robot_rotate(robot *r, float angle_deg, uint32_t cw); // 1 = CW, 0 = CCW
void sensor_control_loop(stepper *s);
void isr_ultrasonic(TIM_HandleTypeDef *htim);

/* --- Trajectory definitions --- */
typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_TURN_CW,
    MOVE_TURN_CCW
} move_type_t;

typedef struct {
    move_type_t type;
    float value;   // meters for FORWARD/BACKWARD, degrees for TURN_*
} robot_move_t;

typedef struct {
    const robot_move_t *moves;
    uint8_t num_moves;
} trajectory_t;

#define NUM_TRAJECTORIES 4
extern const trajectory_t trajectories[NUM_TRAJECTORIES];

void robot_execute_trajectory(robot *r, const trajectory_t *traj);

#endif
