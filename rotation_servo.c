#include <rotation_servo.h>
#include "stm32f4xx_hal.h"
#include <math.h>

#define period_max_cw     500.0f     // 0.5ms
#define period_stop       1500.0f    // 1.5ms
#define period_max_ccw    2500.0f    // 2.5ms
#define period            20000.0f   // 20ms (for the 50 Hz used frequency)
#define pi                3.1415926535f

#define wheel             3.2f       // wheel radius in cm
#define distance_wheels   20.0f      // distance between two wheels in cm

TIM_HandleTypeDef *htim;
uint32_t servo;


void servo_init(TIM_HandleTypeDef *htim_local, uint32_t channel)
{
    HAL_TIM_PWM_Start(htim_local, channel);
    htim = htim_local;
}

float servo_pulse(float distance, float time)
{
    // compute needed rotations
    float circumference = 2.0f * pi * wheel;
    float rotations = distance / circumference;

    // compute rpm
    float rpm = (rotations / time) * 60.0f; // rotations per minute (60s)
    float max_rpm = 60.0f; // 1 rotation per second is the max rpm
    float pulse = period_stop + (rpm / max_rpm) * 1000.0f; // rpm to period ( 2500-1500 = 1000 for ccw / 1500-500=1000 for cw )

    // fix calculations errors if pulse values isn't within 500-2500 band
    if (pulse < period_max_cw) pulse = period_max_cw ;
    if (pulse > period_max_ccw) pulse = period_max_ccw ;

    return pulse;
}


float servo_accel(float target,float current_pwm)
{
    // Compute remaining distance
    float diff = target - current_pwm;

    // Accelerate only if target is above current power
    if (diff > 0.0f)
    {
        // Move 5% of remaining distance
        float step = diff * 0.05f;
        current_pwm += step;
        // Avoid overshoot
        if (current_pwm > target)
            current_pwm = target;
    }
    // Also handle deceleration (if target is below current power)
    else if (diff < 0.0f)
    {
        // Move 5% of remaining distance (diff is negative)
        float step = diff * 0.05f;
        current_pwm += step;
        // Avoid overshoot
        if (current_pwm < target)
            current_pwm = target;
    }

    // Ensure target is inside servo limits
    if (current_pwm < period_max_cw)  current_pwm = period_max_cw;
    if (current_pwm > period_max_ccw) current_pwm = period_max_ccw;

    return current_pwm ;
}


void servo_out(uint32_t channel, float target_pwm, float travel_time_s)
{
    float current_pwm = period_stop;             // start from stop
    float elapsed = 0.0f;
    float travel_time_with_margin = travel_time_s * 1.1f; // +10% margin

    while (elapsed < travel_time_with_margin)
    {
        // we accel from 0 speed to desired
        if (fabsf(current_pwm - target_pwm) > 5.0f)
        {
            current_pwm = servo_accel(target_pwm, current_pwm);
        }
        else
        {
            current_pwm = target_pwm;  // already at target, maintain constant speed
        }

        // set output PWM to servo
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);             // get timer ARR
        float duty = current_pwm / period;                            // duty cycle fraction
        uint32_t ccr = (uint32_t)(duty * arr);                    // compute CCR value

        __HAL_TIM_SET_COMPARE(htim, channel, ccr);                // update PWM

        HAL_Delay((uint32_t)(20.0f));  // wait 20ms
        elapsed += 0.02f ;
    }

    // Stop after total travel time
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);             // get timer ARR
    float duty = period_stop / period;                            // duty cycle fraction
    uint32_t ccr = (uint32_t)(duty * arr);                    // compute CCR value

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);                // update PWM
}




