#include "paso.h"


/*
 * ------------------
 * paso.c guide :
 * ------------------
 *
 * 1) Configure a microsecond-resolution timer and enable its interrupt.
 * 2) Initialize the stepper strut (htim, DIR pin, STEP pin, speed_rad_s...etc)
 * 3) Call move_stepper() or move_two_steppers() to start motion.
 * 4) Call isr_stepper() for 1 stepper usage or isr_stepper_all() for 2 stepper coordinated motion from
 * HAL_TIM_PeriodElapsedCallback().
 */

// array used on the 2 stepper for a single timer
stepper *active_stpr[2];
uint8_t nbr_active_stpr = 0;


// Calculation functions

uint32_t speed_to_delay_us(float speed_rad_s)
{
    if (speed_rad_s <= 0) return 0;

    float steps_per_sec = (speed_rad_s * 200) / (2.0f * 3.1415926f);
    return (uint32_t)(1000000.0f / steps_per_sec);
}

uint32_t distance_to_steps(float distance,float wheel_radius)
{
    float circumference = 2.0f * 3.1415926f * wheel_radius;
    float revolutions   = distance / circumference;
    return (uint32_t)(revolutions * 200);

}


// moving functions

void move_stepper(stepper *s, uint32_t steps, GPIO_PinState direction)
{
    if (steps <= 0) return;

    s->steps_left = steps;
    s->dir = direction;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin, direction);

    s->accel_steps = 30;       // nbr of steps needed to accel

    if (s->accel_steps > steps) s->accel_steps = steps;

    float start_speed = 0.1f;            // intial speed rad/s
    s->start_delay  = speed_to_delay_us(start_speed);
    s->target_delay = speed_to_delay_us(s->speed_rad_s);
    s->init_steps   = steps;

    s->delay       = s->start_delay;
    s->step_state  = 0;

    // Add stepper to active list
    if (nbr_active_stpr < 2)
        active_stpr[nbr_active_stpr++] = s;


    __HAL_TIM_SET_COUNTER(s->htim, 0);
    __HAL_TIM_SET_AUTORELOAD(s->htim, 1);
    HAL_TIM_Base_Start_IT(s->htim);
}


void move_two_steppers(stepper *s1, stepper *s2, uint32_t steps, GPIO_PinState direction)
{
    move_stepper(s1 ,steps ,direction);
    move_stepper(s2 ,steps ,direction);
}


void isr_stepper_all(stepper *s1, stepper *s2) // used for 2 steppers with same speed at once
{
    uint8_t all_done = 1;
    stepper *active_steppers[2] = { s1, s2 };

    for (uint8_t i = 0; i < 2; i++)
    {
        stepper *s = active_steppers[i];

        if (s->steps_left == 0)
            continue;

        all_done = 0;

        if (s->step_state == 0)
        {
            HAL_GPIO_WritePin(s->step_port, s->step_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_AUTORELOAD(s->htim, 20);
            __HAL_TIM_SET_COUNTER(s->htim, 0);
            s->step_state = 1;
        }
        else
        {
            HAL_GPIO_WritePin(s->step_port, s->step_pin, GPIO_PIN_RESET);

            uint32_t steps_done = s->init_steps - s->steps_left;

            if (steps_done < s->accel_steps)
                s->delay = s->start_delay -
                           ((s->start_delay - s->target_delay) * steps_done) / s->accel_steps;
            else
                s->delay = s->target_delay;

            __HAL_TIM_SET_AUTORELOAD(s->htim, s->delay);
            __HAL_TIM_SET_COUNTER(s->htim, 0);

            s->steps_left--;
            s->step_state = 0;
        }
    }

    if (all_done)
    {
        HAL_TIM_Base_Stop_IT(s1->htim);
    }
}



void isr_stepper(stepper *s) // used for one stepper at the time
{


    if (s->steps_left == 0) {
        HAL_TIM_Base_Stop_IT(s->htim);
        return;
    }

      if (s->step_state == 0) {

    	  HAL_GPIO_WritePin(s->step_port,s->step_pin,GPIO_PIN_SET);
    	  __HAL_TIM_SET_AUTORELOAD(s->htim,20);
    	  __HAL_TIM_SET_COUNTER(s->htim,0);

    	  s->step_state = 1 ;

      }else{

    	  HAL_GPIO_WritePin(s->step_port,s->step_pin,GPIO_PIN_RESET);


    	    // linear acceleration over the first 30 steps
    	    uint32_t steps_done = s->init_steps - s->steps_left;

    	    if (steps_done < s->accel_steps) {
    	        s->delay = s->start_delay - ((s->start_delay - s->target_delay) * steps_done) / s->accel_steps;
    	    } else {
    	        s->delay = s->target_delay;
    	    }


    	  __HAL_TIM_SET_AUTORELOAD(s->htim,s->delay);
    	  __HAL_TIM_SET_COUNTER(s->htim, 0);

    	  s->steps_left-- ;
    	  s->step_state = 0 ;
      }
}


void robot_rotate(robot *r, float angle_deg, uint32_t cw)
{
    float angle_rad = angle_deg * (3.1415926f / 180.0f);
    float arc       = (r->wheel_base * 0.5f) * angle_rad;
    float circumference = 2.0f * 3.1415926f * r->wheel_radius;
    float revolutions   = arc / circumference;
    uint32_t step = (uint32_t)(revolutions * 200);

    if (step == 0) return;

    if (cw)
    {   // cw rotation
        move_stepper(r->left,  step, GPIO_PIN_SET);
        move_stepper(r->right, step, GPIO_PIN_RESET);
    }else{  // ccw rotation
        move_stepper(r->left,  step, GPIO_PIN_RESET);
        move_stepper(r->right, step, GPIO_PIN_SET);
    }
}






