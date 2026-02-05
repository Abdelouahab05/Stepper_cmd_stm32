#include "paso.h"


// Variables
extern volatile uint8_t obstacle;
extern float distance;

// Limit of stepper is 2
stepper *active_stpr[2];
uint8_t nbr_active_stpr = 0;

// Calculation functions =================================================================================================

uint32_t speed_to_delay_us(float speed_rad_s)
{
    if (speed_rad_s <= 0) return 0;
    float steps_per_sec = (speed_rad_s * 800.0f) / (2.0f * 3.1415926f);
    return (uint32_t)(1000000.0f / steps_per_sec);
}

uint32_t distance_to_steps(float distance, float wheel_radius)
{
    float circumference = 2.0f * 3.1415926f * wheel_radius;
    float revolutions   = distance / circumference;
    return (uint32_t)(revolutions * 800.0f);
}

// Moving functions =========================================================================================================

void move_stepper(stepper *s, uint32_t steps, GPIO_PinState direction)
{
    if (steps <= 0) return;

    // Initialization of parameters

    s->steps_left = steps;
    s->init_steps = steps;
    s->dir = direction;
    s->step_state = 0;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin, direction);


    // Accel / Decel parameters

    s->accel_steps = 1800;
    if (s->accel_steps > steps / 2) s->accel_steps = steps / 2;
    float start_speed = 2.2f;

    // Counting the needed delay between steps

    s->start_delay  = speed_to_delay_us(start_speed);
    s->target_delay = speed_to_delay_us(s->speed_rad_s);

    s->delay = s->start_delay;

    // Check stepper existence

    if (nbr_active_stpr >= 2) {
        nbr_active_stpr = 0;
    }
    active_stpr[nbr_active_stpr++] = s;

    // Initialization of the used timer

    __HAL_TIM_SET_COUNTER(s->htim, 0);
    __HAL_TIM_SET_AUTORELOAD(s->htim, 1);
    HAL_TIM_Base_Start_IT(s->htim);
}

void move_two_steppers(stepper *s1, stepper *s2, uint32_t steps, GPIO_PinState direction)
{
    move_stepper(s1, steps, direction);
    move_stepper(s2, steps, direction);
}

// Moving Callback Functions =================================================================================================

void isr_stepper_all(stepper *s1, stepper *s2) // Called when controlling 2 steppers
{

    if (s1 == NULL || s1->htim == NULL || s1->step_port == NULL || s2 == NULL || s2->htim == NULL || s2->step_port == NULL) {
        return;
    }

    // Check if obstacle detected stop steps and halt the count of the number of steps left

    if (obstacle) {

        HAL_GPIO_WritePin(s1->step_port, s1->step_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(s2->step_port, s2->step_pin, GPIO_PIN_RESET);

        if (s2 != NULL) HAL_GPIO_WritePin(s2->step_port, s2->step_pin, GPIO_PIN_RESET);
        if (s1 != NULL) HAL_GPIO_WritePin(s1->step_port, s1->step_pin, GPIO_PIN_RESET);

        s1->step_state = 0;
        s2->step_state = 0;

        if (s2 != NULL) s2->step_state = 0;
        if (s1 != NULL) s1->step_state = 0;

        __HAL_TIM_SET_AUTORELOAD(s1->htim, 500);
        __HAL_TIM_SET_AUTORELOAD(s2->htim, 500);

        return;
    }

    //
    uint8_t all_done = 1;
    stepper *stprs[2] = { s1, s2 };

    // For each stepper used

    for (uint8_t i = 0; i < 2; i++) {
        stepper *s = stprs[i];

        // If stepper non existent or finished go out
        if (s == NULL || s->steps_left == 0) {
            continue;
        }

        all_done = 0;

        if (s->step_state == 0) {

            // start with generating the rising edge
            HAL_GPIO_WritePin(s->step_port, s->step_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_AUTORELOAD(s->htim, 100);
            s->step_state = 1;
        }
        else {

            // End with generating the falling edge
            HAL_GPIO_WritePin(s->step_port, s->step_pin, GPIO_PIN_RESET);
            uint32_t steps_done = s->init_steps - s->steps_left;

            // Configuration of the delay between steps while including the Acceleration / Deceleration Logic

            if (steps_done < s->accel_steps) {
                // Accelerating delay
                s->delay = s->start_delay - ((s->start_delay - s->target_delay) * steps_done) / s->accel_steps;
            }
            else if (s->steps_left <= s->accel_steps) {
                // Decelerating delay
                uint32_t decel_progress = s->accel_steps - s->steps_left;
                s->delay = s->target_delay + ((s->start_delay - s->target_delay) * decel_progress) / s->accel_steps;
            }
            else {
                // Target delay
                s->delay = s->target_delay;
            }

            // Generate the falling edge and the delay between steps

            __HAL_TIM_SET_AUTORELOAD(s->htim, s->delay);
            s->steps_left--;
            s->step_state = 0;
        }
    }

    // When finished stop

    if (all_done) {
        HAL_TIM_Base_Stop_IT(s1->htim);
        HAL_TIM_Base_Stop_IT(s2->htim);
        s1->step_state = 0;
        s2->step_state = 0;
        if (s2 != NULL) s2->step_state = 0;
        if (s1 != NULL) s1->step_state = 0;
    }
}

void isr_stepper(stepper *s) // Called when controlling 1 steppers ( no ultrasonic logic )
{

    // If stepper non existent or finished go out

    if (s->steps_left == 0) {
        HAL_TIM_Base_Stop_IT(s->htim);
        return;
    }



    if (s->step_state == 0) {

        // Start with generating the rising edge

        HAL_GPIO_WritePin(s->step_port, s->step_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_AUTORELOAD(s->htim, 20);
        __HAL_TIM_SET_COUNTER(s->htim, 0);
        s->step_state = 1;
    } else {

        // End with generating the falling edge

        HAL_GPIO_WritePin(s->step_port, s->step_pin, GPIO_PIN_RESET);
        uint32_t steps_done = s->init_steps - s->steps_left;

        // Configuration of the delay between steps while including the Acceleration / Deceleration Logic

        if (steps_done < s->accel_steps) {

            // Accelerating delay

            s->delay = s->start_delay - ((s->start_delay - s->target_delay) * steps_done) / s->accel_steps;

        } else if (s->steps_left <= s->accel_steps) {

            // Decelerating delay

            uint32_t decel_progress = s->accel_steps - s->steps_left;
            s->delay = s->target_delay + ((s->start_delay - s->target_delay) * decel_progress) / s->accel_steps;

        } else {

            // Target delay

            s->delay = s->target_delay;
        }

        // Generate the falling edge and the delay between steps

        __HAL_TIM_SET_AUTORELOAD(s->htim, s->delay);
        __HAL_TIM_SET_COUNTER(s->htim, 0);
        s->steps_left--;
        s->step_state = 0;
    }
}

void robot_rotate(robot *r, float angle_deg, uint32_t cw)
{
	// Counting the steps needed for the target angle

    float angle_rad = angle_deg * (3.1415926f / 180.0f);
    float arc       = (r->wheel_base * 0.5f) * angle_rad;
    float circumference = 2.0f * 3.1415926f * r->wheel_radius;
    float revolutions   = arc / circumference;
    uint32_t step = (uint32_t)(revolutions * 800.0f);

    if (step == 0) return;

    // Move

    if (cw) {
        move_stepper(r->left,  step, GPIO_PIN_SET);
        move_stepper(r->right, step, GPIO_PIN_RESET);
    } else {
        move_stepper(r->left,  step, GPIO_PIN_RESET);
        move_stepper(r->right, step, GPIO_PIN_SET);
    }
}


// Obstacle function =======================================================================================================

void sensor_control_loop(stepper *s)
{

	//Check if Ultrasonic sensor is existing

    if (s == NULL || s->trig_port == NULL) return;

    // Send the waves from the trigger
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    for(volatile int i = 0; i < 80; i++);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    HAL_Delay(60);

}

// Obstacle Callback Function =================================================================================================

void isr_ultrasonic(TIM_HandleTypeDef *htim){


	static uint32_t p1 = 0;
	static uint32_t p2 = 0;
	static uint8_t captured = 0;

	if (captured == 0) { // Rising Edge detected

	        p1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	        // Switch to falling edge detection

	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	        captured = 1;

	    } else { // Falling Edge detected

	        p2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	        uint32_t diff = 0;

	        // Calculating the time difference
	        if (p2 > p1) {
	            diff = p2 - p1;
	        } else {
	            diff = (0xFFFF - p1) + p2;
	        }

	        // Calculation
	        distance = (diff * 0.0343f) / 2.0f;

	        // Obstacle Logic
	        if (distance < 20.0f && distance > 2.0f)
	        {
	            obstacle = 1;
	        } else {
	            obstacle = 0;
	        }

	        // Reset timer for the next reading
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	        captured = 0;

	    }

}

