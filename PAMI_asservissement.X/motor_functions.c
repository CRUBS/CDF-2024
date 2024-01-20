#include "motor_functions.h"
#include "xc.h"

/*      Global variables        */

// CONSTANTS    
    
// 32 = nb points coder ; 4 because the QEI mode is x4 ; angle in radians = 0.1309
#define ANGLE_CODER 360.0 / 32.0 / 4.0 * 3.1415926535897932384626433 / 180 
#define TIME_INTERVAL 0.03 // time between two call of interrupt of timer 1 in seconds
    
const float rotating_speed_coef = ANGLE_CODER / TIME_INTERVAL; // Number of rad between 2 pulses divided by the interval
const int kp = 3, ki = 20; // Coef PI
const float pi_coef = 13.0;

// RIGHT MOTOR
int old_position_r = 0; // Previous position of the encoder
// PID
volatile int previous_error_r = 0.0, integral_r = 0.0;
volatile int rotating_speed_target_r = 0; // rad/s

// LEFT MOTOR
int old_position_l = 0; // Previous position of the encoder
// PID
volatile int previous_error_l = 0.0, integral_l = 0.0;
volatile int rotating_speed_target_l = 0; // rad/s

/*      Functions       */

void set_duty_cycle(float duty, bool right_motor)
{
    // duty must be between 0 and 1
    if(duty < 0) duty = 0.0;
    else if(duty > 1) duty = 1.0;
    
    // Set duty cycle
    if(right_motor) PG1DC = MPER * (LATAbits.LATA0 ? duty : 1 - duty);
    else PG2DC = MPER * (LATAbits.LATA1 ? duty : 1 - duty);
}

void set_rotation_clockwise(bool clockwise, bool right_motor)
{
    if(right_motor) LATAbits.LATA0 = clockwise;
    else LATAbits.LATA1 = clockwise;
}

void set_rotating_speed_target_r(int target)
{
    if (target != rotating_speed_target_r)
    {
        rotating_speed_target_r = target;
        set_rotation_clockwise(target > 0, true);

        // Reset PI variables
        integral_r = previous_error_r = 0;
    }
}

void set_rotating_speed_target_l(int target)
{
    if(target != rotating_speed_target_l)
    {
        rotating_speed_target_l = target;
        set_rotation_clockwise(target > 0, false);

        // Reset PI variables
        integral_l = previous_error_l = 0;
    }
}

/*
 * Enslave the right motor to rotate at the speed defined by rotating_speed_target
 * depending on the current speed 
 * @param speed: current rotating speed of the motor in rad/s
 * @param time_interval: time between two controls
 */
void control_motor_speed_r(int speed, float time_interval)
{
    // Calculate the error between the target speed and current speed
    int error = rotating_speed_target_r - speed;
    
    // Set the error with the right sign
    if(rotating_speed_target_r < 0 || (rotating_speed_target_r == 0 && speed < 0)) 
        error = -error;
    
    // Calculate the proportional term
    int proportional = kp * error;
    
    // Calculate the integral term
    integral_r += ki * error * time_interval;
    
    // Change the rotating speed ; 670 is present to put the value between 0 and 1
    set_duty_cycle((float) (proportional + integral_r) / pi_coef, true);
    
    previous_error_r = error; // Update the error
}

/*
 * Enslave the left motor to rotate at the speed defined by rotating_speed_target
 * depending on the current speed 
 * @param speed: current rotating speed of the motor in rad/s
 * @param time_interval: time between two controls
 */
void control_motor_speed_l(int speed, float time_interval)
{
    // Calculate the error between the target speed and current speed
    int error = rotating_speed_target_l - speed;
    
    // Set the error with the right sign
    if(rotating_speed_target_l < 0 || (rotating_speed_target_l == 0 && speed < 0)) 
        error = -error;
    
    // Calculate the proportional term
    int proportional = kp * error;
    
    // Calculate the integral term
    integral_l += ki * error * time_interval;
    
    // Change the rotating speed ; 670 is present to put the value between 0 and 1
    set_duty_cycle((float) (proportional + integral_l) / pi_coef, false);
    
    previous_error_l = error; // Update the error
}

/*
 * Calculate the rotating speed of the right motor
 */
void speed_rotation_measure_r()
{    
    int current_position = (int) POS1CNTL; // Get the pulse count
    
    // Calculate the rotating speed in rad/s ; 
    // Around 13 rad/s at max speed
    int rotating_speed = (current_position - old_position_r) * rotating_speed_coef;
    
    old_position_r = current_position;
    
    control_motor_speed_r(rotating_speed, TIME_INTERVAL); // Enslave
}

/*
 * Calculate the rotating speed of the left motor
 */
void speed_rotation_measure_l()
{    
    int current_position = (int) POS2CNTL; // Get the pulse count
    
    // Calculate the rotating speed in rad/s ; 
    // Around 13 rad/s at max speed
    int rotating_speed = (current_position - old_position_l) * rotating_speed_coef;
    
    old_position_l = current_position;
    
    control_motor_speed_l(rotating_speed, TIME_INTERVAL); // Enslave
}

/*
 * Funtion to be called by the timer interrupt
 */
void speed_rotation_measure()
{
    speed_rotation_measure_r();
    speed_rotation_measure_l();
}
