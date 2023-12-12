/* 
 * File:   motor_functions.h
 * Author: axeltremaudant
 *
 * Created on December 12, 2023, 11:15 AM
 */

#ifndef MOTOR_FUNCTIONS_H
#define	MOTOR_FUNCTIONS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>    
    
/*
 * Set the duty cycle of the PWM
 * @param duty: wanted duty cycle of the pwm
 * 0 <= duty <=1
 * @param right_motor: if true set duty cyle for the right motor, if false set for the left motor 
 */
void set_duty_cycle(float duty, bool right_motor);

/*
 * Set the rotating direction of the motor
 * @param clockwise: set the rotating direction of the motor clockwise
 * The rotating direction set here may not be really it depending how the motor is cabled
 * @param right_motor: if true, set the rotation of the right motor, else the left
 */
void set_rotation_clockwise(bool clockwise, bool right_motor);

/*
 * Set the rotating speed target of the motor.
 * @param target: wanted rotating speed of the motor, 
 * if <0, the motor rotate in the other direction
 */
void set_rotating_speed_target_r(int target);
void set_rotating_speed_target_l(int target);

/*
 * Callback function called by the timer1 interrupts each 20 ms 
 * for calculating the rotating speed of the motors
 */
void speed_rotation_measure();

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_FUNCTIONS_H */

