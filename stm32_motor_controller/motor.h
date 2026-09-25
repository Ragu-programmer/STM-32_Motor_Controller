/**
 * motor.h  --  Low-level H-bridge driver (PWM magnitude + direction).
 */
#ifndef MOTOR_H
#define MOTOR_H

/* Configure GPIO + TIM1 PWM and enable the driver. Call once at startup. */
void motor_init(void);

/* Drive the motor with a signed command.
 *   u > 0  : one direction, duty = |u|
 *   u < 0  : the other direction, duty = |u|
 *   u == 0 : zero duty (still driven, i.e. actively held near stop)
 * |u| is clamped to the PWM resolution. MOTOR_INVERT in config.h flips sign. */
void motor_set(float u);

/* Let the motor spin freely (both half-bridges off / STBY concept). */
void motor_coast(void);

/* Short-brake the motor (both low-side on) -- strong passive hold. */
void motor_brake(void);

#endif /* MOTOR_H */
