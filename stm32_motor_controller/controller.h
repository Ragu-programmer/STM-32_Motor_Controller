/**
 * controller.h  --  Closed-loop position controller (PD + practical extras).
 *
 * controller_tick() is the real-time loop; call it at CONTROL_HZ from the
 * TIM6 period-elapsed interrupt. All other functions are commands/queries
 * that are safe to call from the UART interrupt or the main loop (they only
 * touch small aligned words and set flags -- no blocking).
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

typedef enum {
    ST_DISABLED = 0,   /* motor coasting, no control                         */
    ST_HOMING_SEEK,    /* driving toward mechanical stop                     */
    ST_HOMING_SETTLE,  /* resting on stop, about to zero the encoder         */
    ST_MOVING,         /* converging to target                              */
    ST_HOLDING         /* arrived; holding position                          */
} ctrl_state_t;

void controller_init(void);
void controller_tick(void);                 /* call at CONTROL_HZ (from IRQ)  */

/* ---- commands ----------------------------------------------------------- */
void controller_home(void);                 /* run homing-settle calibration  */
void controller_set_target(int32_t counts); /* clamped to [POS_MIN, POS_MAX]  */
void controller_nudge(int32_t delta);       /* relative move                  */
void controller_stop(void);                 /* hold at current position       */
void controller_disable(void);              /* coast, no control              */
void controller_enable(void);               /* re-enable, hold where it is    */

/* ---- telemetry (all integer, no float printf needed) -------------------- */
ctrl_state_t controller_state(void);
const char  *controller_state_name(void);
int32_t      controller_target(void);
int32_t      controller_position(void);
int32_t      controller_error(void);
int32_t      controller_velocity_cps(void); /* counts per second             */
int32_t      controller_pwm(void);          /* last applied signed duty       */

#endif /* CONTROLLER_H */
