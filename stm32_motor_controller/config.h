/**
 * config.h  --  All tunable parameters for the STM32F446RE gripper actuator.
 *
 * Everything hardware- or behaviour-specific lives here so the rest of the
 * firmware never contains magic numbers. Change wiring, gains, limits and
 * homing behaviour from this one file.
 *
 * Board:  Nucleo-F446RE (SYSCLK 180 MHz via internal HSI PLL, see main.c)
 * Driver: TB6612FNG-style H-bridge (PWM + IN1/IN2 direction + STBY enable)
 *         Works unchanged for DRV8833 / L298N-style drivers that take one
 *         PWM line and two direction lines.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* =========================================================================
 * 1. CLOCK ASSUMPTIONS
 * -------------------------------------------------------------------------
 * With the SystemClock_Config() in main.c (180 MHz):
 *   TIM1  is on APB2 -> timer clock = 180 MHz   (motor PWM)
 *   TIM2  is on APB1 -> timer clock =  90 MHz   (encoder, clock irrelevant)
 *   TIM6  is on APB1 -> timer clock =  90 MHz   (control-loop tick)
 * If you change the clock tree, update the two *_TIM_CLK_HZ values below.
 * ========================================================================= */
#define PWM_TIM_CLK_HZ    180000000UL
#define TICK_TIM_CLK_HZ    90000000UL

/* =========================================================================
 * 2. MOTOR PWM  (TIM1_CH1)
 * ========================================================================= */
#define PWM_FREQ_HZ       20000UL          /* 20 kHz -> inaudible            */
#define PWM_RES           1000UL           /* duty resolution (0..PWM_ARR)   */
#define PWM_ARR           (PWM_RES - 1UL)
#define PWM_PSC           ((PWM_TIM_CLK_HZ / (PWM_FREQ_HZ * PWM_RES)) - 1UL)

/* Flip this if a positive PWM command makes the encoder count DECREASE.
 * (Easiest calibration: home, send 'o', watch whether POS grows or shrinks.)*/
#define MOTOR_INVERT      0

/* =========================================================================
 * 3. CONTROL LOOP TICK  (TIM6, period-elapsed interrupt)
 * ========================================================================= */
#define CONTROL_HZ        1000UL           /* 1 kHz control loop -> dt = 1 ms */
#define TICK_PSC          ((TICK_TIM_CLK_HZ / 1000000UL) - 1UL)   /* -> 1 MHz */
#define TICK_ARR          ((1000000UL / CONTROL_HZ) - 1UL)
#define DT_MS             (1000.0f / (float)CONTROL_HZ)

/* =========================================================================
 * 4. PIN MAP  (Nucleo-F446RE)
 * ------------------------------------------------------------------------- */
/* Motor PWM: TIM1_CH1 on PA8 (AF1)                                          */
#define MOTOR_PWM_PORT    GPIOA
#define MOTOR_PWM_PIN     GPIO_PIN_8
#define MOTOR_PWM_AF      GPIO_AF1_TIM1

/* Direction / enable (TB6612FNG AIN1, AIN2, STBY)                           */
#define MOTOR_IN1_PORT    GPIOB
#define MOTOR_IN1_PIN     GPIO_PIN_4
#define MOTOR_IN2_PORT    GPIOB
#define MOTOR_IN2_PIN     GPIO_PIN_5
#define MOTOR_STBY_PORT   GPIOB
#define MOTOR_STBY_PIN    GPIO_PIN_10

/* Quadrature encoder: TIM2_CH1/CH2 on PA0/PA1 (AF1)                         */
#define ENC_A_PORT        GPIOA
#define ENC_A_PIN         GPIO_PIN_0
#define ENC_B_PORT        GPIOA
#define ENC_B_PIN         GPIO_PIN_1
#define ENC_AF            GPIO_AF1_TIM2
#define ENC_FILTER        6               /* TIM input filter 0..15          */

/* Command / telemetry UART: USART2 PA2 (TX) / PA3 (RX), AF7.
 * On the Nucleo this is bridged to the ST-Link virtual COM port.            */
#define UART_BAUD         115200UL

/* =========================================================================
 * 5. TRAVEL LIMITS  (encoder counts, home = 0)
 * -------------------------------------------------------------------------
 * Homing drives to the CLOSED mechanical stop and calls that 0. Full travel
 * measured on the real actuator was ~2100-2200 counts, so:
 * ========================================================================= */
#define POS_MIN            0               /* closed hard stop (home)        */
#define POS_MAX            2200            /* open hard stop                 */
#define CLOSE_POS          60              /* "closed" target (off the stop) */
#define OPEN_POS           2100            /* "open"  target                 */
#define NUDGE_STEP         50              /* counts per +/- command         */

/* =========================================================================
 * 6. PD CONTROL GAINS + FEATURE PARAMETERS
 * -------------------------------------------------------------------------
 * Units: error in counts, velocity in counts/tick (== counts/ms at 1 kHz),
 * output in PWM duty units (0..PWM_ARR). All are starting points -- see the
 * tuning guide in README.md.
 * ========================================================================= */
#define KP                 0.60f          /* proportional gain              */
#define KD                 8.0f           /* derivative gain (on velocity)  */

/* Velocity-aware arrival detection: "arrived" only when close AND slow,
 * held stable for a debounce window (rejects momentary overshoot).         */
#define POS_TOL            15             /* |error| considered "on target" */
#define VEL_TOL            3.0f           /* |velocity| considered "stopped" */
#define ARRIVE_DEBOUNCE    25             /* ticks stable before HOLDING     */
#define POS_HYST           25             /* re-arm move if drift > tol+hyst */

/* Deadzone compensation: minimum duty that actually produces motion. Any
 * non-zero command below this is bumped up to it while still off-target.   */
#define MIN_MOVE_PWM       150.0f

/* Startup boost: extra kick to break static friction / take up backlash at
 * the start of a move. Applied while off-target and not yet moving.        */
#define BOOST_PWM          350.0f
#define BOOST_TICKS        80             /* max boost duration (~80 ms)     */
#define STALL_VEL          1.0f           /* below this = "not moving yet"   */

#define PWM_MAX_CMD        950.0f         /* clamp (leave headroom < ARR)    */

/* =========================================================================
 * 7. HOMING-SETTLE CALIBRATION
 * -------------------------------------------------------------------------
 * Gently drive toward a mechanical stop, detect the stall, let the
 * mechanics settle, then zero the encoder and back off.
 * ========================================================================= */
#define HOME_DIR              (-1)        /* seek toward CLOSED/min stop     */
#define HOME_PWM              260.0f      /* gentle constant seek duty       */
#define HOME_STALL_VEL        1.5f        /* below this = pressed on stop    */
#define HOME_STALL_DEBOUNCE   60          /* ticks stalled = confirmed stop  */
#define HOME_MIN_SEEK_TICKS   120         /* ignore stall for first ~120 ms  */
#define HOME_TIMEOUT_TICKS    4000        /* safety abort (~4 s)             */
#define HOME_SETTLE_TICKS     200         /* rest at the stop before zeroing */
#define HOME_ZERO_VALUE       0           /* encoder value assigned at stop  */
#define HOME_BACKOFF          60          /* move off the stop after zeroing */

/* Velocity smoothing (EMA) applied to the raw per-tick delta.              */
#define VEL_FILTER_ALPHA      0.30f

#endif /* CONFIG_H */
