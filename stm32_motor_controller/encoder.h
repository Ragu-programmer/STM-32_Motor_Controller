/**
 * encoder.h  --  Quadrature encoder position via TIM2 hardware counter.
 *
 * TIM2 is a 32-bit timer configured in encoder mode (x4 counting). The
 * counter tracks position in hardware with zero CPU load; we simply read
 * it as a signed value so that home = 0 and either direction is valid.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Configure GPIO + TIM2 encoder mode. Call once at startup. */
void encoder_init(void);

/* Current position in counts, signed relative to whatever encoder_set()
 * last established as zero. */
int32_t encoder_read(void);

/* Force the counter to a value (used by homing to define zero). */
void encoder_set(int32_t value);

#endif /* ENCODER_H */
