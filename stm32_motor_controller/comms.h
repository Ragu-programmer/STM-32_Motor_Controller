/**
 * comms.h  --  UART command interface (1-byte, interrupt driven).
 *
 * Each received byte is a complete single-character command handled in the
 * RX-complete interrupt. Anything that would block (printing telemetry) is
 * deferred to comms_service(), which runs in the main loop.
 */
#ifndef COMMS_H
#define COMMS_H

/* Configure USART2 (GPIO + peripheral) and arm the 1-byte RX interrupt. */
void comms_init(void);

/* Run from the main loop: prints telemetry when a status command was seen. */
void comms_service(void);

#endif /* COMMS_H */
