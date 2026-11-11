/*
 *	drv_encoder.h / drv_encoder.c
 *	Subsystem 2 -- wheel encoders, quadrature x4, counted in a GPIO edge interrupt (kernel interrupt API, no Pico SDK).
 *	Left: GP2 (A) / GP3 (B).  Right: GP4 (A) / GP5 (B).
 *	The wheel encoders produce thousands of edges per wheel turn, far too fast to poll at 1 kHz, so every edge
 *	interrupts (IO_IRQ_BANK0, interrupt 13, enabled on core 0) and the ISR only updates two counters.
 *	enc_task calls enc_init() once. If the interrupt cannot be set up it falls back to polling (enc_poll, 1 kHz).
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

#define ENC_L_A		2
#define ENC_L_B		3
#define ENC_R_A		4
#define ENC_R_B		5

int	enc_init(void);					/* 0 = interrupt running, < 0 = kernel error (use enc_poll) */
void	enc_poll(void);					/* fallback: sample the four pins once */
void	enc_read(int32_t *left, int32_t *right);
unsigned enc_pins(void);				/* debug: the four pin levels, bit0 = GP2 (L_A) ... bit3 = GP5 (R_B) */
unsigned enc_irq_count(void);			/* debug: how many edge interrupts since boot */	/* cumulative edge counts, forward positive */

#endif
