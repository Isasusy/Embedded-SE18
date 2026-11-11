/*
 *	drv_line.h / .c -- subsystem 3 hardware, through the kernel's drivers (no Pico SDK):
 *	  * the RP2040 ADC (read directly) for the two line IR sensors (GP26 = ADC ch 0 = IR1 left, GP27 = ADC ch 1 = IR2 right)
 *	  * GP28 (IR3, barcode) = ADC ch 2, sampled every 1 ms from barcode_task and turned into black/white by a
 *	    peak detector (BAR_SWING): works for blurred narrow bars, and for a digital 0/3.3 V sensor too
 *	line_task and barcode_task may run on different cores, so every conversion is taken under a lock.
 *	The edge ring buffer is used ONLY by barcode_task.
 *
 *	No shared-config edit is needed any more: this file sets up the two analog pins itself.
 */
#ifndef DRV_LINE_H
#define DRV_LINE_H

#include <stdint.h>
#include <stdbool.h>

#define PIN_LINE_L	26
#define PIN_LINE_R	27
#define PIN_BARCODE	28

bool	line_adc_init(void);				/* once, from subsystem3_init; false = ADC not ready (readings stay 0) */
void	line_adc_read(int *left, int *right);		/* 12-bit counts */

typedef struct { uint32_t t_us; bool black; int peak; } bar_edge_t;	/* peak = darkest/lightest reading of the stripe that just ended */
void	barcode_init(void);				/* GP28 as an analog input */
int	barcode_raw(void);				/* last GP28 reading, 0..4095 (for the log) */
bool	barcode_is_black(void);
void	barcode_sample(void);				/* call every ~1 ms: records an edge when the level changed */
bool	barcode_edge_pop(bar_edge_t *e);		/* false = none waiting */
unsigned barcode_edge_overruns(void);

#endif
