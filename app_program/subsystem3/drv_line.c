#include "subsystem3.h"
#include <bsp/libbsp.h>
#include "drv_line.h"
#include "line_config.h"
#include "s3_port.h"

/* ---- line + barcode sensors: the RP2040 ADC read directly by polling (no kernel ADC driver, no interrupt) ----
   One conversion takes about 2 us. line_task (5 ms) and barcode_task (1 ms) share it and may run on different
   cores, so each conversion holds a lock for those 2 us (a kernel spin lock on the SMP build). */
#if TK_SUPPORT_SMP
#include <tk/smp.h>
static T_SPLOCK s_adc_lock;
#define ADC_LOCK(s)	(void)ISpinLock(&s_adc_lock, &(s))
#define ADC_UNLOCK(s)	(void)ISpinUnlock(&s_adc_lock, (s))
#else
#define ADC_LOCK(s)	DI(s)
#define ADC_UNLOCK(s)	EI(s)
#endif
#define ADC_CS			(*(volatile unsigned int *)0x4004C000u)
#define ADC_RESULT		(*(volatile unsigned int *)0x4004C004u)
#define ADC_CS_EN		(1u << 0)
#define ADC_CS_START_ONCE	(1u << 2)
#define ADC_CS_READY		(1u << 8)
#define ADC_CS_AINSEL_MASK	(7u << 12)
#define RESETS_RESET		(*(volatile unsigned int *)0x4000C000u)
#define RESETS_RESET_DONE	(*(volatile unsigned int *)0x4000C008u)
#define RESETS_ADC		(1u << 0)

static bool s_adc_up;

/* make a pin a pure analog input: function NULL, input buffer and both pulls off */
static void analog_pin(unsigned pin)
{
	volatile unsigned int *ctrl = (volatile unsigned int *)(0x40014004u + 8u * pin);	/* IO_BANK0 GPIOn_CTRL */
	volatile unsigned int *pad  = (volatile unsigned int *)(0x4001C004u + 4u * pin);	/* PADS_BANK0 GPIOn   */

	*ctrl = 0x1Fu;							/* FUNCSEL = NULL */
	*pad  = *pad & ~((1u << 6) | (1u << 3) | (1u << 2));		/* IE, PUE, PDE off */
}

bool line_adc_init(void)
{
	unsigned n;

#if TK_SUPPORT_SMP
	(void)InitSpinLock(&s_adc_lock);
#endif
	RESETS_RESET &= ~RESETS_ADC;					/* release the ADC from reset (harmless if already) */
	for (n = 0; n < 100000u && !(RESETS_RESET_DONE & RESETS_ADC); n++) { }
	analog_pin(PIN_LINE_L);
	analog_pin(PIN_LINE_R);
	analog_pin(PIN_BARCODE);
	ADC_CS = ADC_CS_EN;
	for (n = 0; n < 100000u && !(ADC_CS & ADC_CS_READY); n++) { }
	s_adc_up = (ADC_CS & ADC_CS_READY) != 0;
	if (!s_adc_up) S3_PRINTF("[S3] ADC did not become ready: line sensors will read 0\n");
	return s_adc_up;
}

static int adc_ch(int ch)
{
	unsigned n;
	UINT s;
	int v;

	if (!s_adc_up) return 0;
	ADC_LOCK(s);
	ADC_CS = (ADC_CS & ~ADC_CS_AINSEL_MASK) | ((unsigned)ch << 12) | ADC_CS_EN;
	ADC_CS |= ADC_CS_START_ONCE;
	for (n = 0; n < 10000u && !(ADC_CS & ADC_CS_READY); n++) { }
	v = (int)(ADC_RESULT & 0x0FFFu);
	ADC_UNLOCK(s);
	return v;
}

void line_adc_read(int *left, int *right)
{
	*left  = adc_ch(0);			/* GP26 = ADC0 */
	*right = adc_ch(1);			/* GP27 = ADC1 */
}

/* ---- barcode: polled edge recorder ---- */
#define RING	64u				/* power of two */
static bar_edge_t	s_ring[RING];
static unsigned		s_head, s_tail, s_over;
static int		s_last = -1;		/* -1 = not started, 0 = white, 1 = black */
static volatile int	s_raw;

void barcode_init(void)
{
	s_last = -1;				/* GP28 was made analog by line_adc_init() */
}

int barcode_raw(void) { return s_raw; }
bool barcode_is_black(void) { return s_last == 1; }

void barcode_sample(void)
{
	uint32_t t = S3_US32();
	int v = adc_ch(2);			/* GP28 = ADC2 */
	s_raw = v;
#if !BARCODE_BLACK_IS_HIGH
	v = 4095 - v;				/* make "black" the high side */
#endif
	int level;
	if (s_last == 1)      level = (v < BAR_ADC_THRESH - BAR_ADC_HYST) ? 0 : 1;	/* hysteresis: no chatter at the edge */
	else                  level = (v > BAR_ADC_THRESH + BAR_ADC_HYST) ? 1 : 0;

	if (s_last < 0) { s_last = level; return; }
	if (level == s_last) return;
	s_last = level;
	if (s_head - s_tail >= RING) { s_over++; return; }
	s_ring[s_head % RING].t_us  = t;
	s_ring[s_head % RING].black = (level == 1);
	s_head++;
}

bool barcode_edge_pop(bar_edge_t *e)
{
	if (s_tail == s_head) return false;
	*e = s_ring[s_tail % RING];
	s_tail++;
	return true;
}

unsigned barcode_edge_overruns(void) { return s_over; }
