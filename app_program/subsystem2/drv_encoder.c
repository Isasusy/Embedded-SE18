#include "subsystem2.h"
#include <bsp/libbsp.h>
#include "drv_encoder.h"
#include "motion_config.h"

#define INTNO_IO_BANK0		13			/* RP2040 IO_IRQ_BANK0 */
#define ENC_INT_PRI		2			/* same level the kernel's ADC / I2C drivers use */
#define IO_BANK0_BASE		0x40014000u
#define IO_INTR0		(*(volatile unsigned int *)(IO_BANK0_BASE + 0x0F0u))	/* GPIO0-7 raw / edge latches (write 1 to clear) */
#define IO_PROC0_INTE0		(*(volatile unsigned int *)(IO_BANK0_BASE + 0x100u))	/* core 0 interrupt enable, GPIO0-7 */
#define SIO_GPIO_IN		(*(volatile unsigned int *)0xD0000004u)
/* per GPIO n the nibble at 4n holds LEVEL_LOW, LEVEL_HIGH, EDGE_LOW (bit 2), EDGE_HIGH (bit 3): edges of GP2..GP5 */
#define ENC_EDGE_MASK		0x00CCCC00u

static volatile int32_t	s_cnt_l, s_cnt_r;
static volatile unsigned s_irqs;
static uint8_t		s_prev_l, s_prev_r;

/* index = (previous state << 2) | new state, state = (A << 1) | B.  Gray sequence 00 01 11 10 = +1 */
static const int8_t qtab[16] = { 0, +1, -1, 0,  -1, 0, 0, +1,  +1, 0, 0, -1,  0, -1, +1, 0 };

static void count(unsigned g)
{
	uint8_t nl = (uint8_t)((((g >> ENC_L_A) & 1u) << 1) | ((g >> ENC_L_B) & 1u));
	uint8_t nr = (uint8_t)((((g >> ENC_R_A) & 1u) << 1) | ((g >> ENC_R_B) & 1u));

	s_cnt_l += MC_ENC_L_SIGN * qtab[(s_prev_l << 2) | nl];
	s_cnt_r += MC_ENC_R_SIGN * qtab[(s_prev_r << 2) | nr];
	s_prev_l = nl;
	s_prev_r = nr;
}

static void enc_isr(UINT intno)
{
	s_irqs++;
	IO_INTR0 = ENC_EDGE_MASK;		/* acknowledge first: an edge that arrives now is counted below or re-fires */
	count(SIO_GPIO_IN);
	ClearInt(intno);
}

/* pull-up on a pad (PADS_BANK0: GPIOn at 0x4001C004 + 4n; PUE = bit 3, PDE = bit 2) */
static void pull_up(unsigned pin)
{
	volatile unsigned int *pad = (volatile unsigned int *)(0x4001C004u + 4u * pin);
	*pad = (*pad | (1u << 3)) & ~(1u << 2);
}

int enc_init(void)
{
	const T_DINT dint = { .intatr = TA_HLNG, .inthdr = enc_isr };
	ER err;

	for (unsigned p = ENC_L_A; p <= ENC_R_B; p++) {
		gpio_set_pin(p, GPIO_MODE_IN);
		pull_up(p);
	}
	{
		unsigned g = SIO_GPIO_IN;
		s_prev_l = (uint8_t)((((g >> ENC_L_A) & 1u) << 1) | ((g >> ENC_L_B) & 1u));
		s_prev_r = (uint8_t)((((g >> ENC_R_A) & 1u) << 1) | ((g >> ENC_R_B) & 1u));
	}

	err = tk_def_int(INTNO_IO_BANK0, &dint);
	if (err < E_OK) return (int)err;
	IO_INTR0 = ENC_EDGE_MASK;		/* drop edges latched before we were ready */
	IO_PROC0_INTE0 |= ENC_EDGE_MASK;	/* GP2..GP5, both edges, to core 0 */
	ClearInt(INTNO_IO_BANK0);
	EnableInt(INTNO_IO_BANK0, ENC_INT_PRI);
	return 0;
}

void enc_poll(void)
{
	count(SIO_GPIO_IN);
}

void enc_read(int32_t *left, int32_t *right)
{
	*left = s_cnt_l;
	*right = s_cnt_r;
}

unsigned enc_pins(void)      { return (SIO_GPIO_IN >> ENC_L_A) & 0xFu; }
unsigned enc_irq_count(void) { return s_irqs; }
