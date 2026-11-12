/*
 *	drv_imu.c
 *	Subsystem 4 -- MPU-6050 family driver, through the kernel's I2C device driver (device "iica" = I2C0, GP0 SDA / GP1 SCL).
 *
 *	The kernel's driver is configured in device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h: its I2C0 pins
 *	must be GP0 (SDA) / GP1 (SCL), see SHARED_CONFIG_CHANGES.md. Only imu_task calls this file.
 */
#include "subsystem4.h"		/* kernel types first: s4_port.h uses tk_get_otm / SYSTIM */
#include "drv_imu.h"
#include "imu_config.h"
#include "s4_port.h"
#include "../../device/include/dev_i2c.h"	/* T_I2C_EXEC, TDN_I2C_EXEC, dev_init_i2c */

/* MPU-6050 / 6500 registers */
#define REG_SMPLRT_DIV		0x19
#define REG_CONFIG		0x1A
#define REG_GYRO_CONFIG		0x1B
#define REG_ACCEL_CONFIG	0x1C
#define REG_ACCEL_XOUT_H	0x3B	/* 14 bytes: AX AY AZ TEMP GX GY GZ (big-endian int16) */
#define REG_PWR_MGMT_1		0x6B
#define REG_WHO_AM_I		0x75

static uint8_t s_addr   = 0x68;
static uint8_t s_who    = 0x00;
static bool    s_bus_up = false;

uint8_t imu_hw_addr(void)	{ return s_addr; }
uint8_t imu_hw_who_am_i(void)	{ return s_who;  }

static ID s_dd = 0;				/* device descriptor of "iica" */

static int s_last_er = 0;			/* last kernel error code, printed when the chip does not answer */

/* Read: the kernel I2C driver's "execute" request = write the register number, then read rcv_n bytes
   (repeated start). It requires BOTH sizes to be > 0. */
static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
	T_I2C_EXEC ex;
	SZ asz;
	ER e;

	if (s_dd <= 0) return false;
	ex.sadr     = addr;			/* 7-bit address */
	ex.snd_size = 1;
	ex.snd_data = &reg;
	ex.rcv_size = (SZ)len;
	ex.rcv_data = buf;
	e = tk_swri_dev(s_dd, TDN_I2C_EXEC, &ex, sizeof ex, &asz);
	s_last_er = (int)e;
	return e >= E_OK;
}

/* Write: a plain device write, where the "start" argument is the 7-bit slave address. */
static bool reg_write(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t b[2];
	SZ asz;
	ER e;

	if (s_dd <= 0) return false;
	b[0] = reg;
	b[1] = val;
	e = tk_swri_dev(s_dd, (INT)addr, b, 2, &asz);
	s_last_er = (int)e;
	return e >= E_OK;
}

/* Called only when the chip does not answer: show WHY, and which addresses on the bus (if any) do answer. */
static void diagnose(void)
{
	static bool done;
	uint8_t b;
	SZ asz;
	uint32_t t0 = S4_US32();
	int found = 0;

	if (done) return;			/* once per power-up is enough */
	done = true;
	S4_PRINTF("[IMU] last error code = %d (E_IO -> device did not acknowledge; E_TMOUT -> bus stuck / no pull-ups)\n", s_last_er);
	/* a probe that times out takes DEVCNF_I2C0_TMO (10 ms) or more: do not scan 112 addresses on a stuck bus */
	if (tk_srea_dev(s_dd, 0x68, &b, 1, &asz) >= E_OK) found = 1;
	if ((uint32_t)(S4_US32() - t0) > 8000u) {
		S4_PRINTF("[IMU] probes time out slowly: SDA/SCL stuck low or not connected. Skipping bus scan.\n");
		return;
	}
	S4_PRINTF("[IMU] bus scan, addresses in decimal (104 = 0x68):");
	for (int a = 0x08; a <= 0x77; a++) {
		if (tk_srea_dev(s_dd, a, &b, 1, &asz) >= E_OK) { S4_PRINTF(" %d", a); found = 1; }
	}
	if (found) S4_PRINTF("\n"); else S4_PRINTF(" none\n");
}

/* Make sure GP0 / GP1 really are the I2C0 pins, with the internal pull-ups on, whatever the kernel did at start-up.
   IO_BANK0 GPIOn_CTRL = 0x40014004 + 8n (FUNCSEL 3 = I2C); PADS_BANK0 GPIOn = 0x4001C004 + 4n. */
static void pin_mux(unsigned pin)
{
	volatile unsigned int *ctrl = (volatile unsigned int *)(0x40014004u + 8u * pin);
	volatile unsigned int *pad  = (volatile unsigned int *)(0x4001C004u + 4u * pin);

	*pad  = (*pad | (1u << 6) | (1u << 3) | (1u << 1)) & ~((1u << 7) | (1u << 2));	/* IE, PUE, schmitt on; OD, PDE off */
	*ctrl = 3u;
}

bool imu_hw_init(imu_delay_fn delay_ms)
{
	if (!s_bus_up) {
		s_dd = tk_opn_dev((UB *)"iica", TD_UPDATE);
		if (s_dd < E_OK) {			/* not registered yet: register I2C0, then open again */
			dev_init_i2c(0);
			s_dd = tk_opn_dev((UB *)"iica", TD_UPDATE);
		}
		if (s_dd < E_OK) { s_dd = 0; return false; }
		pin_mux(PIN_IMU_SDA);
		pin_mux(PIN_IMU_SCL);
		s_bus_up = true;
	}

	/* probe both possible addresses (AD0 low = 0x68, AD0 high = 0x69) */
	bool found = false;
	static const uint8_t cand[2] = { 0x68, 0x69 };
	for (int i = 0; i < 2 && !found; i++) {
		uint8_t who;
		if (reg_read(cand[i], REG_WHO_AM_I, &who, 1)) {
			s_addr = cand[i];
			s_who  = who;
			found  = true;
		}
	}
	if (!found) {
		S4_PRINTF("[IMU] no device answered at 0x68/0x69 on I2C0 (SDA=GP%d SCL=GP%d). Check wiring/power.\n",
			  PIN_IMU_SDA, PIN_IMU_SCL);
		diagnose();
		return false;
	}

	/* 0x68 = MPU6050, 0x70 = MPU6500, 0x71/0x73 = MPU9250/9255. Others: warn; calibration sanity-checks. */
	if (s_who != 0x68 && s_who != 0x70 && s_who != 0x71 && s_who != 0x73) {
		S4_PRINTF("[IMU] WARNING: unexpected WHO_AM_I = %d at %d - not an MPU60x0/65x0/92x0? "
			  "Register map may not match.\n", (unsigned)s_who, (unsigned)s_addr);
	} else {
		S4_PRINTF("[IMU] found chip, WHO_AM_I = %d at address %d\n",
			  (unsigned)s_who, (unsigned)s_addr);
	}

	bool ok = true;
	ok &= reg_write(s_addr, REG_PWR_MGMT_1, 0x80);		/* device reset */
	delay_ms(100);
	ok &= reg_write(s_addr, REG_PWR_MGMT_1, 0x01);		/* wake, clock = gyro X PLL */
	delay_ms(50);
	ok &= reg_write(s_addr, REG_SMPLRT_DIV, 0x00);		/* internal 1 kHz; we poll at 100 Hz */
	ok &= reg_write(s_addr, REG_CONFIG,       0x02);	/* DLPF ~94 Hz accel / 98 Hz gyro */
	ok &= reg_write(s_addr, REG_GYRO_CONFIG,  0x08);	/* +-500 dps  -> 65.5 LSB/dps */
	ok &= reg_write(s_addr, REG_ACCEL_CONFIG, 0x10);	/* +-8 g      -> 4096 LSB/g   */
	delay_ms(20);

	if (!ok) S4_PRINTF("[IMU] configuration write failed\n");
	return ok;
}

static inline int16_t be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

bool imu_hw_read(imu_raw_t *o)
{
	uint8_t b[14];
	if (!reg_read(s_addr, REG_ACCEL_XOUT_H, b, sizeof b)) return false;

	o->ax_g   = be16(&b[0])  / IMU_ACCEL_LSB_PER_G;
	o->ay_g   = be16(&b[2])  / IMU_ACCEL_LSB_PER_G;
	o->az_g   = be16(&b[4])  / IMU_ACCEL_LSB_PER_G;
	/* b[6..7] = temperature, unused */
	o->gx_dps = be16(&b[8])  / IMU_GYRO_LSB_PER_DPS;
	o->gy_dps = be16(&b[10]) / IMU_GYRO_LSB_PER_DPS;
	o->gz_dps = be16(&b[12]) / IMU_GYRO_LSB_PER_DPS;
	return true;
}
