/*
 * TI DaVinci (TMS320DM644x) I2C driver.
 *
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 *
 * --------------------------------------------------------
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <i2c.h>
#include <asm/arch/hardware.h>
#include <asm/arch/i2c_defs.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

static struct i2c_regs __attribute__((section (".data"))) *i2c_base =
						(struct i2c_regs *)I2C_BASE;

static unsigned int __attribute__((section (".data"))) bus_initialized[I2C_BUS_MAX] =
					{ [0 ... (I2C_BUS_MAX-1)] = 0 };
static unsigned int __attribute__((section (".data"))) current_bus = 0;


#define CHECK_NACK() \
	do {\
		if (tmp & (I2C_TIMEOUT | I2C_STAT_NACK)) {\
			REG(&(i2c_base->i2c_con)) = 0;\
			return(1);\
		}\
	} while (0)


static int wait_for_bus(void)
{
	int	stat, timeout;

	REG(&(i2c_base->i2c_stat)) = 0xffff;

	for (timeout = 0; timeout < 10; timeout++) {
		if (!((stat = REG(&(i2c_base->i2c_stat))) & I2C_STAT_BB)) {
			REG(&(i2c_base->i2c_stat)) = 0xffff;
			return(0);
		}

		REG(&(i2c_base->i2c_stat)) = stat;
		udelay(50000);
	}

	REG(&(i2c_base->i2c_stat)) = 0xffff;
	return(1);
}


static int poll_i2c_irq(int mask)
{
	int	stat, timeout;

	for (timeout = 0; timeout < 10; timeout++) {
		udelay(1000);
		stat = REG(&(i2c_base->i2c_stat));
		if (stat & mask) {
			return(stat);
		}
	}

	REG(&(i2c_base->i2c_stat)) = 0xffff;
	return(stat | I2C_TIMEOUT);
}


void flush_rx(void)
{
	while (1) {
		if (!(REG(&(i2c_base->i2c_stat)) & I2C_STAT_RRDY))
			break;

		REG(&(i2c_base->i2c_drr));
		REG(&(i2c_base->i2c_stat)) = I2C_STAT_RRDY;
		udelay(1000);
	}
}


void i2c_init(int speed, int slaveadd)
{
	u_int32_t	div, psc;

	if (REG(&(i2c_base->i2c_con)) & I2C_CON_EN) {
		REG(&(i2c_base->i2c_con)) = 0;
		udelay (50000);
	}

	psc = 2;
	div = (CONFIG_SYS_HZ_CLOCK / ((psc + 1) * speed)) - 10;	/* SCLL + SCLH */
	REG(&(i2c_base->i2c_psc)) = psc;			/* 27MHz / (2 + 1) = 9MHz */
	REG(&(i2c_base->i2c_scll)) = (div * 50) / 100;	/* 50% Duty */
	REG(&(i2c_base->i2c_sclh)) = div - REG(&(i2c_base->i2c_scll));

	REG(&(i2c_base->i2c_oa)) = slaveadd;
	REG(&(i2c_base->i2c_cnt)) = 0;

	/* Interrupts must be enabled or I2C module won't work */
	REG(&(i2c_base->i2c_ie)) = I2C_IE_SCD_IE | I2C_IE_XRDY_IE |
		I2C_IE_RRDY_IE | I2C_IE_ARDY_IE | I2C_IE_NACK_IE;

	/* Now enable I2C controller (get it out of reset) */
	REG(&(i2c_base->i2c_con)) = I2C_CON_EN;

	udelay(1000);

	if (gd->flags & GD_FLG_RELOC)
		bus_initialized[current_bus] = 1;

}

int i2c_set_bus_speed(unsigned int speed)
{
	i2c_init(speed, CONFIG_SYS_I2C_SLAVE);
	return 0;
}

int i2c_probe(u_int8_t chip)
{
	int	rc = 1;

	if (chip == REG(&(i2c_base->i2c_oa))) {
		return(rc);
	}

	REG(&(i2c_base->i2c_con)) = 0;
	if (wait_for_bus()) {return(1);}

	/* try to read one byte from current (or only) address */
	REG(&(i2c_base->i2c_cnt)) = 1;
	REG(&(i2c_base->i2c_sa))  = chip;
	REG(&(i2c_base->i2c_con)) = (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP);
	udelay (50000);

	if (!(REG(&(i2c_base->i2c_stat)) & I2C_STAT_NACK)) {
		rc = 0;
		flush_rx();
		REG(&(i2c_base->i2c_stat)) = 0xffff;
	} else {
		REG(&(i2c_base->i2c_stat)) = 0xffff;
		REG(&(i2c_base->i2c_con)) |= I2C_CON_STP;
		udelay(20000);
		if (wait_for_bus()) {return(1);}
	}

	flush_rx();
	REG(&(i2c_base->i2c_stat)) = 0xffff;
	REG(&(i2c_base->i2c_cnt)) = 0;
	return(rc);
}


int i2c_read(u_int8_t chip, u_int32_t addr, int alen, u_int8_t *buf, int len)
{
	u_int32_t	tmp;
	int		i;

	if ((alen < 0) || (alen > 2)) {
		printf("%s(): bogus address length %x\n", __FUNCTION__, alen);
		return(1);
	}

	if (wait_for_bus()) {return(1);}

	if (alen != 0) {
		/* Start address phase */
		tmp = I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX;
		REG(&(i2c_base->i2c_cnt)) = alen;
		REG(&(i2c_base->i2c_sa)) = chip;
		REG(&(i2c_base->i2c_con)) = tmp;

		tmp = poll_i2c_irq(I2C_STAT_XRDY | I2C_STAT_NACK);

		CHECK_NACK();

		switch (alen) {
			case 2:
				/* Send address MSByte */
				if (tmp & I2C_STAT_XRDY) {
					REG(&(i2c_base->i2c_dxr)) = (addr >> 8) & 0xff;
				} else {
					REG(&(i2c_base->i2c_con)) = 0;
					return(1);
				}

				tmp = poll_i2c_irq(I2C_STAT_XRDY | I2C_STAT_NACK);

				CHECK_NACK();
				/* No break, fall through */
			case 1:
				/* Send address LSByte */
				if (tmp & I2C_STAT_XRDY) {
					REG(&(i2c_base->i2c_dxr)) = addr & 0xff;
				} else {
					REG(&(i2c_base->i2c_con)) = 0;
					return(1);
				}

				tmp = poll_i2c_irq(I2C_STAT_XRDY | I2C_STAT_NACK | I2C_STAT_ARDY);

				CHECK_NACK();

				if (!(tmp & I2C_STAT_ARDY)) {
					REG(&(i2c_base->i2c_con)) = 0;
					return(1);
				}
		}
	}

	/* Address phase is over, now read 'len' bytes and stop */
	tmp = I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP;
	REG(&(i2c_base->i2c_cnt)) = len & 0xffff;
	REG(&(i2c_base->i2c_sa)) = chip;
	REG(&(i2c_base->i2c_con)) = tmp;

	for (i = 0; i < len; i++) {
		tmp = poll_i2c_irq(I2C_STAT_RRDY | I2C_STAT_NACK | I2C_STAT_ROVR);

		CHECK_NACK();

		if (tmp & I2C_STAT_RRDY) {
			buf[i] = REG(&(i2c_base->i2c_drr));
		} else {
			REG(&(i2c_base->i2c_con)) = 0;
			return(1);
		}
	}

	tmp = poll_i2c_irq(I2C_STAT_SCD | I2C_STAT_NACK);

	CHECK_NACK();

	if (!(tmp & I2C_STAT_SCD)) {
		REG(&(i2c_base->i2c_con)) = 0;
		return(1);
	}

	flush_rx();
	REG(&(i2c_base->i2c_stat)) = 0xffff;
	REG(&(i2c_base->i2c_cnt)) = 0;
	REG(&(i2c_base->i2c_con)) = 0;

	return(0);
}


int i2c_write(u_int8_t chip, u_int32_t addr, int alen, u_int8_t *buf, int len)
{
	u_int32_t	tmp;
	int		i;

	if ((alen < 0) || (alen > 2)) {
		printf("%s(): bogus address length %x\n", __FUNCTION__, alen);
		return(1);
	}
	if (len < 0) {
		printf("%s(): bogus length %x\n", __FUNCTION__, len);
		return(1);
	}

	if (wait_for_bus()) {return(1);}

	/* Start address phase */
	tmp = I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX | I2C_CON_STP;
	REG(&(i2c_base->i2c_cnt)) = (alen == 0) ? len & 0xffff : (len & 0xffff) + alen;
	REG(&(i2c_base->i2c_sa)) = chip;
	REG(&(i2c_base->i2c_con)) = tmp;

	switch (alen) {
		case 2:
			/* Send address MSByte */
			tmp = poll_i2c_irq(I2C_STAT_XRDY | I2C_STAT_NACK);

			CHECK_NACK();

			if (tmp & I2C_STAT_XRDY) {
				REG(&(i2c_base->i2c_dxr)) = (addr >> 8) & 0xff;
			} else {
				REG(&(i2c_base->i2c_con)) = 0;
				return(1);
			}
			/* No break, fall through */
		case 1:
			/* Send address LSByte */
			tmp = poll_i2c_irq(I2C_STAT_XRDY | I2C_STAT_NACK);

			CHECK_NACK();

			if (tmp & I2C_STAT_XRDY) {
				REG(&(i2c_base->i2c_dxr)) = addr & 0xff;
			} else {
				REG(&(i2c_base->i2c_con)) = 0;
				return(1);
			}
	}

	for (i = 0; i < len; i++) {
		tmp = poll_i2c_irq(I2C_STAT_XRDY | I2C_STAT_NACK);

		CHECK_NACK();

		if (tmp & I2C_STAT_XRDY) {
			REG(&(i2c_base->i2c_dxr)) = buf[i];
		} else {
			return(1);
		}
	}

	tmp = poll_i2c_irq(I2C_STAT_SCD | I2C_STAT_NACK);

	CHECK_NACK();

	if (!(tmp & I2C_STAT_SCD)) {
		REG(&(i2c_base->i2c_con)) = 0;
		return(1);
	}

	flush_rx();
	REG(&(i2c_base->i2c_stat)) = 0xffff;
	REG(&(i2c_base->i2c_cnt)) = 0;
	REG(&(i2c_base->i2c_con)) = 0;

	return(0);
}

int i2c_set_bus_num(unsigned int bus)
{
	if ((bus < 0) || (bus >= I2C_BUS_MAX)) {
		printf("Bad bus: %d\n", bus);
		return -1;
	}

	switch (bus) {
#if I2C_BUS_MAX == 3
	case 2: i2c_base = (struct i2c_regs *)I2C2_BASE; break;
#endif
#if I2C_BUS_MAX >= 2
	case 1: i2c_base = (struct i2c_regs *)I2C1_BASE; break;
#endif
	default:
		i2c_base = (struct i2c_regs *)I2C0_BASE;
		bus = 0;
	}

	current_bus = bus;

	if (!bus_initialized[current_bus])
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	return 0;
}

unsigned int i2c_get_bus_num(void)
{
	return (int) current_bus;
}
