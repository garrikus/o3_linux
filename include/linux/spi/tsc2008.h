#ifndef __LINUX_SPI_TSC2008_H
#define __LINUX_SPI_TSC2008_H

/* linux/spi/tsc2008.h */

struct tsc2008_platform_data {
	u16	model;				/* 2008. */
	u16	x_plate_ohms;
	u16	bit_mode;			/* 8 (default) or 12 */

	int	(*get_pendown_state)(void);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   interrupt source */
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);

    unsigned long irq_flags;
};

#endif
