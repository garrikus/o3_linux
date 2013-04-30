/*
 * SDRC register values for the Micron MT46H64M32LFCM-5
 *
 *
 *
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H64M32LFCM
#define ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H64M32LFCM

#include <plat/sdrc.h>

/* Micron MT46H64M32LFCM-5 */
/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
static struct omap_sdrc_params mt46h64m32lfcm5_sdrc_params[] = {
	[0] = {
		.rate	     = 200000000,
		.actim_ctrla = 0x7ae1b4c6,
		.actim_ctrlb = 0x00021217,
		.rfr_ctrl    = 0x0005e601,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 185000000,
		.actim_ctrla = 0x72e1b4c6,
		.actim_ctrlb = 0x00021215,
		.rfr_ctrl    = 0x00057201,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 166000000,
		.actim_ctrla = 0x629db4c6,
		.actim_ctrlb = 0x00011113,
		.rfr_ctrl    = 0x0004e201,
		.mr	     = 0x00000032,
	},
	[3] = {
		.rate	     = 133000000,
		.actim_ctrla = 0x5259b485,
		.actim_ctrlb = 0x0001110f,
		.rfr_ctrl    = 0x0003de01,
		.mr	     = 0x00000032,
	},
	[4] = {
		.rate	     = 0
	},
};

#endif
