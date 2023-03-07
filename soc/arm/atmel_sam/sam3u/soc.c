/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAM3U MCU series initialization code
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Atmel SAM3U series processor.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/arch/arm/aarch32/cortex_m/cmsis.h>
#include <zephyr/irq.h>

/*
 * PLL clock = Main * (MULA + 1) / DIVA
 *
 * By default, MULA == 7, DIVA == 1.
 * With main crystal running at 12 MHz,
 * PLL = 12 * (7 + 1) / 1 = 96 MHz
 *
 * With Processor Clock prescaler at 1
 * Processor Clock (HCLK) = 96 MHz.
 */
#define SYS_BOARD_OSCOUNT (CKGR_MOR_MOSCXTST(0x8UL))
#define SYS_BOARD_PLLAR                                                                            \
	(CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(CONFIG_SOC_ATMEL_SAM3U_PLLA_MULA) |                      \
	 CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(CONFIG_SOC_ATMEL_SAM3U_PLLA_DIVA))
#define SYS_BOARD_MCKR (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK)

/**
 * @brief Setup various clocks on SoC at boot time.
 *
 * Setup Slow, Main, PLLA, Processor and Master clocks during the device boot.
 * It is assumed that the relevant registers are at their reset value.
 */
static ALWAYS_INLINE void clock_init(void)
{
	/* Initialize main oscillator */
	if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)) {
		PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN |
				CKGR_MOR_MOSCXTEN;

		while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) {
		}
	}

	/* Switch to 3-20MHz Xtal oscillator */
	PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN |
			CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL;
	while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) {
	}

	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {
	}

	/* Initialize PLLA */
	PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
	while (!(PMC->PMC_SR & PMC_SR_LOCKA)) {
	}

	/* Switch to main clock */
	PMC->PMC_MCKR = (SYS_BOARD_MCKR & ~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {
	}

	/* Switch to PLLA */
	PMC->PMC_MCKR = SYS_BOARD_MCKR;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {
	}
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This has to be run at the very beginning thus the init priority is set at
 * 0 (zero).
 *
 * @return 0
 */
static int atmel_sam3u_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/*
	 * Set FWS (Flash Wait State) value before increasing Master Clock
	 * (MCK) frequency.
	 * TODO: set FWS based on the actual MCK frequency and VDDCORE value
	 * rather than maximum supported 84 MHz at standard VDDCORE=1.8V
	 */
	EFC0->EEFC_FMR = EEFC_FMR_FWS(4);
#if IS_ENABLED(CONFIG_SOC_PART_NUMBER_SAM3U4C) || IS_ENABLED(CONFIG_SOC_PART_NUMBER_SAM3U4E)
	EFC1->EEFC_FMR = EEFC_FMR_FWS(4);
#endif

	/* Setup system clocks */
	clock_init();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_sam3u_init, PRE_KERNEL_1, 0);
