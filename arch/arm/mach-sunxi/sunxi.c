/*
 * Device Tree support for Allwinner A1X SoCs
 *
 * Copyright (C) 2012 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset/sunxi.h>

#include <asm/mach/arch.h>
#include <asm/secure_cntvoff.h>
#include <asm/mach/map.h>
#include <asm/mcpm.h>

#include "sunxi.h"

static void __init sunxi_dt_cpufreq_init(void)
{
	platform_device_register_simple("cpufreq-dt", -1, NULL, 0);
}

static const char * const sunxi_board_dt_compat[] = {
	"allwinner,sun4i-a10",
	"allwinner,sun5i-a10s",
	"allwinner,sun5i-a13",
	"allwinner,sun5i-r8",
	"nextthing,gr8",
	NULL,
};

DT_MACHINE_START(SUNXI_DT, "Allwinner sun4i/sun5i Families")
	.dt_compat	= sunxi_board_dt_compat,
	.init_late	= sunxi_dt_cpufreq_init,
MACHINE_END

static const char * const sun6i_board_dt_compat[] = {
	"allwinner,sun6i-a31",
	"allwinner,sun6i-a31s",
	NULL,
};

static void __init sun6i_timer_init(void)
{
	of_clk_init(NULL);
	if (IS_ENABLED(CONFIG_RESET_CONTROLLER))
		sun6i_reset_init();
	timer_probe();
}

DT_MACHINE_START(SUN6I_DT, "Allwinner sun6i (A31) Family")
	.init_time	= sun6i_timer_init,
	.dt_compat	= sun6i_board_dt_compat,
	.init_late	= sunxi_dt_cpufreq_init,
MACHINE_END

static const char * const sun7i_board_dt_compat[] = {
	"allwinner,sun7i-a20",
	NULL,
};

DT_MACHINE_START(SUN7I_DT, "Allwinner sun7i (A20) Family")
	.dt_compat	= sun7i_board_dt_compat,
	.init_late	= sunxi_dt_cpufreq_init,
MACHINE_END

static struct map_desc sunxi_io_desc[] __initdata = {
	{
		.virtual = (unsigned long) UARTIO_ADDRESS(SUNXI_UART_PBASE),
		.pfn     = __phys_to_pfn(SUNXI_UART_PBASE),
		.length  = SUNXI_UART_SIZE,
		.type    = MT_DEVICE,
	},
};

void __init sunxi_map_io(void)
{
	iotable_init(sunxi_io_desc, ARRAY_SIZE(sunxi_io_desc));
}


static struct platform_device sunxi_cpuidle = {
	.name = "sunxi_cpuidle",
};

static void __init sunxi_init_late(void)
{
	if (of_machine_is_compatible("allwinner,sun8iw15p1"))
		platform_device_register(&sunxi_cpuidle);
}

static const char * const sun8i_board_dt_compat[] = {
	"allwinner,sun8i-a23",
	"allwinner,sun8i-a33",
	"allwinner,sun8i-h2-plus",
	"allwinner,sun8i-h3",
	"allwinner,sun8i-r40",
	"allwinner,sun8i-v3s",
	"allwinner,sun8iw15p1",
	"allwinner,sun8iw20p1",
	NULL,
};

DT_MACHINE_START(SUN8I_DT, CONFIG_SUNXI_SOC_NAME)
	.init_time	= sun6i_timer_init,
	.map_io		= sunxi_map_io,
	.init_late	= sunxi_init_late,
	.dt_compat	= sun8i_board_dt_compat,
MACHINE_END

static void __init sun8i_a83t_cntvoff_init(void)
{
#ifdef CONFIG_SMP
	secure_cntvoff_init();
#endif
}

static const char * const sun8i_a83t_cntvoff_board_dt_compat[] = {
	"allwinner,sun8i-a83t",
	NULL,
};

DT_MACHINE_START(SUN8I_A83T_CNTVOFF_DT, "Allwinner A83t board")
	.init_early	= sun8i_a83t_cntvoff_init,
	.init_time	= sun6i_timer_init,
	.dt_compat	= sun8i_a83t_cntvoff_board_dt_compat,
MACHINE_END

static const char * const sun9i_board_dt_compat[] = {
	"allwinner,sun9i-a80",
	NULL,
};

DT_MACHINE_START(SUN9I_DT, "Allwinner sun9i Family")
	.dt_compat	= sun9i_board_dt_compat,
MACHINE_END

static const char * const suniv_board_dt_compat[] = {
	"allwinner,suniv-f1c100s",
	NULL,
};

DT_MACHINE_START(SUNIV_DT, "Allwinner suniv Family")
	.dt_compat	= suniv_board_dt_compat,
MACHINE_END
