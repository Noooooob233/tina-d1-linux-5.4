// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 *
 * All RISC-V systems have a timer attached to every hart.  These timers can be
 * read from the "time" and "timeh" CSRs, and can use the SBI to setup
 * events.
 */
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/sched_clock.h>
#include <asm/smp.h>
#include <asm/sbi.h>
#include <linux/cpu_pm.h>

static int riscv_clock_next_event(unsigned long delta,
		struct clock_event_device *ce)
{
	csr_set(sie, SIE_STIE);
	sbi_set_timer(get_cycles64() + delta);
	return 0;
}

static DEFINE_PER_CPU(struct clock_event_device, riscv_clock_event) = {
	.name			= "riscv_timer_clockevent",
	.features		= CLOCK_EVT_FEAT_ONESHOT|CLOCK_EVT_FEAT_C3STOP,
	.rating			= 400,
	.set_next_event		= riscv_clock_next_event,
};

/*
 * It is guaranteed that all the timers across all the harts are synchronized
 * within one tick of each other, so while this could technically go
 * backwards when hopping between CPUs, practically it won't happen.
 */
static unsigned long long riscv_clocksource_rdtime(struct clocksource *cs)
{
	return get_cycles64();
}

static u64 notrace riscv_sched_clock(void)
{
	return get_cycles64();
}

static struct clocksource riscv_clocksource = {
	.name		= "riscv_clocksource",
	.rating		= 400,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= riscv_clocksource_rdtime,
};

static int riscv_timer_starting_cpu(unsigned int cpu)
{
	struct clock_event_device *ce = per_cpu_ptr(&riscv_clock_event, cpu);

	ce->cpumask = cpumask_of(cpu);
	clockevents_config_and_register(ce, riscv_timebase, 100, 0x7fffffff);

	csr_set(sie, SIE_STIE);
	return 0;
}

static int riscv_timer_dying_cpu(unsigned int cpu)
{
	csr_clear(sie, SIE_STIE);
	return 0;
}

/* called directly from the low-level interrupt handler */
void riscv_timer_interrupt(void)
{
	struct clock_event_device *evdev = this_cpu_ptr(&riscv_clock_event);

	csr_clear(sie, SIE_STIE);
	evdev->event_handler(evdev);
}

#ifdef CONFIG_CPU_PM
static DEFINE_PER_CPU(unsigned long, saved_sie_stie);
static int riscv_timer_cpu_pm_notify(struct notifier_block *self,
				    unsigned long action, void *hcpu)
{
	if (action == CPU_PM_ENTER) {
		__this_cpu_write(saved_sie_stie, csr_read(sie));
		csr_clear(sie, SIE_STIE);
	} else if (action == CPU_PM_ENTER_FAILED || action == CPU_PM_EXIT) {
		csr_write(sie, saved_sie_stie);
	}
	return NOTIFY_OK;
}

static struct notifier_block riscv_timer_cpu_pm_notifier = {
	.notifier_call = riscv_timer_cpu_pm_notify,
};

static int __init riscv_timer_cpu_pm_init(void)
{
	return cpu_pm_register_notifier(&riscv_timer_cpu_pm_notifier);
}
#else
static int __init riscv_timer_cpu_pm_init(void)
{
	return 0;
}
#endif

static int __init riscv_timer_init_dt(struct device_node *n)
{
	int cpuid, hartid, error;

	hartid = riscv_of_processor_hartid(n);
	if (hartid < 0) {
		pr_warn("Not valid hartid for node [%pOF] error = [%d]\n",
			n, hartid);
		return hartid;
	}

	cpuid = riscv_hartid_to_cpuid(hartid);
	if (cpuid < 0) {
		pr_warn("Invalid cpuid for hartid [%d]\n", hartid);
		return cpuid;
	}

	if (cpuid != smp_processor_id())
		return 0;

	pr_info("%s: Registering clocksource cpuid [%d] hartid [%d]\n",
	       __func__, cpuid, hartid);
	error = clocksource_register_hz(&riscv_clocksource, riscv_timebase);
	if (error) {
		pr_err("RISCV timer register failed [%d] for cpu = [%d]\n",
		       error, cpuid);
		return error;
	}

	sched_clock_register(riscv_sched_clock, 64, riscv_timebase);

	error = cpuhp_setup_state(CPUHP_AP_RISCV_TIMER_STARTING,
			 "clockevents/riscv/timer:starting",
			 riscv_timer_starting_cpu, riscv_timer_dying_cpu);
	if (error)
		pr_err("cpu hp setup state failed for RISCV timer [%d]\n",
		       error);
	error = riscv_timer_cpu_pm_init();
	if (error)
		pr_err("failed to register RISCV timer PM API[%d]\n", error);
	return error;
}

TIMER_OF_DECLARE(riscv_timer, "riscv", riscv_timer_init_dt);
