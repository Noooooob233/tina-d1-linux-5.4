// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 SiFive
 */

#include <asm/pgtable.h>
#include <asm/cacheflush.h>

#ifdef CONFIG_SMP

#include <asm/sbi.h>

void flush_icache_all(void)
{
	sbi_remote_fence_i(NULL);
}
EXPORT_SYMBOL(flush_icache_all);

/*
 * Performs an icache flush for the given MM context.  RISC-V has no direct
 * mechanism for instruction cache shoot downs, so instead we send an IPI that
 * informs the remote harts they need to flush their local instruction caches.
 * To avoid pathologically slow behavior in a common case (a bunch of
 * single-hart processes on a many-hart machine, ie 'make -j') we avoid the
 * IPIs for harts that are not currently executing a MM context and instead
 * schedule a deferred local instruction cache flush to be performed before
 * execution resumes on each hart.
 */
void flush_icache_mm(struct mm_struct *mm, bool local)
{
	unsigned int cpu;
	cpumask_t others, hmask, *mask;

	preempt_disable();

	/* Mark every hart's icache as needing a flush for this MM. */
	mask = &mm->context.icache_stale_mask;
	cpumask_setall(mask);
	/* Flush this hart's I$ now, and mark it as flushed. */
	cpu = smp_processor_id();
	cpumask_clear_cpu(cpu, mask);
	local_flush_icache_all();

	/*
	 * Flush the I$ of other harts concurrently executing, and mark them as
	 * flushed.
	 */
	cpumask_andnot(&others, mm_cpumask(mm), cpumask_of(cpu));
	local |= cpumask_empty(&others);
	if (mm != current->active_mm || !local) {
		riscv_cpuid_to_hartid_mask(&others, &hmask);
		sbi_remote_fence_i(hmask.bits);
	} else {
		/*
		 * It's assumed that at least one strongly ordered operation is
		 * performed on this hart between setting a hart's cpumask bit
		 * and scheduling this MM context on that hart.  Sending an SBI
		 * remote message will do this, but in the case where no
		 * messages are sent we still need to order this hart's writes
		 * with flush_icache_deferred().
		 */
		smp_mb();
	}

	preempt_enable();
}

#endif /* CONFIG_SMP */

void flush_icache_pte(pte_t pte)
{
	struct page *page = pte_page(pte);

	if (!test_and_set_bit(PG_dcache_clean, &page->flags))
		flush_icache_all();
}

void dma_wbinv_range(unsigned long start, unsigned long end)
{
	register unsigned long i asm("a0") = start & ~(L1_CACHE_BYTES - 1);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x02b5000b"); /* dcache.cipa a0 */

	asm volatile (".long 0x01b0000b");
}

void dma_wb_range(unsigned long start, unsigned long end)
{
	register unsigned long i asm("a0") = start & ~(L1_CACHE_BYTES - 1);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x0295000b"); /* dcache.cpa a0 */

	asm volatile (".long 0x01b0000b");
}

void dma_usr_va_wb_range(void *user_addr, unsigned long len)
{
	unsigned long start = (unsigned long)user_addr;
	unsigned long end = start + len;
	register unsigned long i asm("a5") = start & ~(L1_CACHE_BYTES - 1);

	csr_set(CSR_SSTATUS, SR_SUM);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile(".long 0x0257800b"); /* dcache.cva a5 */

	asm volatile(".long 0x01b0000b");

	csr_clear(CSR_SSTATUS, SR_SUM);
}

void dma_usr_va_inv_range(void *user_addr, unsigned long len)
{
	unsigned long start = (unsigned long)user_addr;
	unsigned long end = start + len;
	register unsigned long i asm("a5") = start & ~(L1_CACHE_BYTES - 1);

	csr_set(CSR_SSTATUS, SR_SUM);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x0267800b"); /* dcache.iva a5 */

	asm volatile(".long 0x01b0000b");

	csr_clear(CSR_SSTATUS, SR_SUM);
}

void dma_va_wb_range(void *kernel_addr, unsigned long len)
{
	unsigned long start = (unsigned long)kernel_addr;
	unsigned long end = start + len;
	register unsigned long i asm("a5") = start & ~(L1_CACHE_BYTES - 1);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile(".long 0x0257800b"); /* dcache.cva a5 */

	asm volatile(".long 0x01b0000b");
}

void dma_va_inv_range(void *kernel_addr, unsigned long len)
{
	unsigned long start = (unsigned long)kernel_addr;
	unsigned long end = start + len;
	register unsigned long i asm("a5") = start & ~(L1_CACHE_BYTES - 1);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x0267800b"); /* dcache.iva a5 */

	asm volatile(".long 0x01b0000b");
}

void dma_va_wbinv_range(void *kernel_addr, unsigned long len)
{
	unsigned long start = (unsigned long)kernel_addr;
	unsigned long end = start + len;
	register unsigned long i asm("a5") = start & ~(L1_CACHE_BYTES - 1);

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x0277800b"); /* dcache.civa a5 */

	asm volatile(".long 0x01b0000b");
}

void dma_clean_dcache_all(void)
{
	asm volatile(".long 0x0010000b":::"memory") ;
}
