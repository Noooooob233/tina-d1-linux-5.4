// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Hangzhou C-SKY Microsystems co.,ltd.

#include <linux/cache.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-noncoherent.h>
#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/version.h>
#include <asm/cache.h>
#include <linux/dma-iommu.h>

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	void *ptr = page_address(page);

	memset(ptr, 0, size);
	dma_wbinv_range(page_to_phys(page), page_to_phys(page) + size);
}

static inline void cache_op(phys_addr_t paddr, size_t size,
			    void (*fn)(unsigned long start, unsigned long end))
{
	unsigned long start = (unsigned long)paddr;

	fn(start, start + size);
}

void arch_sync_dma_for_device(struct device *dev, phys_addr_t paddr,
			      size_t size, enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		cache_op(paddr, size, dma_wb_range);
		break;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
		cache_op(paddr, size, dma_wbinv_range);
		break;
	default:
		BUG();
	}
}

void arch_sync_dma_for_cpu(struct device *dev, phys_addr_t paddr,
			   size_t size, enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		return;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
		cache_op(paddr, size, dma_wbinv_range);
		break;
	default:
		BUG();
	}
}

pgprot_t arch_dma_mmap_pgprot(struct device *dev, pgprot_t prot,
		unsigned long attrs)
{
	if (attrs & DMA_ATTR_WRITE_COMBINE)
		return pgprot_writecombine(prot);
	return pgprot_noncached(prot);
}

void arch_setup_dma_ops(struct device *dev, u64 dma_base, u64 size,
			const struct iommu_ops *iommu, bool coherent)
{
	dev->dma_coherent = coherent;
	if (iommu)
		iommu_setup_dma_ops(dev, dma_base, size);

}
EXPORT_SYMBOL_GPL(arch_setup_dma_ops);
