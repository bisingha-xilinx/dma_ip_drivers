// SPDX-License-Identifier: GPL-2.0
/*
 * Host glue for the RAVE2 PF0 native HDMA endpoint.
 *
 * The endpoint is 10ee:9038 with subsystem 0007. Its DesignWare VSEC
 * (ID 6, revision 0, length 0x18) describes native HDMA in BAR0 at
 * offset 0. The stock dw-edma-pcie driver does not bind this vendor
 * and maps linked lists through the Synopsys EDDA BAR2 layout.
 *
 * This module maps only the register BAR, keeps linked lists in host
 * memory, and calls dw_edma_probe(). If the channel-enable register
 * still reads as all ones, it stops before that call so the core does
 * not write the aperture.
 */

#include <linux/bitfield.h>
#include <linux/dma/edma.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/pci.h>

#define RAVE2_HDMA_VSEC_ID		0x6
#define RAVE2_HDMA_VSEC_REV		0
#define RAVE2_HDMA_VSEC_LEN		0x18
#define RAVE2_HDMA_MAP_MASK		GENMASK(2, 0)
#define RAVE2_HDMA_BAR_MASK		GENMASK(10, 8)
#define RAVE2_HDMA_WR_CH_MASK		GENMASK(9, 0)
#define RAVE2_HDMA_RD_CH_MASK		GENMASK(25, 16)
#define RAVE2_HDMA_LL_SIZE		PAGE_SIZE

static const struct pci_device_id rave2_hdma_ids[] = {
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_XILINX, 0x9038,
			 PCI_VENDOR_ID_XILINX, 0x0007) },
	{ }
};
MODULE_DEVICE_TABLE(pci, rave2_hdma_ids);

static int rave2_hdma_irq_vector(struct device *dev, unsigned int nr)
{
	return pci_irq_vector(to_pci_dev(dev), nr);
}

static u64 rave2_hdma_pci_address(struct device *dev, phys_addr_t cpu_addr)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct resource res = {
		.flags = IORESOURCE_MEM,
		.start = cpu_addr,
		.end = cpu_addr,
	};
	struct pci_bus_region region;

	pcibios_resource_to_bus(pdev->bus, &region, &res);
	return region.start;
}

static const struct dw_edma_plat_ops rave2_hdma_ops = {
	.irq_vector = rave2_hdma_irq_vector,
	.pci_address = rave2_hdma_pci_address,
};

static int rave2_hdma_find_vsec(struct pci_dev *pdev, u16 *pos)
{
	u16 vsec = 0;
	u32 header;

	while ((vsec = pci_find_next_ext_capability(pdev, vsec,
						    PCI_EXT_CAP_ID_VNDR))) {
		if (pci_read_config_dword(pdev, vsec + PCI_VNDR_HEADER,
					  &header))
			return -EIO;

		if (PCI_VNDR_HEADER_ID(header) == RAVE2_HDMA_VSEC_ID &&
		    PCI_VNDR_HEADER_REV(header) == RAVE2_HDMA_VSEC_REV &&
		    PCI_VNDR_HEADER_LEN(header) == RAVE2_HDMA_VSEC_LEN) {
			*pos = vsec;
			return 0;
		}
	}

	return -ENODEV;
}

static int rave2_hdma_alloc_ll(struct pci_dev *pdev, struct dw_edma_chip *chip)
{
	struct dw_edma_region *region;
	dma_addr_t dma;
	int i;

	for (i = 0; i < chip->ll_wr_cnt; i++) {
		region = &chip->ll_region_wr[i];
		region->sz = RAVE2_HDMA_LL_SIZE;
		region->vaddr.mem = dmam_alloc_coherent(&pdev->dev, region->sz,
							&dma, GFP_KERNEL);
		if (!region->vaddr.mem)
			return -ENOMEM;
		region->paddr = dma;
	}

	for (i = 0; i < chip->ll_rd_cnt; i++) {
		region = &chip->ll_region_rd[i];
		region->sz = RAVE2_HDMA_LL_SIZE;
		region->vaddr.mem = dmam_alloc_coherent(&pdev->dev, region->sz,
							&dma, GFP_KERNEL);
		if (!region->vaddr.mem)
			return -ENOMEM;
		region->paddr = dma;
	}

	return 0;
}

static int rave2_hdma_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct dw_edma_chip *chip;
	void __iomem *regs;
	u32 map, channels, lo, hi, first;
	u16 vsec, wr, rd;
	u64 offset;
	int err, nr_irqs;

	err = pcim_enable_device(pdev);
	if (err)
		return err;

	err = rave2_hdma_find_vsec(pdev, &vsec);
	if (err) {
		pci_err(pdev, "native HDMA VSEC ID %u not found\n",
			RAVE2_HDMA_VSEC_ID);
		return err;
	}

	err = pci_read_config_dword(pdev, vsec + 0x8, &map);
	if (err)
		return -EIO;
	if (FIELD_GET(RAVE2_HDMA_MAP_MASK, map) != EDMA_MF_HDMA_NATIVE ||
	    FIELD_GET(RAVE2_HDMA_BAR_MASK, map) != 0) {
		pci_err(pdev, "unexpected HDMA map dword %#x\n", map);
		return -ENODEV;
	}

	err = pci_read_config_dword(pdev, vsec + 0xc, &channels);
	if (err)
		return -EIO;
	wr = FIELD_GET(RAVE2_HDMA_WR_CH_MASK, channels);
	rd = FIELD_GET(RAVE2_HDMA_RD_CH_MASK, channels);
	if (!wr || wr > EDMA_MAX_WR_CH || !rd || rd > EDMA_MAX_RD_CH) {
		pci_err(pdev, "unsupported channel counts wr=%u rd=%u\n",
			wr, rd);
		return -ENODEV;
	}

	err = pci_read_config_dword(pdev, vsec + 0x10, &lo);
	if (err)
		return -EIO;
	err = pci_read_config_dword(pdev, vsec + 0x14, &hi);
	if (err)
		return -EIO;
	offset = ((u64)hi << 32) | lo;
	if (offset > pci_resource_len(pdev, 0) - 4) {
		pci_err(pdev, "HDMA register offset %#llx is outside BAR0\n",
			offset);
		return -ENODEV;
	}

	err = pcim_iomap_regions(pdev, BIT(0), pci_name(pdev));
	if (err) {
		pci_err(pdev, "BAR0 mapping failed\n");
		return err;
	}
	regs = pcim_iomap_table(pdev)[0];
	if (!regs)
		return -ENOMEM;
	regs += offset;

	first = readl(regs);
	pci_info(pdev,
		 "HDMA native BAR0+%#llx channels wr=%u rd=%u ch_en=%#x\n",
		 offset, wr, rd, first);
	if (first == 0xffffffff) {
		pci_err(pdev,
			"HDMA channel enable reads as all ones; not programming the BAR\n");
		return -ENODEV;
	}

	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err)
		return err;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->mf = EDMA_MF_HDMA_NATIVE;
	chip->ops = &rave2_hdma_ops;
	chip->reg_base = regs;
	chip->ll_wr_cnt = wr;
	chip->ll_rd_cnt = rd;

	err = rave2_hdma_alloc_ll(pdev, chip);
	if (err)
		return err;

	nr_irqs = pci_alloc_irq_vectors(pdev, 1, 7, PCI_IRQ_MSIX | PCI_IRQ_MSI);
	if (nr_irqs < 1) {
		pci_err(pdev, "interrupt allocation failed\n");
		return nr_irqs ? nr_irqs : -EPERM;
	}
	chip->nr_irqs = nr_irqs;

	pci_set_master(pdev);
	err = dw_edma_probe(chip);
	if (err) {
		pci_err(pdev, "dw_edma_probe failed: %d\n", err);
		pci_free_irq_vectors(pdev);
		return err;
	}

	pci_set_drvdata(pdev, chip);
	pci_info(pdev, "registered %u write and %u read HDMA channels, irqs=%d\n",
		 wr, rd, nr_irqs);
	return 0;
}

static void rave2_hdma_remove(struct pci_dev *pdev)
{
	struct dw_edma_chip *chip = pci_get_drvdata(pdev);

	if (chip)
		dw_edma_remove(chip);
	pci_free_irq_vectors(pdev);
	pci_clear_master(pdev);
}

static struct pci_driver rave2_hdma_driver = {
	.name = "rave2-hdma",
	.id_table = rave2_hdma_ids,
	.probe = rave2_hdma_probe,
	.remove = rave2_hdma_remove,
};

module_pci_driver(rave2_hdma_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RAVE2 PF0 native HDMA glue");
MODULE_AUTHOR("AMD");
