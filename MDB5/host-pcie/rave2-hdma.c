// SPDX-License-Identifier: GPL-2.0
/*
 * Host glue for the RAVE2 PF0 native HDMA endpoint.
 *
 * The endpoint is 10ee:9038 with subsystem 0007. Its DesignWare VSEC
 * (ID 6, revision 0, length 0x18) describes native HDMA in BAR0 at
 * offset 0. The stock dw-edma-pcie driver does not bind this vendor
 * and maps linked lists through the Synopsys EDDA BAR2 layout.
 *
 * Linked lists live in the last 512 KiB of Versal high DDR. PF0 BAR2
 * translates to that AXI address, and dw-edma programs it as the LLP.
 * The host writes the descriptors through BAR2. If the channel-enable
 * register still reads as all ones, probe stops before dw_edma_probe()
 * so the core does not write the aperture.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma/edma.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>

#define RAVE2_HDMA_VSEC_ID		0x6
#define RAVE2_HDMA_VSEC_REV		0
#define RAVE2_HDMA_VSEC_LEN		0x18
#define RAVE2_HDMA_MAP_MASK		GENMASK(2, 0)
#define RAVE2_HDMA_BAR_MASK		GENMASK(10, 8)
#define RAVE2_HDMA_WR_CH_MASK		GENMASK(9, 0)
#define RAVE2_HDMA_RD_CH_MASK		GENMASK(25, 16)
#define RAVE2_HDMA_LL_SIZE		PAGE_SIZE
/* Last 512 KiB of the PCIe high-DDR window. Matches bd_rave2.tcl. */
#define RAVE2_HDMA_LL_AXI		0xb7ff80000ULL
#define RAVE2_HDMA_CH_EN		0x00
#define RAVE2_HDMA_DOORBELL		0x04
#define RAVE2_HDMA_LLP_LO		0x10
#define RAVE2_HDMA_LLP_HI		0x14
#define RAVE2_HDMA_CYCLE		0x18
#define RAVE2_HDMA_SIZE			0x1c
#define RAVE2_HDMA_SAR_LO		0x20
#define RAVE2_HDMA_SAR_HI		0x24
#define RAVE2_HDMA_DAR_LO		0x28
#define RAVE2_HDMA_DAR_HI		0x2c
#define RAVE2_HDMA_CONTROL1		0x34
#define RAVE2_HDMA_CH_STAT		0x80
#define RAVE2_HDMA_INT_STAT		0x84
#define RAVE2_HDMA_INT_SETUP		0x88
#define RAVE2_HDMA_INT_CLEAR		0x8c
#define RAVE2_HDMA_INT_MASK		(BIT(0) | BIT(2))
#define RAVE2_HDMA_RD_CH		0x100

static u64 rave2_ep_addr = 0x97fff0000ULL;
module_param_named(ep_addr, rave2_ep_addr, ullong, 0644);
MODULE_PARM_DESC(ep_addr, "Versal AXI address read by the HDMA self-test");

static DEFINE_MUTEX(rave2_selftest_lock);

static const struct pci_device_id rave2_hdma_ids[] = {
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_XILINX, 0x9038,
			 PCI_VENDOR_ID_XILINX, 0x0007) },
	{ }
};
MODULE_DEVICE_TABLE(pci, rave2_hdma_ids);

static ssize_t rave2_ll_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct dw_edma_chip *chip = pci_get_drvdata(to_pci_dev(dev));
	struct dw_edma_region *regions[2] = {
		&chip->ll_region_wr[0],
		&chip->ll_region_rd[0],
	};
	static const char * const names[] = { "write0", "read0" };
	ssize_t len = 0;
	int i;

	for (i = 0; i < 2; i++) {
		void __iomem *word = regions[i]->vaddr.io;
		u64 sar, dar, llp;

		if (!word)
			continue;

		sar = ((u64)readl(word + 12) << 32) | readl(word + 8);
		dar = ((u64)readl(word + 20) << 32) | readl(word + 16);
		llp = ((u64)readl(word + 36) << 32) | readl(word + 32);
		len += sysfs_emit_at(buf, len, "%s paddr=%#llx\n",
				     names[i], regions[i]->paddr);
		len += sysfs_emit_at(buf, len,
				     "elem0 control=%#x size=%#x sar=%#llx dar=%#llx\n",
				     readl(word + 0), readl(word + 4), sar, dar);
		len += sysfs_emit_at(buf, len,
				     "elem1 control=%#x reserved=%#x llp=%#llx\n",
				     readl(word + 24), readl(word + 28), llp);
	}

	return len;
}
static DEVICE_ATTR_RO(rave2_ll);

static ssize_t selftest_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dw_edma_chip *chip = pci_get_drvdata(pdev);
	void __iomem *regs = chip->reg_base;
	u32 *data, stat = 0, intr, saved, size;
	dma_addr_t dma;
	ssize_t len;
	int ret;

	if (!mutex_trylock(&rave2_selftest_lock))
		return -EBUSY;

	data = dma_alloc_coherent(&pdev->dev, 4, &dma, GFP_KERNEL);
	if (!data) {
		mutex_unlock(&rave2_selftest_lock);
		return -ENOMEM;
	}
	*data = 0x11111111;

	saved = readl(regs + RAVE2_HDMA_INT_SETUP);
	writel(saved | RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_SETUP);
	writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_CLEAR);
	writel(0, regs + RAVE2_HDMA_CH_EN);
	writel(0, regs + RAVE2_HDMA_CONTROL1);
	writel(lower_32_bits(rave2_ep_addr), regs + RAVE2_HDMA_SAR_LO);
	writel(upper_32_bits(rave2_ep_addr), regs + RAVE2_HDMA_SAR_HI);
	writel(lower_32_bits(dma), regs + RAVE2_HDMA_DAR_LO);
	writel(upper_32_bits(dma), regs + RAVE2_HDMA_DAR_HI);
	writel(4, regs + RAVE2_HDMA_SIZE);
	writel(1, regs + RAVE2_HDMA_CH_EN);
	writel(1, regs + RAVE2_HDMA_DOORBELL);

	ret = readl_poll_timeout(regs + RAVE2_HDMA_CH_STAT, stat,
				 (stat & 3) != 1, 10, 100000);
	intr = readl(regs + RAVE2_HDMA_INT_STAT);
	size = readl(regs + RAVE2_HDMA_SIZE);
	writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_CLEAR);
	writel(0, regs + RAVE2_HDMA_CH_EN);
	writel(saved, regs + RAVE2_HDMA_INT_SETUP);

	len = sysfs_emit(buf,
			 "simple ep=%#llx dar=%#llx word=%#x stat=%#x int=%#x size=%#x poll=%d\n",
			 rave2_ep_addr, (u64)dma, *data, stat, intr, size, ret);
	dma_free_coherent(&pdev->dev, 4, data, dma);

	data = dma_alloc_coherent(&pdev->dev, 4, &dma, GFP_KERNEL);
	if (data) {
		u32 *list;
		dma_addr_t list_dma;

		*data = 0x11111111;
		list = dma_alloc_coherent(&pdev->dev, 64, &list_dma, GFP_KERNEL);
		if (!list) {
			dma_free_coherent(&pdev->dev, 4, data, dma);
		} else {
			list[0] = 1;
			list[1] = 4;
			list[2] = lower_32_bits(rave2_ep_addr);
			list[3] = upper_32_bits(rave2_ep_addr);
			list[4] = lower_32_bits(dma);
			list[5] = upper_32_bits(dma);
			list[6] = 6;
			list[7] = 0;
			list[8] = lower_32_bits(list_dma);
			list[9] = upper_32_bits(list_dma);
			dma_wmb();

			writel(saved | RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_SETUP);
			writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_CLEAR);
			writel(0, regs + RAVE2_HDMA_CH_EN);
			writel(1, regs + RAVE2_HDMA_CONTROL1);
			writel(lower_32_bits(list_dma), regs + RAVE2_HDMA_LLP_LO);
			writel(upper_32_bits(list_dma), regs + RAVE2_HDMA_LLP_HI);
			writel(3, regs + RAVE2_HDMA_CYCLE);
			writel(1, regs + RAVE2_HDMA_CH_EN);
			writel(1, regs + RAVE2_HDMA_DOORBELL);

			stat = 0;
			ret = readl_poll_timeout(regs + RAVE2_HDMA_CH_STAT, stat,
						 (stat & 3) != 1, 10, 100000);
			intr = readl(regs + RAVE2_HDMA_INT_STAT);
			writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_CLEAR);
			writel(0, regs + RAVE2_HDMA_CH_EN);
			writel(0, regs + RAVE2_HDMA_CONTROL1);
			writel(saved, regs + RAVE2_HDMA_INT_SETUP);

			len += sysfs_emit_at(buf, len,
					     "list ep=%#llx dar=%#llx llp=%#llx word=%#x stat=%#x int=%#x poll=%d\n",
					     rave2_ep_addr, (u64)dma, (u64)list_dma,
					     *data, stat, intr, ret);

			/*
			 * The endpoint fetches the list through its local AXI
			 * port. Stage the same descriptor in Versal DDR and
			 * point the channel at that address.
			 */
			list[8] = lower_32_bits(rave2_ep_addr + 0x1000);
			list[9] = upper_32_bits(rave2_ep_addr + 0x1000);
			dma_wmb();
			/*
			 * Read channel: source is host memory, destination is
			 * Versal DDR. The write channel cannot make that copy.
			 */
			writel(readl(regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_INT_SETUP) |
			       RAVE2_HDMA_INT_MASK,
			       regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_INT_SETUP);
			writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_INT_CLEAR);
			writel(0, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_CH_EN);
			writel(0, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_CONTROL1);
			writel(lower_32_bits(list_dma), regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_SAR_LO);
			writel(upper_32_bits(list_dma), regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_SAR_HI);
			writel(list[8], regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_DAR_LO);
			writel(list[9], regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_DAR_HI);
			writel(64, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_SIZE);
			writel(1, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_CH_EN);
			writel(1, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_DOORBELL);
			stat = 0;
			ret = readl_poll_timeout(regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_CH_STAT,
						 stat, (stat & 3) != 1, 10, 100000);
			writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_INT_CLEAR);
			writel(0, regs + RAVE2_HDMA_RD_CH + RAVE2_HDMA_CH_EN);
			len += sysfs_emit_at(buf, len, "copy stat=%#x poll=%d\n", stat, ret);

			*data = 0x11111111;
			writel(lower_32_bits(rave2_ep_addr + 0x1000), regs + RAVE2_HDMA_SAR_LO);
			writel(upper_32_bits(rave2_ep_addr + 0x1000), regs + RAVE2_HDMA_SAR_HI);
			writel(lower_32_bits(dma), regs + RAVE2_HDMA_DAR_LO);
			writel(upper_32_bits(dma), regs + RAVE2_HDMA_DAR_HI);
			writel(4, regs + RAVE2_HDMA_SIZE);
			writel(1, regs + RAVE2_HDMA_CH_EN);
			writel(1, regs + RAVE2_HDMA_DOORBELL);
			readl_poll_timeout(regs + RAVE2_HDMA_CH_STAT, stat,
					   (stat & 3) != 1, 10, 100000);
			writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_CLEAR);
			writel(0, regs + RAVE2_HDMA_CH_EN);
			len += sysfs_emit_at(buf, len, "staged word=%#x stat=%#x\n",
					     *data, stat);

			*data = 0x11111111;
			writel(1, regs + RAVE2_HDMA_CONTROL1);
			writel(list[8], regs + RAVE2_HDMA_LLP_LO);
			writel(list[9], regs + RAVE2_HDMA_LLP_HI);
			writel(3, regs + RAVE2_HDMA_CYCLE);
			writel(1, regs + RAVE2_HDMA_CH_EN);
			writel(1, regs + RAVE2_HDMA_DOORBELL);
			stat = 0;
			ret = readl_poll_timeout(regs + RAVE2_HDMA_CH_STAT, stat,
						 (stat & 3) != 1, 10, 100000);
			intr = readl(regs + RAVE2_HDMA_INT_STAT);
			writel(RAVE2_HDMA_INT_MASK, regs + RAVE2_HDMA_INT_CLEAR);
			writel(0, regs + RAVE2_HDMA_CH_EN);
			writel(0, regs + RAVE2_HDMA_CONTROL1);
			writel(saved, regs + RAVE2_HDMA_INT_SETUP);
			len += sysfs_emit_at(buf, len,
					     "local llp=%#llx word=%#x stat=%#x int=%#x poll=%d\n",
					     rave2_ep_addr + 0x1000, *data, stat, intr, ret);
			dma_free_coherent(&pdev->dev, 64, list, list_dma);
			dma_free_coherent(&pdev->dev, 4, data, dma);
		}
	}
	mutex_unlock(&rave2_selftest_lock);
	return len;
}
static DEVICE_ATTR_RO(selftest);

static int rave2_hdma_irq_vector(struct device *dev, unsigned int nr)
{
	return pci_irq_vector(to_pci_dev(dev), nr);
}

static u64 rave2_hdma_pci_address(struct device *dev, phys_addr_t cpu_addr)
{
	/*
	 * dmaengine passes the Versal AXI address here. It is already the
	 * address the endpoint HDMA must use. Host buffers arrive separately
	 * as dma_addr_t values and are not translated again.
	 */
	dev_info(dev, "HDMA endpoint address %#llx\n", (u64)cpu_addr);
	return cpu_addr;
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

static int rave2_hdma_map_ll(struct pci_dev *pdev, struct dw_edma_chip *chip,
			    void __iomem *bar2)
{
	struct dw_edma_region *region;
	u64 need;
	int i;

	need = (u64)(chip->ll_wr_cnt + chip->ll_rd_cnt) * RAVE2_HDMA_LL_SIZE;
	if (need > pci_resource_len(pdev, 2)) {
		pci_err(pdev, "BAR2 is %#llx bytes, linked lists need %#llx\n",
			(u64)pci_resource_len(pdev, 2), need);
		return -ENOSPC;
	}

	for (i = 0; i < chip->ll_wr_cnt; i++) {
		region = &chip->ll_region_wr[i];
		region->sz = RAVE2_HDMA_LL_SIZE;
		region->vaddr.io = bar2 + (i * RAVE2_HDMA_LL_SIZE);
		region->paddr = RAVE2_HDMA_LL_AXI + (i * RAVE2_HDMA_LL_SIZE);
	}

	for (i = 0; i < chip->ll_rd_cnt; i++) {
		region = &chip->ll_region_rd[i];
		region->sz = RAVE2_HDMA_LL_SIZE;
		region->vaddr.io = bar2 + ((chip->ll_wr_cnt + i) * RAVE2_HDMA_LL_SIZE);
		region->paddr = RAVE2_HDMA_LL_AXI +
				((chip->ll_wr_cnt + i) * RAVE2_HDMA_LL_SIZE);
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

	err = pcim_iomap_regions(pdev, BIT(0) | BIT(2), pci_name(pdev));
	if (err) {
		pci_err(pdev, "BAR0/BAR2 mapping failed\n");
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

	if (!pcim_iomap_table(pdev)[2])
		return -ENOMEM;
	err = rave2_hdma_map_ll(pdev, chip, pcim_iomap_table(pdev)[2]);
	if (err)
		return err;
	pci_info(pdev, "HDMA linked lists in BAR2, AXI base %#llx\n",
		 RAVE2_HDMA_LL_AXI);

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
	err = device_create_file(&pdev->dev, &dev_attr_rave2_ll);
	if (err) {
		dw_edma_remove(chip);
		pci_free_irq_vectors(pdev);
		return err;
	}
	err = device_create_file(&pdev->dev, &dev_attr_selftest);
	if (err) {
		device_remove_file(&pdev->dev, &dev_attr_rave2_ll);
		dw_edma_remove(chip);
		pci_free_irq_vectors(pdev);
		return err;
	}
	pci_info(pdev, "registered %u write and %u read HDMA channels, irqs=%d\n",
		 wr, rd, nr_irqs);
	return 0;
}

static void rave2_hdma_remove(struct pci_dev *pdev)
{
	struct dw_edma_chip *chip = pci_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_selftest);
	device_remove_file(&pdev->dev, &dev_attr_rave2_ll);
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
