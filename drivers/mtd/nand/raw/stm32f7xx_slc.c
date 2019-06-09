/*
 * ST STM32XX NAND SLC driver
 *
 * Authors:
 *    Yang Sheng <iysheng@163.com>
 *    Kevin Wells <kevin.wells@nxp.com>
 *    Roland Stigge <stigge@antcom.de>
 *
 * Copyright © 2011 NXP Semiconductors
 * Copyright © 2012 Roland Stigge
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define STM32F7XX_MODNAME		"stm32f7xx-nand"

/*
 * NAND ECC Layout for small page NAND devices
 * Note: For large and huge page devices, the default layouts are used
 */
static int stm32f7xx_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = 6;
	oobregion->offset = 10;

	return 0;
}

static int stm32f7xx_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	if (section > 1)
		return -ERANGE;

	if (!section) {
		oobregion->offset = 0;
		oobregion->length = 4;
	} else {
		oobregion->offset = 6;
		oobregion->length = 4;
	}

	return 0;
}

static const struct mtd_ooblayout_ops stm32f7xx_ooblayout_ops = {
	.ecc = stm32f7xx_ooblayout_ecc,
	.free = stm32f7xx_ooblayout_free,
};

static u8 bbt_pattern[] = {'B', 'b', 't', '0' };
static u8 mirror_pattern[] = {'1', 't', 'b', 'B' };
#if 0
/*
 * Small page FLASH BBT descriptors, marker at offset 0, version at offset 6
 * Note: Large page devices used the default layout
 */
static struct nand_bbt_descr bbt_smallpage_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	0,
	.len = 4,
	.veroffs = 6,
	.maxblocks = 4,
	.pattern = bbt_pattern
};
static struct nand_bbt_descr bbt_smallpage_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	0,
	.len = 4,
	.veroffs = 6,
	.maxblocks = 4,
	.pattern = mirror_pattern
};
#endif

/*
 * NAND platform configuration structure
 */
struct stm32f7xx_nand_cfg_slc {
	void __iomem * iobase_addr;
	void __iomem * iobase_cmd;
	void __iomem * iobase_data;
	struct mtd_partition *parts;
	unsigned num_parts;
};

struct stm32f7xx_nand_host {
	struct nand_chip	nand_chip;
	struct stm32f7xx_slc_platform_data *pdata;
	struct clk		*clk;
	void __iomem		*io_base;
	struct stm32f7xx_nand_cfg_slc *ncfg;

	struct completion	comp;
	struct dma_chan		*dma_chan;
	uint32_t		dma_buf_len;
	struct dma_slave_config	dma_slave_config;
	struct scatterlist	sgl;

	/*
	 * DMA and CPU addresses of ECC work area and data buffer
	 */
	uint32_t		*ecc_buf;
	uint8_t			*data_buf;
	dma_addr_t		io_base_dma;
};
/* 这个函数需要修改 */
static void stm32f7xx_nand_setup(struct stm32f7xx_nand_host *host)
{
#if 0
	/* Reset SLC controller */
	writeb(SLCCTRL_SW_RESET, host->ncfg->iobase_cmd);
	udelay(1000);

	/* Basic setup */
	writeb(0, SLC_CFG(host->io_base));
	writeb(0, SLC_IEN(host->io_base));
	writeb((SLCSTAT_INT_TC | SLCSTAT_INT_RDY_EN),
		SLC_ICR(host->io_base));

	/* Get base clock for SLC block */
	clkrate = clk_get_rate(host->clk);
	if (clkrate == 0)
		clkrate = STM32F7XX_DEF_BUS_RATE;

	/* Compute clock setup values */
	tmp = SLCTAC_WDR(host->ncfg->wdr_clks) |
		SLCTAC_WWIDTH(clkrate, host->ncfg->wwidth) |
		SLCTAC_WHOLD(clkrate, host->ncfg->whold) |
		SLCTAC_WSETUP(clkrate, host->ncfg->wsetup) |
		SLCTAC_RDR(host->ncfg->rdr_clks) |
		SLCTAC_RWIDTH(clkrate, host->ncfg->rwidth) |
		SLCTAC_RHOLD(clkrate, host->ncfg->rhold) |
		SLCTAC_RSETUP(clkrate, host->ncfg->rsetup);
	writeb(tmp, SLC_TAC(host->io_base));
#endif
}

/*
 * Hardware specific access to control lines
 */
static void stm32f7xx_nand_cmd_ctrl(struct nand_chip *chip, int cmd,
				  unsigned int ctrl)
{
	struct stm32f7xx_nand_host *host = nand_get_controller_data(chip);

	if (cmd != NAND_CMD_NONE) {
		if (ctrl & NAND_CLE)
			/* 命令寄存器 */
			//writeb(cmd, SLC_CMD(host->io_base));
			writeb(cmd, host->ncfg->iobase_cmd);
		else
			/* 写地址 */
			//writeb(cmd, SLC_ADDR(host->io_base));
			writeb(cmd, host->ncfg->iobase_data);
	}
}

/*
 * Prepares SLC for transfers with H/W ECC enabled
 */
static void stm32f7xx_nand_ecc_enable(struct nand_chip *chip, int mode)
{
	/* Hardware ECC is enabled automatically in hardware as needed */
}

/*
 * Calculates the ECC for the data
 */
static int stm32f7xx_nand_ecc_calculate(struct nand_chip *chip,
				      const unsigned char *buf,
				      unsigned char *code)
{
	/*
	 * ECC is calculated automatically in hardware during syndrome read
	 * and write operations, so it doesn't need to be calculated here.
	 */
	return 0;
}

/*
 * Read a single byte from NAND device
 */
static uint8_t stm32f7xx_nand_read_byte(struct nand_chip *chip)
{
	struct stm32f7xx_nand_host *host = nand_get_controller_data(chip);

	return (uint8_t)readb(host->ncfg->iobase_addr);
}

/*
 * Simple device read without ECC
 */
static void stm32f7xx_nand_read_buf(struct nand_chip *chip, u_char *buf, int len)
{
	struct stm32f7xx_nand_host *host = nand_get_controller_data(chip);

	/* Direct device read with no ECC */
	while (len-- > 0)
		*buf++ = (uint8_t)readb(host->ncfg->iobase_addr);
}

/*
 * Simple device write without ECC
 */
static void stm32f7xx_nand_write_buf(struct nand_chip *chip, const uint8_t *buf,
				   int len)
{
	struct stm32f7xx_nand_host *host = nand_get_controller_data(chip);

	/* Direct device write with no ECC */
	while (len-- > 0)
		writeb((uint8_t)*buf++, host->ncfg->iobase_addr);
}

/*
 * Read the data and OOB data from the device, no ECC correction with the
 * data or OOB data
 */
static int stm32f7xx_nand_read_page_raw_syndrome(struct nand_chip *chip,
					       uint8_t *buf, int oob_required,
					       int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	/* Issue read command */
	nand_read_page_op(chip, page, 0, NULL, 0);

	/* Raw reads can just use the FIFO interface */
	chip->legacy.read_buf(chip, buf, chip->ecc.size * chip->ecc.steps);
	chip->legacy.read_buf(chip, chip->oob_poi, mtd->oobsize);

	return 0;
}

/*
 * Write the data and OOB data to the device, no ECC correction with the
 * data or OOB data
 */
static int stm32f7xx_nand_write_page_raw_syndrome(struct nand_chip *chip,
						const uint8_t *buf,
						int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	/* Raw writes can just use the FIFO interface */
	nand_prog_page_begin_op(chip, page, 0, buf,
				chip->ecc.size * chip->ecc.steps);
	chip->legacy.write_buf(chip, chip->oob_poi, mtd->oobsize);

	return nand_prog_page_end_op(chip);
}

static struct stm32f7xx_nand_cfg_slc *stm32f7xx_parse_dt(struct device *dev)
{
	struct stm32f7xx_nand_cfg_slc *ncfg;
	struct device_node *np = dev->of_node;
	u32 tmp_addr = 0;

	ncfg = devm_kzalloc(dev, sizeof(*ncfg), GFP_KERNEL);
	if (!ncfg)
		return NULL;

	of_property_read_u32(np, "iobase_addr", (u32 *)&tmp_addr);
	ncfg->iobase_addr = ioremap_nocache(tmp_addr, PAGE_SIZE);
	of_property_read_u32(np, "iobase_cmd", (u32 *)&tmp_addr);
	ncfg->iobase_cmd = ioremap_nocache(tmp_addr, PAGE_SIZE);
	of_property_read_u32(np, "iobase_data", (u32 *)&tmp_addr);
	ncfg->iobase_data = ioremap_nocache(tmp_addr, PAGE_SIZE);

	return ncfg;
}

static int stm32f7xx_nand_attach_chip(struct nand_chip *chip)
{
#if 0
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct stm32f7xx_nand_host *host = nand_get_controller_data(chip);

	/* OOB and ECC CPU and DMA work areas */
	host->ecc_buf = (uint32_t *)(host->data_buf + STM32F7XX_DMA_DATA_SIZE);

	/*
	 * Small page FLASH has a unique OOB layout, but large and huge
	 * page FLASH use the standard layout. Small page FLASH uses a
	 * custom BBT marker layout.
	 */
	if (mtd->writesize <= 512)
		mtd_set_ooblayout(mtd, &stm32f7xx_ooblayout_ops);

	/* These sizes remain the same regardless of page size */
	chip->ecc.size = 256;
	chip->ecc.bytes = STM32F7XX_SLC_DEV_ECC_BYTES;
	chip->ecc.prepad = 0;
	chip->ecc.postpad = 0;

	/*
	 * Use a custom BBT marker setup for small page FLASH that
	 * won't interfere with the ECC layout. Large and huge page
	 * FLASH use the standard layout.
	 */
	if ((chip->bbt_options & NAND_BBT_USE_FLASH) &&
	    mtd->writesize <= 512) {
		chip->bbt_td = &bbt_smallpage_main_descr;
		chip->bbt_md = &bbt_smallpage_mirror_descr;
	}
#endif
	return 0;
}

static const struct nand_controller_ops stm32f7xx_nand_controller_ops = {
	.attach_chip = stm32f7xx_nand_attach_chip,
};

/*
 * Probe for NAND controller
 */
/* stm32f7xx nand 控制器的 probe 函数 */
static int stm32f7xx_nand_probe(struct platform_device *pdev)
{
	struct stm32f7xx_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct resource *rc;
	int res;

	/* Allocate memory for the device structure (and zero it) */
	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	rc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->io_base = devm_ioremap_resource(&pdev->dev, rc);
	if (IS_ERR(host->io_base))
		return PTR_ERR(host->io_base);
	if (pdev->dev.of_node)
		/* 从设备树解析 NAND 控制器的配置 */
		host->ncfg = stm32f7xx_parse_dt(&pdev->dev);

	if (!host->ncfg) {
		dev_err(&pdev->dev,
			"Missing or bad NAND config from device tree\n");
		return -ENOENT;
	}

	host->pdata = dev_get_platdata(&pdev->dev);

	chip = &host->nand_chip;
	mtd = nand_to_mtd(chip);
	/* chip 是 nand flash 的抽象，host 是 soc NAND 控制器的抽象
	 * 将 chip 的 prive 关联到 host 控制器
	 */
	nand_set_controller_data(chip, host);
	nand_set_flash_node(chip, pdev->dev.of_node);
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	/* Get NAND clock */
	host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(host->clk)) {
		dev_err(&pdev->dev, "Clock failure\n");
		res = -ENOENT;
		goto init_over;
	}
	res = clk_prepare_enable(host->clk);
	if (res)
		goto init_over;

	/* Set NAND IO addresses and command/ready functions */
	/* 初始化 nand chip 的读写数据的地址，以及发送命令的回调函数 */
	chip->legacy.IO_ADDR_R = host->ncfg->iobase_addr; //host->ncfg->iobase_data;
	chip->legacy.IO_ADDR_W = host->ncfg->iobase_addr; //host->ncfg->iobase_data;
	chip->legacy.cmd_ctrl = stm32f7xx_nand_cmd_ctrl;
	//chip->legacy.dev_ready = stm32f7xx_nand_device_ready;
	chip->legacy.chip_delay = 20; /* 20us command delay time */

	/* Init NAND controller */
	stm32f7xx_nand_setup(host);

	platform_set_drvdata(pdev, host);

	/* NAND callbacks for LPC32xx SLC hardware */
	/* 初始化回调函数 */
	chip->ecc.mode = NAND_ECC_SOFT;//NAND_ECC_NONE;//NAND_ECC_HW_SYNDROME;
	chip->ecc.algo = 1; //"bch";
	chip->legacy.read_byte = stm32f7xx_nand_read_byte;
	chip->legacy.read_buf = stm32f7xx_nand_read_buf;
	chip->legacy.write_buf = stm32f7xx_nand_write_buf;
	chip->ecc.read_page_raw = stm32f7xx_nand_read_page_raw_syndrome;
//	chip->ecc.read_page = stm32f7xx_nand_read_page_syndrome;
	chip->ecc.write_page_raw = stm32f7xx_nand_write_page_raw_syndrome;
//	chip->ecc.write_page = stm32f7xx_nand_write_page_syndrome;
//	chip->ecc.write_oob = stm32f7xx_nand_write_oob_syndrome;
//	chip->ecc.read_oob = stm32f7xx_nand_read_oob_syndrome;
//	chip->ecc.calculate = stm32f7xx_nand_ecc_calculate;
//	chip->ecc.correct = nand_correct_data;
//	chip->ecc.strength = 1;
//	chip->ecc.hwctl = stm32f7xx_nand_ecc_enable;

	/* Find NAND device */
	/* 控制器的专有函数处理集合 */
	chip->legacy.dummy_controller.ops = &stm32f7xx_nand_controller_ops;
	res = nand_scan(chip, 1);
	if (res)
		goto cleanup_nand;

	mtd->name = "st_stm32f7xx_slc";
	/* 注册 nand flash 设备到 mtd 框架 */
	res = mtd_device_register(mtd, host->ncfg->parts,
				  host->ncfg->num_parts);
	if (res)
		goto cleanup_nand;

	return 0;

cleanup_nand:
	nand_cleanup(chip);
//unprepare_clk:
	clk_disable_unprepare(host->clk);
init_over:
	return res;
}

/*
 * Remove NAND device.
 */
static int stm32f7xx_nand_remove(struct platform_device *pdev)
{
	struct stm32f7xx_nand_host *host = platform_get_drvdata(pdev);

	nand_release(&host->nand_chip);
	clk_disable_unprepare(host->clk);

	return 0;
}

#ifdef CONFIG_PM
static int stm32f7xx_nand_resume(struct platform_device *pdev)
{
#if 0
	struct stm32f7xx_nand_host *host = platform_get_drvdata(pdev);
	/* Re-enable NAND clock */
	ret = clk_prepare_enable(host->clk);
	if (ret)
		return ret;

	/* Fresh init of NAND controller */
	stm32f7xx_nand_setup(host);
#endif

	return 0;
}

static int stm32f7xx_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
#if 0
	struct stm32f7xx_nand_host *host = platform_get_drvdata(pdev);
	/* Disable clock */
	clk_disable_unprepare(host->clk);
#endif
	return 0;
}

#else
#define stm32f7xx_nand_resume NULL
#define stm32f7xx_nand_suspend NULL
#endif

static const struct of_device_id stm32f7xx_nand_match[] = {
	{ .compatible = "st,stm32f7x-slc" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stm32f7xx_nand_match);

static struct platform_driver stm32f7xx_nand_driver = {
	.probe		= stm32f7xx_nand_probe,
	.remove		= stm32f7xx_nand_remove,
	.resume		= stm32f7xx_nand_resume,
	.suspend	= stm32f7xx_nand_suspend,
	.driver		= {
		.name	= STM32F7XX_MODNAME,
		.of_match_table = stm32f7xx_nand_match,
	},
};

module_platform_driver(stm32f7xx_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Sheng <iysheng@163.com>");
MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_AUTHOR("Roland Stigge <stigge@antcom.de>");
MODULE_DESCRIPTION("NAND driver for the ST STM32F7XX SLC controller");
