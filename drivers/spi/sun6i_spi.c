// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2018 Amarula Solutions.
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <common.h>
#include <dm.h>
#include <spi.h>
#include <errno.h>
#include <fdt_support.h>
#include <wait_bit.h>

#include <asm/bitops.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include <asm/arch/clock.h>

DECLARE_GLOBAL_DATA_PTR;

#define SUN6I_FIFO_DEPTH			128
#define SUN8I_FIFO_DEPTH			64

#define SUN6I_GBL_CTL_BUS_ENABLE		BIT(0)
#define SUN6I_GBL_CTL_MASTER			BIT(1)
#define SUN6I_GBL_CTL_TP			BIT(7)
#define SUN6I_GBL_CTL_RST			BIT(31)

#define SUN6I_TFR_CTL_CPHA			BIT(0)
#define SUN6I_TFR_CTL_CPOL			BIT(1)
#define SUN6I_TFR_CTL_SPOL			BIT(2)
#define SUN6I_TFR_CTL_CS_MASK			0x30
#define SUN6I_TFR_CTL_CS(cs)			(((cs) << 4) & SUN6I_TFR_CTL_CS_MASK)
#define SUN6I_TFR_CTL_CS_MANUAL			BIT(6)
#define SUN6I_TFR_CTL_CS_LEVEL			BIT(7)
#define SUN6I_TFR_CTL_DHB			BIT(8)
#define SUN6I_TFR_CTL_FBS			BIT(12)
#define SUN6I_TFR_CTL_XCH_MASK			0x80000000
#define SUN6I_TFR_CTL_XCH			BIT(31)

#define SUN6I_FIFO_CTL_RF_RDY_TRIG_LEVEL_MASK	0xff
#define SUN6I_FIFO_CTL_RF_RDY_TRIG_LEVEL_BITS	0
#define SUN6I_FIFO_CTL_RF_RST			BIT(15)
#define SUN6I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_MASK	0xff
#define SUN6I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_BITS	16
#define SUN6I_FIFO_CTL_TF_RST			BIT(31)

#define SUN6I_CLK_CTL_CDR2_MASK			0xff
#define SUN6I_CLK_CTL_CDR2(div)			(((div) & SUN6I_CLK_CTL_CDR2_MASK) << 0)
#define SUN6I_CLK_CTL_CDR1_MASK			0xf
#define SUN6I_CLK_CTL_CDR1(div)			(((div) & SUN6I_CLK_CTL_CDR1_MASK) << 8)
#define SUN6I_CLK_CTL_DRS			BIT(12)

#define SUN6I_MAX_XFER_SIZE			0xffffff
#define SUN6I_BURST_CNT(cnt)			((cnt) & SUN6I_MAX_XFER_SIZE)
#define SUN6I_XMIT_CNT(cnt)			((cnt) & SUN6I_MAX_XFER_SIZE)
#define SUN6I_BURST_CTL_CNT_STC(cnt)		((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_SPI_MAX_RATE	24000000
#define SUN6I_SPI_MIN_RATE	3000
#define SUN6I_SPI_DEFAULT_RATE	1000000
#define SUN6I_SPI_TIMEOUT_US	1000000

/* sun6i spi register set */
struct sun6i_spi_regs {
	u32 res1;	/* 0x00 */ 
	u32 gblctl;	/* 0x04 */
	u32 tfrctl;	/* 0x08 */
	u32 res2;	/* 0x0c */ 
	u32 intctl;	/* 0x10 */
	u32 intsta;	/* 0x14 */
	u32 fifoctl;	/* 0x18 */
	u32 fifosta;	/* 0x1c */
	u32 res3;	/* 0x20 */ 
	u32 clkctl;	/* 0x24 */
	u32 res4[2];	/* 0x28 */ 
	u32 bc;		/* 0x30 */
	u32 tc;		/* 0x34 */
	u32 bctlc;	/* 0x38 */
	u32 res5[113];	/* 0x3c */ 
	u32 txdata;	/* 0x200 */
	u32 res6[63];	/* 0x204 */ 
	u32 rxdata;	/* 0x300 */
};

struct sun6i_spi_platdata {
	u32 base_addr;
	u32 max_hz;
};

struct sun6i_spi_priv {
	struct sun6i_spi_regs *regs;
	u32 freq;
	u32 mode;
	u32 fifo_depth;

	const u8 *tx_buf;
	u8 *rx_buf;
};

static inline void sun6i_spi_drain_fifo(struct sun6i_spi_priv *priv, int len)
{
	u8 byte;

	while (len--) {
		byte = readb(&priv->regs->rxdata);
		if (priv->rx_buf)
			*priv->rx_buf++ = byte;

	}
}

static inline void sun6i_spi_fill_fifo(struct sun6i_spi_priv *priv, int len)
{
	u8 byte;

	while (len--) {
		byte = priv->tx_buf ? *priv->tx_buf++ : 0;
		writeb(byte, &priv->regs->txdata);
	}
}

static void sun6i_spi_set_cs(struct udevice *bus, u8 cs, bool enable)
{
	struct sun6i_spi_priv *priv = dev_get_priv(bus);
	u32 reg;

	reg = readl(&priv->regs->tfrctl);
	reg &= ~SUN6I_TFR_CTL_CS_MASK;
	reg |= SUN6I_TFR_CTL_CS(cs);

	if (enable)
		reg &= ~SUN6I_TFR_CTL_CS_LEVEL;
	else
		reg |= SUN6I_TFR_CTL_CS_LEVEL;

	writel(reg, &priv->regs->tfrctl);
}

static int sun6i_spi_parse_pins(struct udevice *dev)
{
	const void *fdt = gd->fdt_blob;
	const char *pin_name;
	const fdt32_t *list;
	u32 phandle;
	int drive, pull = 0, pin, i;
	int offset;
	int size;

	list = fdt_getprop(fdt, dev_of_offset(dev), "pinctrl-0", &size);
	if (!list) {
		printf("WARNING: sun6i_spi: cannot find pinctrl-0 node\n");
		return -EINVAL;
	}

	while (size) {
		phandle = fdt32_to_cpu(*list++);
		size -= sizeof(*list);

		offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (offset < 0)
			return offset;

		drive = fdt_getprop_u32_default_node(fdt, offset, 0,
						     "drive-strength", 0);
		if (drive) {
			if (drive <= 10)
				drive = 0;
			else if (drive <= 20)
				drive = 1;
			else if (drive <= 30)
				drive = 2;
			else
				drive = 3;
		} else {
			drive = fdt_getprop_u32_default_node(fdt, offset, 0,
							     "allwinner,drive",
							      0);
			drive = min(drive, 3);
		}

		if (fdt_get_property(fdt, offset, "bias-disable", NULL))
			pull = 0;
		else if (fdt_get_property(fdt, offset, "bias-pull-up", NULL))
			pull = 1;
		else if (fdt_get_property(fdt, offset, "bias-pull-down", NULL))
			pull = 2;
		else
			pull = fdt_getprop_u32_default_node(fdt, offset, 0,
							    "allwinner,pull",
							     0);
		pull = min(pull, 2);

		for (i = 0; ; i++) {
			pin_name = fdt_stringlist_get(fdt, offset,
						      "pins", i, NULL);
			if (!pin_name) {
				pin_name = fdt_stringlist_get(fdt, offset,
							      "allwinner,pins",
							       i, NULL);
				if (!pin_name)
					break;
			}

			pin = name_to_gpio(pin_name);
			if (pin < 0)
				break;

#if defined(CONFIG_MACH_SUN50I)
			sunxi_gpio_set_cfgpin(pin, SUN50I_GPC_SPI0);
#else
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SPI0);
#endif
			sunxi_gpio_set_drv(pin, drive);
			sunxi_gpio_set_pull(pin, pull);
		}
	}
	return 0;
}

static inline void sun6i_spi_enable_clock(void)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *const)SUNXI_CCM_BASE;

	setbits_le32(&ccm->ahb_reset0_cfg, (1 << AHB_RESET_OFFSET_SPI0));
	setbits_le32(&ccm->ahb_gate0, (1 << AHB_GATE_OFFSET_SPI0));
	writel((1 << 31), &ccm->spi0_clk_cfg);
}

static int sun6i_spi_ofdata_to_platdata(struct udevice *bus)
{
	struct sun6i_spi_platdata *plat = dev_get_platdata(bus);
	int node = dev_of_offset(bus);

	plat->base_addr = devfdt_get_addr(bus);
	plat->max_hz = fdtdec_get_int(gd->fdt_blob, node,
				      "spi-max-frequency",
				      SUN6I_SPI_DEFAULT_RATE);

	if (plat->max_hz > SUN6I_SPI_MAX_RATE)
		plat->max_hz = SUN6I_SPI_MAX_RATE;

	return 0;
}

static int sun6i_spi_probe(struct udevice *bus)
{
	struct sun6i_spi_platdata *plat = dev_get_platdata(bus);
	struct sun6i_spi_priv *priv = dev_get_priv(bus);

	sun6i_spi_enable_clock();
	sun6i_spi_parse_pins(bus);

	priv->regs = (struct sun6i_spi_regs *)(uintptr_t)plat->base_addr;
	priv->freq = plat->max_hz;
	priv->fifo_depth = dev_get_driver_data(bus);

	return 0;
}

static int sun6i_spi_claim_bus(struct udevice *dev)
{
	struct sun6i_spi_priv *priv = dev_get_priv(dev->parent);

	writel(SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER |
	       SUN6I_GBL_CTL_TP, &priv->regs->gblctl);
	writel(SUN6I_TFR_CTL_CS_MANUAL, &priv->regs->tfrctl);

	return 0;
}

static int sun6i_spi_release_bus(struct udevice *dev)
{
	struct sun6i_spi_priv *priv = dev_get_priv(dev->parent);
	u32 reg;

	reg = readl(&priv->regs->gblctl);
	reg &= ~SUN6I_GBL_CTL_BUS_ENABLE;
	writel(reg, &priv->regs->gblctl);

	return 0;
}

static int sun6i_spi_xfer(struct udevice *dev, unsigned int bitlen,
			  const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct sun6i_spi_priv *priv = dev_get_priv(bus);
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);

	u32 len = bitlen / 8;
	u32 reg;
	u8 nbytes;
	int ret;

	priv->tx_buf = dout;
	priv->rx_buf = din;

	if (bitlen % 8) {
		debug("%s: non byte-aligned SPI transfer.\n", __func__);
		return -ENAVAIL;
	}

	if (flags & SPI_XFER_BEGIN)
		sun6i_spi_set_cs(bus, slave_plat->cs, true);

	/* Reset FIFOs */
	writel(SUN6I_FIFO_CTL_RF_RST | SUN6I_FIFO_CTL_TF_RST,
	       &priv->regs->fifoctl);

	while (len) {
		/* Setup the transfer now... */
		nbytes = min(len, priv->fifo_depth);

		/* Setup the counters */
		writel(SUN6I_BURST_CNT(nbytes), &priv->regs->bc);
		writel(SUN6I_XMIT_CNT(nbytes), &priv->regs->tc);
		writel(SUN6I_BURST_CTL_CNT_STC(nbytes), &priv->regs->bctlc);

		/* Fill the TX FIFO */
		sun6i_spi_fill_fifo(priv, nbytes);

		/* Start the transfer */
		reg = readl(&priv->regs->tfrctl);
		writel(reg | SUN6I_TFR_CTL_XCH, &priv->regs->tfrctl);

		/* Wait transfer to complete */
		ret = wait_for_bit_le32(&priv->regs->tfrctl,
					SUN6I_TFR_CTL_XCH_MASK, false,
					SUN6I_SPI_TIMEOUT_US, false);
		if (ret) {
			printf("ERROR: sun6i_spi: Timeout transferring data\n");
			sun6i_spi_set_cs(bus, slave_plat->cs, false);
			return ret;
		}

		/* Drain the RX FIFO */
		sun6i_spi_drain_fifo(priv, nbytes);

		len -= nbytes;
	}

	if (flags & SPI_XFER_END)
		sun6i_spi_set_cs(bus, slave_plat->cs, false);

	return 0;
}

static int sun6i_spi_set_speed(struct udevice *dev, uint speed)
{
	struct sun6i_spi_platdata *plat = dev_get_platdata(dev);
	struct sun6i_spi_priv *priv = dev_get_priv(dev);
	unsigned int div;
	u32 reg;

	if (speed > plat->max_hz)
		speed = plat->max_hz;

	if (speed < SUN6I_SPI_MIN_RATE)
		speed = SUN6I_SPI_MIN_RATE;

	/*
	 * Setup clock divider.
	 *
	 * We have two choices there. Either we can use the clock
	 * divide rate 1, which is calculated thanks to this formula:
	 * SPI_CLK = MOD_CLK / (2 ^ cdr)
	 * Or we can use CDR2, which is calculated with the formula:
	 * SPI_CLK = MOD_CLK / (2 * (cdr + 1))
	 * Whether we use the former or the latter is set through the
	 * DRS bit.
	 *
	 * First try CDR2, and if we can't reach the expected
	 * frequency, fall back to CDR1.
	 */
	reg = readl(&priv->regs->clkctl);
	div = SUN6I_SPI_MAX_RATE / (2 * speed);
	if (div <= (SUN6I_CLK_CTL_CDR2_MASK + 1)) {
		if (div > 0)
			div--;

		reg |= SUN6I_CLK_CTL_CDR2(div) | SUN6I_CLK_CTL_DRS;
	} else {
		div = __ilog2(SUN6I_SPI_MAX_RATE) - __ilog2(speed);
		reg |= SUN6I_CLK_CTL_CDR1(div);
	}

	writel(reg, &priv->regs->clkctl);
	priv->freq = speed;

	return 0;
}

static int sun6i_spi_set_mode(struct udevice *dev, uint mode)
{
	struct sun6i_spi_priv *priv = dev_get_priv(dev);
	u32 reg;

	reg = readl(&priv->regs->tfrctl);
	reg &= ~(SUN6I_TFR_CTL_CPOL | SUN6I_TFR_CTL_CPHA);

	if (mode & SPI_CPOL)
		reg |= SUN6I_TFR_CTL_CPOL;

	if (mode & SPI_CPHA)
		reg |= SUN6I_TFR_CTL_CPHA;

	priv->mode = mode;
	writel(reg, &priv->regs->tfrctl);

	return 0;
}

static const struct dm_spi_ops sun6i_spi_ops = {
	.claim_bus		= sun6i_spi_claim_bus,
	.release_bus		= sun6i_spi_release_bus,
	.xfer			= sun6i_spi_xfer,
	.set_speed		= sun6i_spi_set_speed,
	.set_mode		= sun6i_spi_set_mode,
};

static const struct udevice_id sun6i_spi_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-spi", .data = SUN6I_FIFO_DEPTH },
	{ .compatible = "allwinner,sun8i-h3-spi", .data = SUN8I_FIFO_DEPTH },
	{ .compatible = "allwinner,sun50i-a64-spi", .data = SUN8I_FIFO_DEPTH },
	{ }
};

U_BOOT_DRIVER(sun6i_spi) = {
	.name	= "sun6i_spi",
	.id	= UCLASS_SPI,
	.of_match	= sun6i_spi_ids,
	.ops	= &sun6i_spi_ops,
	.ofdata_to_platdata	= sun6i_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size	= sizeof(struct sun6i_spi_platdata),
	.priv_auto_alloc_size	= sizeof(struct sun6i_spi_priv),
	.probe	= sun6i_spi_probe,
};
