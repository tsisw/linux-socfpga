#include <linux/module.h>
#include <linux/property.h>
#include <linux/clkdev.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-dp.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "soc_dp_reg.h"

#define SOC_DP_VCO_MIN_KHZ        1000000
#define SOC_DP_VCO_MAX_KHZ        3000000
#define SOC_DP_PLL_FRAC_MOD       16777216  /* 2^24 */
#define SOC_DP_PLL_ERR_TOLERANCE  10

/* Per-stream register stride: none on the vendor's FPGA (G15). */
#ifdef CONFIG_SOC_DP_FPGA_LIMITS
#define SOC_DPTX_STREAM_OFFSET    0x0
#else
#define SOC_DPTX_STREAM_OFFSET    0x10000
#endif
#define SOC_DP_PHY_MAX_STREAMS    4

#ifdef CONFIG_SOC_DP_ACTIVATE_DO_DIV
#include <linux/math64.h>
#endif

/* Mappings for PHY Swing/Emphasis Levels */
static const uint32_t phy_swing_map[] = { 0x0, 0x1, 0x2, 0x3 };
static const uint32_t phy_preemp_map[] = { 0x0, 0x1, 0x2, 0x3 };

enum soc_dp_link_rate {
	SOC_DP_LINK_RATE_1_62 = 1620000, /* 1.62 Gbps */
	SOC_DP_LINK_RATE_2_70 = 2700000, /* 2.70 Gbps */
	SOC_DP_LINK_RATE_5_40 = 5400000, /* 5.40 Gbps */
	SOC_DP_LINK_RATE_8_10 = 8100000, /* 8.10 Gbps */
};

enum soc_dp_lane_count {
	SOC_DP_LANE_1 = 1,
	SOC_DP_LANE_2 = 2,
	SOC_DP_LANE_4 = 4,
};

/* Data structure for Core PLL results */
struct soc_dp_core_pll_cfg {
	uint32_t target_rate_kbps;
	uint32_t actual_rate_khz;
	uint32_t vco_freq_khz;
	uint32_t frac;
	uint16_t fbdiv;
	uint8_t  prediv;
	uint8_t  postdiv_reg;
	uint8_t  postdiv_en;
	uint8_t  vcoclk_div8_en;
	uint8_t  frac_pd;
	uint8_t  clk_16mdiv;
	bool     valid;
};

/* Data structure for Pixel PLL results */
struct soc_dp_pixel_pll_cfg {
	uint32_t target_pclk_khz;
	uint32_t actual_pclk_khz;
	uint32_t vco_freq_khz;
	uint32_t frac;
	uint16_t fbdiv;
	uint8_t  prediv;
	uint8_t  div5_en;
	uint8_t  divm;
	uint8_t  divaux;
	uint8_t  divp;
	uint8_t  frac_pd;
	bool     valid;
};

struct soc_dp_phy_pixel_clk {
	struct clk_hw hw;
	struct soc_dp_phy_priv *priv;
	uint8_t stream_id;
};

#define to_soc_dp_phy_pixel_clk(_hw) \
	container_of(_hw, struct soc_dp_phy_pixel_clk, hw)

struct soc_dp_phy_priv {
	struct device *dev;
	void __iomem *regs;
	uint32_t ref_clk_khz;

	uint32_t link_rate_khz;
	int lane_count;

	struct soc_dp_phy_pixel_clk pixel_clks[SOC_DP_PHY_MAX_STREAMS];
	unsigned long pixel_rate_hz[SOC_DP_PHY_MAX_STREAMS];
};

static const struct soc_dp_phy_vol_setting {
	uint8_t mainsel; /* 5 bits */
	uint8_t postsel; /* 4 bits */
	uint8_t presel;  /* 3 bits */
	uint8_t isel;    /* 4 bits */
} vol_cfg_table[4][4][4] = {
	/* --- 1.62 Gbps --- */
	{
		/* Swing 0 */
		{ {0x0a, 0x0, 0x0, 0x5}, {0x0e, 0x2, 0x0, 0x5}, {0x11, 0x4, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 1 */
		{ {0x0c, 0x0, 0x0, 0x5}, {0x0f, 0x2, 0x0, 0x5}, {0x13, 0x5, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 2 */
		{ {0x0f, 0x0, 0x0, 0x5}, {0x12, 0x3, 0x0, 0x5}, {0x18, 0x6, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 3 */
		{ {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },
	},
	/* --- 2.7 Gbps --- */
	{
		/* Swing 0 */
		{ {0x07, 0x0, 0x0, 0x5}, {0x0a, 0x2, 0x0, 0x5}, {0x0f, 0x5, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 1 */
		{ {0x0d, 0x0, 0x0, 0x5}, {0x10, 0x2, 0x0, 0x5}, {0x14, 0x5, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 2 */
		{ {0x12, 0x0, 0x0, 0x5}, {0x16, 0x3, 0x0, 0x5}, {0x19, 0x6, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 3 */
		{ {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },
	},
	/* --- 5.4 Gbps --- */
	{
		/* Swing 0 */
		{ {0x06, 0x0, 0x0, 0x5}, {0x09, 0x1, 0x0, 0x5}, {0x0d, 0x3, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 1 */
		{ {0x0b, 0x0, 0x0, 0x5}, {0x12, 0x3, 0x0, 0x5}, {0x17, 0x5, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 2 */
		{ {0x10, 0x0, 0x0, 0x5}, {0x1d, 0x4, 0x0, 0x5}, {0x1a, 0x6, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 3 */
		{ {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },
	},
	/* --- 8.1 Gbps --- */
	{
		/* Swing 0 */
		{ {0x06, 0x0, 0x0, 0x5}, {0x09, 0x1, 0x0, 0x5}, {0x0d, 0x3, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 1 */
		{ {0x0b, 0x0, 0x0, 0x5}, {0x12, 0x3, 0x0, 0x5}, {0x17, 0x5, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 2 */
		{ {0x10, 0x0, 0x0, 0x5}, {0x1d, 0x4, 0x0, 0x5}, {0x1a, 0x6, 0x0, 0x5}, {0, 0, 0, 0} },
		/* Swing 3 */
		{ {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },
	},
};

static int soc_dp_reg_write(struct soc_dp_phy_priv *priv,
		uint32_t offset, uint32_t bit_wide, uint32_t mask, uint32_t val)
{
	uint32_t reg_val;

	reg_val = (uint32_t)readl(priv->regs + offset);
	reg_val &= ~mask;
	reg_val |= val & mask;
	// printk("[W] 0x%x 0x%x\n", offset, reg_val);
	writel(reg_val, priv->regs + offset);

	return 0;
}

static int soc_dp_reg_write_range(struct soc_dp_phy_priv *priv,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val)
{
	uint32_t mask;

	mask = (uint32_t)(((((uint64_t)1) << (high - low + 1)) - 1) << low);
	return soc_dp_reg_write(priv, offset, 32, mask, (val << low) & mask);
}

static int soc_dp_stream_reg_write_range(struct soc_dp_phy_priv *priv, uint8_t stream_id,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t val)
{
	return soc_dp_reg_write_range(priv,
			offset + (stream_id * SOC_DPTX_STREAM_OFFSET), high, low, val);
}

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
static int soc_dp_reg_read(struct soc_dp_phy_priv *priv,
		uint32_t offset, uint32_t bit_wide, uint32_t mask, uint32_t *val)
{
	*val = ((uint32_t)readl(priv->regs + offset)) & mask;
	// printk("[R] 0x%x 0x%x\n", offset, *val);
	return 0;
}

static int soc_dp_reg_read_range(struct soc_dp_phy_priv *priv,
		uint32_t offset, uint32_t high, uint32_t low, uint32_t *val)
{
	int ret;
	uint32_t mask;

	mask = (uint32_t)(((((uint64_t)1) << (high - low + 1)) - 1) << low);
	ret = soc_dp_reg_read(priv, offset, 32, mask, val);
	*val = *val >> low;

	return ret;
}
#endif

/*
 * soc_dp_div64
 * @n: Pointer to dividend (will be updated to quotient)
 * @base: Divisor
 * Return: Remainder
 */
static uint32_t soc_dp_div64(uint64_t *n, uint32_t base)
{
#ifdef CONFIG_SOC_DP_ACTIVATE_DO_DIV
	return do_div(*n, base);
#else
	uint32_t rem = *n % base;
	*n = *n / base;
	return rem;
#endif
}

static inline uint64_t soc_dp_abs_diff(uint64_t a, uint64_t b)
{
	return (a > b) ? (a - b) : (b - a);
}

/*
 * soc_dp_is_better_config - Determines if the new configuration is better
 * Priorities:
 * 1. Integer mode (Frac == 0)
 * 2. Lower pre-divider (Higher PFD frequency)
 * 3. Higher VCO frequency
 */
static int soc_dp_is_better_config(bool new_valid, bool new_is_int, uint8_t new_pre, uint32_t new_vco,
		bool best_valid, bool best_is_int, uint8_t best_pre, uint32_t best_vco)
{
	if (!new_valid)
		return 0;
	if (!best_valid)
		return 1;

	if (new_is_int && !best_is_int)
		return 1;
	if (!new_is_int && best_is_int)
		return 0;

	if (new_pre < best_pre)
		return 1;
	if (new_pre > best_pre)
		return 0;

	if (new_vco > best_vco)
		return 1;

	return 0;
}

/*
 * soc_dp_solve_pll_frac - Iterative solver for Fractional PLL
 * Searches for optimal Pre/FB/Frac parameters for a target VCO.
 */
static int soc_dp_solve_pll_frac(uint32_t target_vco_khz, uint32_t ref_clk_khz,
		uint8_t *best_pre, uint16_t *best_fb, uint32_t *best_frac)
{
	int pre, found = 0;
	bool best_is_int = false;
	uint64_t min_err = ~0ULL;
	uint64_t vco_int, vco_frac;

	/* Iterate pre-divider 1 to 63 to find best PFD frequency */
	for (pre = 1; pre <= 63; pre++) {
		uint64_t ref_clk_hz = (uint64_t)ref_clk_khz * 1000;
		uint64_t target_vco_hz = (uint64_t)target_vco_khz * 1000;

		/* Calculate required multiplier: Mult = (TargetVCO * Pre) / Ref */
		bool current_is_int;
		uint64_t num = target_vco_hz * pre;
		uint64_t den = ref_clk_hz;
		uint64_t remainder;
		uint64_t fb_val;
		uint64_t frac_val;
		uint64_t actual_vco;
		uint64_t diff;

		fb_val = num;
		remainder = soc_dp_div64(&fb_val, (uint32_t)den);

		if (fb_val > 4095)
			continue;

		/* Frac = (Remainder * 2^24 + Ref/2) / Ref */
		frac_val = remainder * SOC_DP_PLL_FRAC_MOD;
		frac_val += (den / 2);
		soc_dp_div64(&frac_val, (uint32_t)den);

		if (frac_val > 0xFFFFFF)
			frac_val = 0xFFFFFF;

		/*
		 * Calculate actual VCO for error checking.
		 * VCO = (Ref * FB / Pre) + (Ref * Frac / (Pre * 2^24))
		 */

		// Integer part: (Ref * FB) / Pre
		vco_int = ref_clk_hz * fb_val;
		soc_dp_div64(&vco_int, pre);

		// Fractional part: (Ref * Frac) / (Pre * 2^24)
		vco_frac = ref_clk_hz * frac_val;
		soc_dp_div64(&vco_frac, pre);
		soc_dp_div64(&vco_frac, SOC_DP_PLL_FRAC_MOD);

		actual_vco = vco_int + vco_frac;

		diff = soc_dp_abs_diff(actual_vco, target_vco_hz);
		current_is_int = (frac_val == 0);

		if (diff < min_err) {
			min_err = diff;
			*best_pre = pre;
			*best_fb = (uint16_t)fb_val;
			*best_frac = (uint32_t)frac_val;
			best_is_int = current_is_int;
			found = 1;
		} else if (diff == min_err) {
			if (current_is_int && !best_is_int) {
				*best_pre = pre;
				*best_fb = (uint16_t)fb_val;
				*best_frac = (uint32_t)frac_val;
				best_is_int = true;
				found = 1;
			}
		}
	}

	return found ? 0 : -1;
}

/*
 * soc_dp_get_rate_khz - Reverse calculate frequency from parameters
 * Uses stepwise division to prevent 32-bit overflow in do_div base.
 */
static uint32_t soc_dp_get_rate_khz(uint8_t pre, uint16_t fb, uint32_t frac,
		uint32_t ref_clk_khz, uint32_t total_div)
{
	uint64_t vco_hz;
	uint64_t int_part;
	uint64_t frac_part;
	uint64_t ref_hz = (uint64_t)ref_clk_khz * 1000;

	/* Integer part: (Ref * FB) / Pre */
	int_part = ref_hz * fb;
	int_part += (pre / 2);
	soc_dp_div64(&int_part, pre);

	/* Fractional part: (Ref * Frac) / (Pre * 2^24) */
	frac_part = ref_hz * frac;

	frac_part += (pre / 2);
	soc_dp_div64(&frac_part, pre);

	frac_part += (SOC_DP_PLL_FRAC_MOD / 2);
	soc_dp_div64(&frac_part, SOC_DP_PLL_FRAC_MOD);

	vco_hz = int_part + frac_part;

	/* Final Rate = VCO / total_div */
	vco_hz += (total_div / 2); // Rounding before final division
	soc_dp_div64(&vco_hz, total_div);

	vco_hz += 500; // Rounding for 1000
	soc_dp_div64(&vco_hz, 1000);

	return (uint32_t)vco_hz;
}

static int soc_dp_calc_core_pll(uint32_t target_rate_kbps, uint32_t ref_clk_khz, struct soc_dp_core_pll_cfg *cfg)
{
	int i;
	int valid_postdivs[] = {1, 2, 4, 8, 16, 32};
	uint32_t target_bitclk_base = target_rate_kbps / 2;
	struct soc_dp_core_pll_cfg best = {0};

	uint8_t pre;
	uint16_t fb;
	uint32_t frac;

	memset(cfg, 0, sizeof(*cfg));

	for (i = 0; i < 6; i++) {
		struct soc_dp_core_pll_cfg curr = {0};
		int post_div_ratio = valid_postdivs[i];
		uint32_t target_vco = target_bitclk_base * post_div_ratio;

		if (target_vco < SOC_DP_VCO_MIN_KHZ || target_vco > SOC_DP_VCO_MAX_KHZ)
			continue;

		if (soc_dp_solve_pll_frac(target_vco, ref_clk_khz, &pre, &fb, &frac) == 0) {
			uint32_t actual_vco = soc_dp_get_rate_khz(pre, fb, frac, ref_clk_khz, 1);
			uint32_t actual_rate = (actual_vco / post_div_ratio) * 2;

			if (soc_dp_abs_diff(actual_rate, target_rate_kbps) > SOC_DP_PLL_ERR_TOLERANCE)
				continue;

			curr.valid = true;
			curr.vco_freq_khz = target_vco;
			curr.actual_rate_khz = actual_rate;
			curr.prediv = pre;
			curr.fbdiv = fb;
			curr.frac = frac;
			curr.frac_pd = (frac == 0) ? 3 : 0;

			/* Map post_div to register value */
			if (post_div_ratio == 1) {
				curr.postdiv_reg = 0; curr.postdiv_en = 0; curr.vcoclk_div8_en = 0;
			} else {
				if (post_div_ratio == 2)
					curr.postdiv_reg = 0;
				else if (post_div_ratio == 4)
					curr.postdiv_reg = 1;
				else if (post_div_ratio == 8)
					curr.postdiv_reg = 3;
				else if (post_div_ratio == 16)
					curr.postdiv_reg = 5;
				else if (post_div_ratio == 32)
					curr.postdiv_reg = 7;
				else
					curr.postdiv_reg = 0;

				curr.postdiv_en = 1; curr.vcoclk_div8_en = 1;
			}

			curr.clk_16mdiv = (actual_rate / 2) / 64000;

			if (soc_dp_is_better_config(curr.valid, (curr.frac == 0), curr.prediv, curr.vco_freq_khz,
						best.valid, (best.frac == 0), best.prediv, best.vco_freq_khz)) {
				best = curr;
			}
		}
	}

	if (!best.valid)
		return -EINVAL;

	*cfg = best;
	return 0;
}

static void soc_dp_calc_core_pll_to_reg(struct soc_dp_phy_priv *priv, struct soc_dp_core_pll_cfg *cfg)
{
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_PD, 1);
	mdelay(2);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_PREDIV, cfg->prediv);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_FBDIV_LBIT, cfg->fbdiv & 0xFF);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_FBDIV_HBIT, (cfg->fbdiv >> 8) & 0xF);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_DACPD, (cfg->frac_pd >> 1) & 0x1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_DSMPD, cfg->frac_pd & 0x1);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_FRAC_LBIT, cfg->frac & 0xFF);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_FRAC_MBIT, (cfg->frac >> 8) & 0xFF);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_FRAC_HBIT, (cfg->frac >> 16) & 0xFF);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_POSTDIV, cfg->postdiv_reg);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_POSTDIVEN, cfg->postdiv_en);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_VCOCLK_DIV8_EN, cfg->vcoclk_div8_en);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_CLKDIV_16M, cfg->clk_16mdiv);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_PREPLL_LOCK_BYPEN, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_PD, 0);
	mdelay(2);
}

static int soc_dp_calc_pixel_pll(uint32_t target_pclk_khz, uint32_t ref_clk_khz, struct soc_dp_pixel_pll_cfg *cfg)
{
	int aux, pclk_div;
	uint32_t div_total;
	uint32_t target_vco;
	struct soc_dp_pixel_pll_cfg best = {0};
	struct soc_dp_pixel_pll_cfg curr = {0};

	uint8_t pre;
	uint16_t fb;
	uint32_t frac;

	/* Strategy 1: Div5 Path (VCO = PCLK * 5) */
	div_total = 5;
	target_vco = target_pclk_khz * div_total;

	if (target_vco >= SOC_DP_VCO_MIN_KHZ && target_vco <= SOC_DP_VCO_MAX_KHZ) {
		if (soc_dp_solve_pll_frac(target_vco, ref_clk_khz, &pre, &fb, &frac) == 0) {
			uint32_t actual_pclk = soc_dp_get_rate_khz(pre, fb, frac, ref_clk_khz, div_total);

			if (soc_dp_abs_diff(actual_pclk, target_pclk_khz) <= SOC_DP_PLL_ERR_TOLERANCE) {
				curr.valid = true;
				curr.vco_freq_khz = target_vco;
				curr.actual_pclk_khz = actual_pclk;
				curr.prediv = pre; curr.fbdiv = fb; curr.frac = frac;

				curr.frac_pd = (frac == 0) ? 3 : 0;

				curr.div5_en = 1;
				curr.divaux = 0; curr.divm = 0; curr.divp = 0;

				if (soc_dp_is_better_config(curr.valid, (curr.frac == 0), curr.prediv, curr.vco_freq_khz,
							best.valid, (best.frac == 0), best.prediv, best.vco_freq_khz)) {
					best = curr;
				}
			}
		}
	}

	/* Strategy 2 & 3: Iterate PclkDiv (1 to 31) */
	for (pclk_div = 1; pclk_div <= 31; pclk_div++) {
		/* Strategy 2: DivM Path (DivAux = 1) */
		int i;
		int divm_factors[] = {1, 2, 3, 5};
		int divm_regs[]    = {0, 1, 2, 3};

		for (i = 0; i < 4; i++) {
			struct soc_dp_pixel_pll_cfg curr = {0};
			int m_val = divm_factors[i];
			uint32_t div_total = 2 * m_val * pclk_div;
			uint32_t target_vco = target_pclk_khz * div_total;

			if (target_vco < SOC_DP_VCO_MIN_KHZ || target_vco > SOC_DP_VCO_MAX_KHZ)
				continue;

			if (soc_dp_solve_pll_frac(target_vco, ref_clk_khz, &pre, &fb, &frac) == 0) {
				uint32_t actual_pclk = soc_dp_get_rate_khz(pre, fb, frac, ref_clk_khz, div_total);

				if (soc_dp_abs_diff(actual_pclk, target_pclk_khz) > SOC_DP_PLL_ERR_TOLERANCE)
					continue;

				curr.valid = true;
				curr.vco_freq_khz = target_vco;
				curr.actual_pclk_khz = actual_pclk;
				curr.prediv = pre; curr.fbdiv = fb; curr.frac = frac;

				curr.frac_pd = (frac == 0) ? 3 : 0;

				curr.div5_en = 0;
				curr.divaux = 1;         /* Must be 1 to enable DivM logic */
				curr.divm = divm_regs[i];
				curr.divp = pclk_div;

				if (soc_dp_is_better_config(curr.valid, (curr.frac == 0), curr.prediv, curr.vco_freq_khz,
							best.valid, (best.frac == 0), best.prediv, best.vco_freq_khz)) {
					best = curr;
				}
			}
		}

		/* Strategy 3: DivAux Path (DivAux > 1) */
		for (aux = 2; aux <= 31; aux++) {
			struct soc_dp_pixel_pll_cfg curr = {0};
			uint32_t div_total = 2 * aux * pclk_div;
			uint32_t target_vco = target_pclk_khz * div_total;

			if (target_vco < SOC_DP_VCO_MIN_KHZ || target_vco > SOC_DP_VCO_MAX_KHZ)
				continue;

			if (soc_dp_solve_pll_frac(target_vco, ref_clk_khz, &pre, &fb, &frac) == 0) {
				uint32_t actual_pclk = soc_dp_get_rate_khz(pre, fb, frac, ref_clk_khz, div_total);

				if (soc_dp_abs_diff(actual_pclk, target_pclk_khz) > SOC_DP_PLL_ERR_TOLERANCE)
					continue;

				curr.valid = true;
				curr.vco_freq_khz = target_vco;
				curr.actual_pclk_khz = actual_pclk;
				curr.prediv = pre; curr.fbdiv = fb; curr.frac = frac;

				curr.frac_pd = (frac == 0) ? 3 : 0;

				curr.div5_en = 0;
				curr.divaux = aux;
				curr.divm = 0; /* Ignored when divaux != 1 */
				curr.divp = pclk_div;

				if (soc_dp_is_better_config(curr.valid, (curr.frac == 0), curr.prediv, curr.vco_freq_khz,
							best.valid, (best.frac == 0), best.prediv, best.vco_freq_khz)) {
					best = curr;
				}
			}
		}
	}

	if (!best.valid)
		return -EINVAL;

	*cfg = best;
	return 0;
}

static void soc_dp_mst_calc_pixel_pll_to_reg(struct soc_dp_phy_priv *priv,
		uint8_t stream_id, struct soc_dp_pixel_pll_cfg *cfg)
{
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PD, 1);
	mdelay(2);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PREDIV, cfg->prediv);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_FBDIV2_LBIT, cfg->fbdiv & 0xFF);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_FBDIV2_HBIT, (cfg->fbdiv >> 8) & 0xF);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_DACPD, (cfg->frac_pd >> 1) & 0x1);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_DSMPD, cfg->frac_pd & 0x1);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_FRAC2_LBIT, cfg->frac & 0xFF);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_FRAC2_MBIT, (cfg->frac >> 8) & 0xFF);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_FRAC2_HBIT, (cfg->frac >> 16) & 0xFF);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PRECLK_DIVM, cfg->divm);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PRECLK_DIVAUX, cfg->divaux);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PCLKDIV5_EN, cfg->div5_en);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PCLK_DIVAUX, cfg->divp);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_PD, 0);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_REG_PCLK_OUTPUT_NORMAL, 1);
	mdelay(2);
}

static int soc_dp_phy_pixel_clk_apply(struct soc_dp_phy_priv *priv,
		uint8_t stream_id, uint32_t pixel_clk_khz)
{
	int ret;
	struct soc_dp_pixel_pll_cfg cfg = {0};

	dev_info(priv->dev, "Setting Pixel PLL stream %u to %u kHz\n",
			stream_id, pixel_clk_khz);

	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_DP_EN, 1);
	soc_dp_stream_reg_write_range(priv, stream_id, SOC_DPTX_ANA_PREPLL_HDMI_EN, 0);

	ret = soc_dp_calc_pixel_pll(pixel_clk_khz, priv->ref_clk_khz, &cfg);
	if (ret)
		return ret;

	soc_dp_mst_calc_pixel_pll_to_reg(priv, stream_id, &cfg);
	priv->pixel_rate_hz[stream_id] = (unsigned long)cfg.actual_pclk_khz * 1000;

	return 0;
}

static unsigned long soc_dp_phy_pixel_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct soc_dp_phy_pixel_clk *pclk = to_soc_dp_phy_pixel_clk(hw);

	return pclk->priv->pixel_rate_hz[pclk->stream_id];
}

static long soc_dp_phy_pixel_clk_round_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long *parent_rate)
{
	int ret;
	uint32_t target_khz = (uint32_t)(rate / 1000);
	struct soc_dp_phy_pixel_clk *pclk = to_soc_dp_phy_pixel_clk(hw);
	struct soc_dp_pixel_pll_cfg cfg = {0};

	ret = soc_dp_calc_pixel_pll(target_khz, pclk->priv->ref_clk_khz, &cfg);
	if (ret)
		return rate;

	return (unsigned long)cfg.actual_pclk_khz * 1000;
}

static int soc_dp_phy_pixel_clk_set_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long parent_rate)
{
	struct soc_dp_phy_pixel_clk *pclk = to_soc_dp_phy_pixel_clk(hw);

	return soc_dp_phy_pixel_clk_apply(pclk->priv, pclk->stream_id,
			(uint32_t)(rate / 1000));
}

static const struct clk_ops soc_dp_phy_pixel_clk_ops = {
	.recalc_rate = soc_dp_phy_pixel_clk_recalc_rate,
	.round_rate  = soc_dp_phy_pixel_clk_round_rate,
	.set_rate    = soc_dp_phy_pixel_clk_set_rate,
};

static int soc_dp_check_pll_lock(struct soc_dp_phy_priv *priv)
{
	uint32_t pll_locked;

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_reg_read_range(priv, SOC_DPTX_AD_LOCK_PIXELPLL, &pll_locked);
#else
	pll_locked = 1;
#endif
	if (!pll_locked) {
		dev_err(priv->dev, "Pre_pll unlocked\n");
		return -EINVAL;
	}

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	soc_dp_reg_read_range(priv, SOC_DPTX_AD_LOCK_COREPLL, &pll_locked);
#else
	pll_locked = 1;
#endif
	if (!pll_locked) {
		dev_err(priv->dev, "Post_pll unlocked\n");
		return -EINVAL;
	}

	return 0;
}

static int soc_dp_phy_exit(struct phy *phy)
{
	int i;
	struct soc_dp_phy_priv *priv = phy_get_drvdata(phy);

	dev_info(priv->dev, "Disabling PHY Transmitters and Power Down\n");

	soc_dp_reg_write_range(priv, SOC_DPTX_XMIT_ENABLE, 0);
	mdelay(2);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_PD, 1);
	for (i = 0; i < SOC_DP_PHY_MAX_STREAMS; i++)
		soc_dp_stream_reg_write_range(priv, i, SOC_DPTX_ANA_PREPLL_PD, 1);
	mdelay(2);

	return 0;
}

static int soc_dp_phy_power_off(struct phy *phy)
{
	int i;
	struct soc_dp_phy_priv *priv = phy_get_drvdata(phy);

	dev_info(priv->dev, "Disabling PHY Transmitters and Power Down\n");

	soc_dp_reg_write_range(priv, SOC_DPTX_XMIT_ENABLE, 0);
	mdelay(2);

	for (i = 0; i < SOC_DP_PHY_MAX_STREAMS; i++)
		soc_dp_stream_reg_write_range(priv, i, SOC_DPTX_ANA_PREPLL_PD, 1);
	mdelay(2);

	return 0;
}

static int soc_dp_phy_power_on(struct phy *phy)
{
	int ret;
	uint32_t lane_en;
	struct soc_dp_phy_priv *priv = phy_get_drvdata(phy);

	dev_info(priv->dev, "Enabling PHY transmitters and power up\n");

	switch (priv->lane_count) {
	case SOC_DP_LANE_1:
		lane_en = 0x1; break;
	case SOC_DP_LANE_2:
		lane_en = 0x3; break;
	case SOC_DP_LANE_4:
	default:
		lane_en = 0xF; break;
	}

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_PD, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_PREPLL_PD, 0);
	mdelay(2);

	soc_dp_reg_write_range(priv, SOC_DPTX_XMIT_ENABLE, lane_en);
	mdelay(2);

	ret = soc_dp_check_pll_lock(priv);
	if (ret)
		return ret;

	return 0;
}

static void soc_dp_phy_config_lanes(struct soc_dp_phy_priv *priv, int lane_count)
{
	uint32_t phy_lanes_val;

	switch (lane_count) {
	case SOC_DP_LANE_1:
		phy_lanes_val = 0; break;
	case SOC_DP_LANE_2:
		phy_lanes_val = 1; break;
	case SOC_DP_LANE_4:
	default:
		phy_lanes_val = 2; break;
	}

	dev_info(priv->dev, "Configuring PHY Lane Count: %d (Reg: %u)\n",
			lane_count, phy_lanes_val);

	soc_dp_reg_write_range(priv, SOC_DPTX_PHY_NUM_LANES, phy_lanes_val);
	priv->lane_count = lane_count;
}

static int soc_dp_phy_config_rate(struct soc_dp_phy_priv *priv, int rate_khz)
{
	int ret;
	uint32_t rate_val = 0;
	struct soc_dp_core_pll_cfg core_pll_cfg = {0};

	switch (rate_khz) {
	case SOC_DP_LINK_RATE_1_62:
		rate_val = 0; break;
	case SOC_DP_LINK_RATE_2_70:
		rate_val = 1; break;
	case SOC_DP_LINK_RATE_5_40:
		rate_val = 2; break;
	case SOC_DP_LINK_RATE_8_10:
	default:
		rate_val = 3; break;
	}

	soc_dp_reg_write_range(priv, SOC_DPTX_PHY_RATE, rate_val);

	/* Recalculate and set Core PLL */
	dev_info(priv->dev, "Setting Core PLL to Rate %d kHz\n", rate_khz);

	ret = soc_dp_calc_core_pll(rate_khz, priv->ref_clk_khz, &core_pll_cfg);
	if (ret) {
		dev_err(priv->dev, "Failed to calc core PLL\n");
		return ret;
	}

	soc_dp_calc_core_pll_to_reg(priv, &core_pll_cfg);
	priv->link_rate_khz = rate_khz;
	return 0;
}

static void soc_dp_phy_set_voltages(struct soc_dp_phy_priv *priv,
		struct phy_configure_opts_dp *opts)
{
	int i;
	int rate_idx;
	uint32_t swing, preemp;
	const struct soc_dp_phy_vol_setting *cfg;

	/* Determine Rate Index */
	switch (priv->link_rate_khz) {
	case SOC_DP_LINK_RATE_1_62:
		rate_idx = 0;
		break;
	case SOC_DP_LINK_RATE_2_70:
		rate_idx = 1;
		break;
	case SOC_DP_LINK_RATE_5_40:
		rate_idx = 2;
		break;
	case SOC_DP_LINK_RATE_8_10:
		rate_idx = 3;
		break;
	default:
		rate_idx = 0;
		break;
	}

	for (i = 0; i < priv->lane_count; i++) {
		swing = opts->voltage[i] & 0x3;
		preemp = opts->pre[i] & 0x3;

		/* Remap to our table indices */
		swing = phy_swing_map[swing];
		preemp = phy_preemp_map[preemp];

		/* Get configuration parameters */
		cfg = &vol_cfg_table[rate_idx][swing][preemp];

		/* Write to analog registers based on Lane ID */
		switch (i) {
		case 0:
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D0, cfg->isel);
			soc_dp_reg_write_range(priv, SOC_DPTX_DA_TX_MAINSEL_D0_4_0, cfg->mainsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D0, cfg->postsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D0, cfg->presel);
			break;
		case 1:
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D1, cfg->isel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MAINSEL_D1, cfg->mainsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D1, cfg->postsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D1, cfg->presel);
			break;
		case 2:
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D2, cfg->isel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MAINSEL_D2, cfg->mainsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D2, cfg->postsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D2, cfg->presel);
			break;
		case 3:
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D3, cfg->isel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MAINSEL_D3, cfg->mainsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D3, cfg->postsel);
			soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D3, cfg->presel);
			break;
		}
	}
}

static int soc_dp_phy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	int ret;
	struct soc_dp_phy_priv *priv = phy_get_drvdata(phy);
	struct phy_configure_opts_dp *dp_opts = &opts->dp;

	if (dp_opts->set_lanes)
		soc_dp_phy_config_lanes(priv, dp_opts->lanes);

	if (dp_opts->set_rate) {
		ret = soc_dp_phy_config_rate(priv, dp_opts->link_rate * 1000);
		if (ret)
			return ret;
	}

	if (dp_opts->set_voltages)
		soc_dp_phy_set_voltages(priv, dp_opts);

	return 0;
}

static int soc_dp_phy_init(struct phy *phy)
{
	int ret;
	struct soc_dp_phy_priv *priv = phy_get_drvdata(phy);
	uint32_t clk_div;
	uint32_t m_isel = 0x5, m_mainsel = 0xb;
	uint32_t m_pre = 0x0, m_post = 0x2;
	uint32_t tx_mode = 0x1, tx_pre = 0x0;
	union phy_configure_opts phy_opts = {0};

	clk_div = priv->ref_clk_khz / 100;

	soc_dp_phy_exit(phy);

	// Reset Controller and PHY (PHY specific parts)
	soc_dp_reg_write_range(priv, SOC_DPTX_PHY_RESET, 0x1);
	mdelay(5);
	soc_dp_reg_write_range(priv, SOC_DPTX_PHY_RESET, 0x0);
	mdelay(2);

	// Disable PHY SSC (Spread Spectrum Clocking)
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_MPLL_DISABLE_SSCG, 0x1);

	// Bypass PHY busy state
	soc_dp_reg_write_range(priv, SOC_DPTX_PHY_BUSY_BYP, 0x1);

	// Enable Enhance Framing and Scale Down Mode
	soc_dp_reg_write_range(priv, SOC_DPTX_ENHANCE_FRAMING_EN, 0x1);

	phy_opts.dp.lanes = SOC_DP_LANE_2;
	phy_opts.dp.link_rate = SOC_DP_LINK_RATE_1_62 / 1000;
	phy_opts.dp.set_lanes = 1;
	phy_opts.dp.set_rate = 1;
	phy_opts.dp.set_voltages = 1;

	ret = soc_dp_phy_configure(phy, &phy_opts);
	if (ret)
		return ret;

	ret = soc_dp_phy_pixel_clk_apply(priv, 0, 25175);
	if (ret)
		return ret;

	// Analog initialization
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D0, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D1, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D2, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D3, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_RTCAL_FREQDIV_HBIT, (clk_div >> 8) & 0x7f);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_RTCAL_BYPASS, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_RTCAL_FREQDIV_LBIT, clk_div & 0xff);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_BG_RCAL_SEL, 0);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_RTM_D3, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_RTM_D2, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_RTM_D1, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_RTM_D0, 0);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_RTCAL_BYPASS, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_RTCAL_BYPASS, 0);
	msleep(100);

	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_PRE_D3, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_PRE_D2, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_PRE_D1, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_PRE_D0, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_DE_D3, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_DE_D2, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_DE_D1, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_DE_D0, 1);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_PRE_D3, tx_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_PRE_D2, tx_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_PRE_D1, tx_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_PRE_D0, tx_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D3, m_isel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D2, m_isel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MAINSEL_D2, m_mainsel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MAINSEL_D3, m_mainsel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D1, m_isel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_ISEL_DRV_D0, m_isel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D1, m_post);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D0, m_post);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D3, m_post);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_POSTSEL_D2, m_post);
	soc_dp_reg_write_range(priv, SOC_DPTX_DA_TX_MAINSEL_D0_4_0, m_mainsel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MAINSEL_D1, m_mainsel);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D1, m_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D0, m_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D3, m_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_PRESEL_D2, m_pre);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D3, tx_mode);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D2, tx_mode);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D1, tx_mode);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_MODE_D0, tx_mode);
	soc_dp_reg_write_range(priv, SOC_DPTX_ANA_TX_AUX_RX_VSEL, 0x0);

	return 0;
}

static const struct phy_ops soc_dp_phy_ops = {
	.init           = soc_dp_phy_init,
	.exit           = soc_dp_phy_exit,
	.power_on       = soc_dp_phy_power_on,
	.power_off      = soc_dp_phy_power_off,
	.configure      = soc_dp_phy_configure,
	.owner          = THIS_MODULE,
};

/* TSI: see the comment at its one call site in soc_dp_phy_probe(). */
struct soc_dp_phy_lookup {
	struct phy *phy;
	const char *dev_id;
};

static void soc_dp_phy_lookup_remove(void *data)
{
	struct soc_dp_phy_lookup *lookup = data;

	phy_remove_lookup(lookup->phy, "phy", lookup->dev_id);
}

static int soc_dp_phy_probe(struct platform_device *pdev)
{
	int i, ret;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct soc_dp_phy_priv *priv;
	struct phy_provider *phy_provider;
	struct clk_hw_onecell_data *clk_data;
	struct phy *phy;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Failed to obtain dp resource\n");
		return -EINVAL;
	}

#ifndef CONFIG_SOC_DP_DRIVER_QEMU
	priv->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
#else
	priv->regs = devm_kmalloc(dev, resource_size(res), GFP_KERNEL);
#endif
	if (!priv->regs)
		return -ENOMEM;

	if (device_property_read_u32(dev, "ref_clock", &priv->ref_clk_khz)) {
		dev_err(dev, "ref_clock attribute not found, default to use 24M\n");
		priv->ref_clk_khz = 24000;
	}

	phy = devm_phy_create(&pdev->dev, NULL, &soc_dp_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, priv);
	phy_provider = devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	for (i = 0; i < SOC_DP_PHY_MAX_STREAMS; i++) {
		char clk_name[32];

		snprintf(clk_name, sizeof(clk_name), "soc_dp_pixel_clk_%d", i);
		priv->pixel_clks[i].priv = priv;
		priv->pixel_clks[i].stream_id = i;
		priv->pixel_clks[i].hw.init = CLK_HW_INIT_NO_PARENT(clk_name,
				&soc_dp_phy_pixel_clk_ops, 0);

		ret = devm_clk_hw_register(dev, &priv->pixel_clks[i].hw);
		if (ret) {
			dev_err(dev, "Failed to register pixel clock %d: %d\n", i, ret);
			return ret;
		}
	}

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, SOC_DP_PHY_MAX_STREAMS),
			GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = SOC_DP_PHY_MAX_STREAMS;
	for (i = 0; i < SOC_DP_PHY_MAX_STREAMS; i++)
		clk_data->hws[i] = &priv->pixel_clks[i].hw;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (ret) {
		dev_err(dev, "Failed to add clock provider: %d\n", ret);
		return ret;
	}

	/*
	 * TSI: without a devicetree there are no phy/clock phandles. ACPI names
	 * the consumer (_DSD "tsi,consumer", the DP controller's device name) so
	 * phy_get("phy") and clk_get("pixel-N") resolve through lookups instead.
	 */
	if (!dev->of_node) {
		const char *consumer;
		struct soc_dp_phy_lookup *lookup;

		if (device_property_read_string(dev, "tsi,consumer", &consumer))
			return dev_err_probe(dev, -EINVAL,
					     "no tsi,consumer for the ACPI phy/clk lookups\n");
		ret = phy_create_lookup(phy, "phy", consumer);
		if (ret)
			return ret;

		/*
		 * TSI: phy_create_lookup() is a plain kzalloc() on a global,
		 * non-devm list (drivers/phy/phy-core.c); nothing unwinds it
		 * when devm_phy_create()'s phy is freed on unbind, which
		 * leaves a stale pointer for the next phy_find() to return.
		 * Tie the matching phy_remove_lookup() to this device's
		 * teardown. Registered after devm_phy_create()'s own action,
		 * so devm's LIFO order runs this first: the lookup is gone
		 * before the phy it points to is destroyed.
		 */
		lookup = devm_kzalloc(dev, sizeof(*lookup), GFP_KERNEL);
		if (!lookup) {
			phy_remove_lookup(phy, "phy", consumer);
			return -ENOMEM;
		}
		lookup->phy = phy;
		lookup->dev_id = consumer;
		ret = devm_add_action_or_reset(dev, soc_dp_phy_lookup_remove, lookup);
		if (ret)
			return ret;

		for (i = 0; i < SOC_DP_PHY_MAX_STREAMS; i++) {
			char con_id[16];

			snprintf(con_id, sizeof(con_id), "pixel-%d", i);
			ret = devm_clk_hw_register_clkdev(dev, &priv->pixel_clks[i].hw,
							  con_id, consumer);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct of_device_id soc_dp_phy_of_match[] = {
	{ .compatible = "soc,dp-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, soc_dp_phy_of_match);

struct platform_driver soc_dp_phy_driver = {
	.probe  = soc_dp_phy_probe,
	.driver = {
		.name = "soc_dp_phy",
		.of_match_table = soc_dp_phy_of_match,
	},
};

module_platform_driver(soc_dp_phy_driver);
MODULE_LICENSE("GPL");
