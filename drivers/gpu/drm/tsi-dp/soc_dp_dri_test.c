// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the DP link-priority and color-format tables
 * (soc_dp_dri.c), prompted by PR #26 review: enabling the silicon
 * link table (gap G15, CONFIG_SOC_DP_FPGA_LIMITS) changed which
 * configurations the driver will train, so the data it enabled gets
 * validated here.
 *
 * Independent expectations throughout: capacities are the vendor's
 * documented 8b/10b formula (rate * lanes * 8 / 10) written out as
 * literals, bpp values are the DP spec per-format figures, and the
 * tier structure is the vendor's own table comments.
 *
 * Copyright (c) 2026 Tsavorite Scalable Intelligence
 */

#include <kunit/test.h>

#include "soc_dp_dri.h"

/* The vendor's capacity formula (soc_dp_calc_link_capacity). */
static u32 cap(const struct soc_dp_link_config *c)
{
	return ((u32)c->rate * c->lanes * 8) / 10;
}

/*
 * The selection contract of soc_dp_establish_link(): walk the table in
 * order, skip entries the sink cannot do, pick the first whose
 * capacity covers the required bandwidth (pixel_clk_khz * bpp).
 * Returns the picked index, or -1.
 */
static int pick(u32 max_rate, u32 max_lanes, u32 clk_khz, u32 bpp)
{
	unsigned int i;

	for (i = 0; i < soc_dp_link_priority_table_len; i++) {
		const struct soc_dp_link_config *c =
			&soc_dp_link_priority_table[i];

		if ((u32)c->rate > max_rate || (u32)c->lanes > max_lanes)
			continue;
		if (cap(c) >= clk_khz * bpp)
			return i;
	}
	return -1;
}

/*
 * "Priority order" only means anything if the capacities really are
 * non-increasing: an out-of-order entry would make first-fit pick a
 * slower link while a faster one lower down also fits.
 */
static void dp_link_table_sorted_and_unique(struct kunit *test)
{
	unsigned int i, j;

	KUNIT_ASSERT_GT(test, soc_dp_link_priority_table_len, 0u);

	for (i = 1; i < soc_dp_link_priority_table_len; i++)
		KUNIT_EXPECT_LE_MSG(test,
				    cap(&soc_dp_link_priority_table[i]),
				    cap(&soc_dp_link_priority_table[i - 1]),
				    "entry %u out of priority order", i);

	for (i = 0; i < soc_dp_link_priority_table_len; i++)
		for (j = i + 1; j < soc_dp_link_priority_table_len; j++)
			KUNIT_EXPECT_FALSE_MSG(test,
				soc_dp_link_priority_table[i].rate ==
				soc_dp_link_priority_table[j].rate &&
				soc_dp_link_priority_table[i].lanes ==
				soc_dp_link_priority_table[j].lanes,
				"entries %u and %u duplicate", i, j);
}

/*
 * Every entry must be trainable on this PHY: HBR3 (8.1 Gbps) cannot be
 * programmed because its 4.05 GHz base VCO exceeds the PHY's 3 GHz
 * ceiling (the soc_dp_phy.c:397 review finding), so HBR2 is the
 * hardest limit an entry may carry. Lane counts come from the 2-pair
 * bonded package (HW answer, 7 Oct).
 */
static void dp_link_table_within_phy_limits(struct kunit *test)
{
	unsigned int i;

	for (i = 0; i < soc_dp_link_priority_table_len; i++) {
		const struct soc_dp_link_config *c =
			&soc_dp_link_priority_table[i];

		KUNIT_EXPECT_LE_MSG(test, (u32)c->rate, 5400000u,
				    "entry %u beyond HBR2 (PHY VCO ceiling)",
				    i);
		KUNIT_EXPECT_TRUE_MSG(test,
				      c->lanes == SOC_DP_LANE_1 ||
				      c->lanes == SOC_DP_LANE_2 ||
				      c->lanes == SOC_DP_LANE_4,
				      "entry %u bad lane count %u", i,
				      c->lanes);
	}
}

/*
 * The W1.3 (G15) build split: silicon gets the vendor's full 9-entry
 * table headed by HBR2 x4, the FPGA build keeps only the two RBR
 * entries its platform can train.
 */
static void dp_link_table_matches_build(struct kunit *test)
{
#ifdef CONFIG_SOC_DP_FPGA_LIMITS
	KUNIT_EXPECT_EQ(test, soc_dp_link_priority_table_len, 2u);
	KUNIT_EXPECT_EQ(test, soc_dp_link_priority_table[0].rate,
			SOC_DP_LINK_RATE_1_62);
	KUNIT_EXPECT_EQ(test, soc_dp_link_priority_table[0].lanes,
			SOC_DP_LANE_2);
#else
	KUNIT_EXPECT_EQ(test, soc_dp_link_priority_table_len, 9u);
	/* top of the table is the 21.6 Gbps config ... */
	KUNIT_EXPECT_EQ(test, soc_dp_link_priority_table[0].rate,
			SOC_DP_LINK_RATE_5_40);
	KUNIT_EXPECT_EQ(test, soc_dp_link_priority_table[0].lanes,
			SOC_DP_LANE_4);
	/* ... and the floor is RBR x1 */
	KUNIT_EXPECT_EQ(test,
			soc_dp_link_priority_table[8].rate,
			SOC_DP_LINK_RATE_1_62);
	KUNIT_EXPECT_EQ(test,
			soc_dp_link_priority_table[8].lanes,
			SOC_DP_LANE_1);
#endif
}

/* Per-format bpp against the DP spec figures, plus the documented
 * out-of-range fallback to RGB888.
 */
static void dp_bpp_table_matches_spec(struct kunit *test)
{
	static const struct { u32 fmt; int bpp; } expect[] = {
		{ SOC_VIDEO_RGB_6BIT,	  18 },
		{ SOC_VIDEO_RGB_8BIT,	  24 },
		{ SOC_VIDEO_RGB_10BIT,	  30 },
		{ SOC_VIDEO_RGB_12BIT,	  36 },
		{ SOC_VIDEO_RGB_16BIT,	  48 },
		{ SOC_VIDEO_YUV444_8BIT,  24 },
		{ SOC_VIDEO_YUV444_10BIT, 30 },
		{ SOC_VIDEO_YUV444_12BIT, 36 },
		{ SOC_VIDEO_YUV444_16BIT, 48 },
		{ SOC_VIDEO_YUV422_8BIT,  16 },
		{ SOC_VIDEO_YUV422_10BIT, 20 },
		{ SOC_VIDEO_YUV422_12BIT, 24 },
		{ SOC_VIDEO_YUV422_16BIT, 32 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(expect); i++)
		KUNIT_EXPECT_EQ_MSG(test, soc_dp_get_bpp(expect[i].fmt),
				    expect[i].bpp, "format %u", expect[i].fmt);

	/* out of range falls back to RGB888, never to a zero divisor */
	KUNIT_EXPECT_EQ(test, soc_dp_get_bpp(ARRAY_SIZE(expect)), 24);
	KUNIT_EXPECT_EQ(test, soc_dp_get_bpp(0xffffffff), 24);
}

/*
 * First-fit selection over the real table, with the bandwidth math as
 * independent literals: 1080p60 is 148500 kHz, so 24 bpp needs
 * 3,564,000 (kHz*bpp); HBR x2 carries 4,320,000 and RBR x2 only
 * 2,592,000. 4k60 (533250 kHz, 24 bpp) needs 12,798,000, which only
 * HBR2 x4 (17,280,000) covers.
 */
static void dp_first_fit_selection(struct kunit *test)
{
#ifdef CONFIG_SOC_DP_FPGA_LIMITS
	/* 640x480@60 (25175 kHz) fits RBR x2; 1080p does not fit at all */
	KUNIT_EXPECT_EQ(test, pick(5400000, 4, 25175, 24), 0);
	KUNIT_EXPECT_EQ(test, pick(5400000, 4, 148500, 24), -1);
#else
	/* an unrestricted sink always gets the top entry */
	KUNIT_EXPECT_EQ(test, pick(5400000, 4, 25175, 24), 0);
	/* a HBR x2 sink showing 1080p lands exactly on HBR x2 */
	KUNIT_EXPECT_EQ(test, pick(2700000, 2, 148500, 24), 4);
	KUNIT_EXPECT_EQ(test,
			soc_dp_link_priority_table[4].rate,
			SOC_DP_LINK_RATE_2_70);
	KUNIT_EXPECT_EQ(test,
			soc_dp_link_priority_table[4].lanes,
			SOC_DP_LANE_2);
	/* RBR x2 cannot carry 1080p/24bpp: first fit must skip past it */
	KUNIT_EXPECT_NE(test, pick(5400000, 4, 148500, 24), -1);
	KUNIT_EXPECT_LT(test, pick(5400000, 4, 148500, 24), 6);
	/* 4k60 needs HBR2 x4; a sink capped at HBR x4 has no config */
	KUNIT_EXPECT_EQ(test, pick(5400000, 4, 533250, 24), 0);
	KUNIT_EXPECT_EQ(test, pick(2700000, 4, 533250, 24), -1);
#endif
}

static struct kunit_case soc_dp_dri_cases[] = {
	KUNIT_CASE(dp_link_table_sorted_and_unique),
	KUNIT_CASE(dp_link_table_within_phy_limits),
	KUNIT_CASE(dp_link_table_matches_build),
	KUNIT_CASE(dp_bpp_table_matches_spec),
	KUNIT_CASE(dp_first_fit_selection),
	{}
};

static struct kunit_suite soc_dp_dri_suite = {
	.name = "soc-dp-dri-tables",
	.test_cases = soc_dp_dri_cases,
};
kunit_test_suite(soc_dp_dri_suite);
