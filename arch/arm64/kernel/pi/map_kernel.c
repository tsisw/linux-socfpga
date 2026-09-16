// SPDX-License-Identifier: GPL-2.0-only
// Copyright 2023 Google LLC
// Author: Ard Biesheuvel <ardb@google.com>

#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/linkage.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/string.h>

#include <asm/memory.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>

#include "pi.h"

extern const u8 __eh_frame_start[], __eh_frame_end[];

extern void idmap_cpu_replace_ttbr1(void *pgdir);

static void __init map_segment(pgd_t *pg_dir, u64 *pgd, u64 va_offset,
			       void *start, void *end, pgprot_t prot,
			       bool may_use_cont, int root_level)
{
	map_range(pgd, ((u64)start + va_offset) & ~PAGE_OFFSET,
		  ((u64)end + va_offset) & ~PAGE_OFFSET, (u64)start,
		  prot, root_level, (pte_t *)pg_dir, may_use_cont, 0);
}

static void __init unmap_segment(pgd_t *pg_dir, u64 va_offset, void *start,
				 void *end, int root_level)
{
	map_segment(pg_dir, NULL, va_offset, start, end, __pgprot(0),
		    false, root_level);
}

static void __init map_kernel(u64 kaslr_offset, u64 va_offset, int root_level)
{
	bool enable_scs = IS_ENABLED(CONFIG_UNWIND_PATCH_PAC_INTO_SCS);
	bool twopass = IS_ENABLED(CONFIG_RELOCATABLE);
	u64 pgdp = (u64)init_pg_dir + PAGE_SIZE;
	pgprot_t text_prot = PAGE_KERNEL_ROX;
	pgprot_t data_prot = PAGE_KERNEL;
	pgprot_t prot;

	/*
	 * External debuggers may need to write directly to the text mapping to
	 * install SW breakpoints. Allow this (only) when explicitly requested
	 * with rodata=off.
	 */
	if (arm64_test_sw_feature_override(ARM64_SW_FEATURE_OVERRIDE_RODATA_OFF))
		text_prot = PAGE_KERNEL_EXEC;

	/*
	 * We only enable the shadow call stack dynamically if we are running
	 * on a system that does not implement PAC or BTI. PAC and SCS provide
	 * roughly the same level of protection, and BTI relies on the PACIASP
	 * instructions serving as landing pads, preventing us from patching
	 * those instructions into something else.
	 */
	if (IS_ENABLED(CONFIG_ARM64_PTR_AUTH_KERNEL) && cpu_has_pac())
		enable_scs = false;

	if (IS_ENABLED(CONFIG_ARM64_BTI_KERNEL) && cpu_has_bti()) {
		enable_scs = false;

		/*
		 * If we have a CPU that supports BTI and a kernel built for
		 * BTI then mark the kernel executable text as guarded pages
		 * now so we don't have to rewrite the page tables later.
		 */
		text_prot = __pgprot_modify(text_prot, PTE_GP, PTE_GP);
	}

	/* Map all code read-write on the first pass if needed */
	twopass |= enable_scs;
	prot = twopass ? data_prot : text_prot;

	map_segment(init_pg_dir, &pgdp, va_offset, _stext, _etext, prot,
		    !twopass, root_level);
	map_segment(init_pg_dir, &pgdp, va_offset, __start_rodata,
		    __inittext_begin, data_prot, false, root_level);
	map_segment(init_pg_dir, &pgdp, va_offset, __inittext_begin,
		    __inittext_end, prot, false, root_level);
	map_segment(init_pg_dir, &pgdp, va_offset, __initdata_begin,
		    __initdata_end, data_prot, false, root_level);
	map_segment(init_pg_dir, &pgdp, va_offset, _data, _end, data_prot,
		    true, root_level);
	dsb(ishst);

	idmap_cpu_replace_ttbr1(init_pg_dir);

	if (twopass) {
		if (IS_ENABLED(CONFIG_RELOCATABLE))
			relocate_kernel(kaslr_offset);

		if (enable_scs) {
			scs_patch(__eh_frame_start + va_offset,
				  __eh_frame_end - __eh_frame_start);
			asm("ic ialluis");

			dynamic_scs_is_enabled = true;
		}

		/*
		 * Unmap the text region before remapping it, to avoid
		 * potential TLB conflicts when creating the contiguous
		 * descriptors.
		 */
		unmap_segment(init_pg_dir, va_offset, _stext, _etext,
			      root_level);
		dsb(ishst);
		isb();
		__tlbi(vmalle1);
		isb();

		/*
		 * Remap these segments with different permissions
		 * No new page table allocations should be needed
		 */
		map_segment(init_pg_dir, NULL, va_offset, _stext, _etext,
			    text_prot, true, root_level);
		map_segment(init_pg_dir, NULL, va_offset, __inittext_begin,
			    __inittext_end, text_prot, false, root_level);
	}

	/* Copy the root page table to its final location */
	memcpy((void *)swapper_pg_dir + va_offset, init_pg_dir, PAGE_SIZE);
	dsb(ishst);
	idmap_cpu_replace_ttbr1(swapper_pg_dir);
}

static void noinline __section(".idmap.text") set_ttbr0_for_lpa2(u64 ttbr)
{
	u64 sctlr = read_sysreg(sctlr_el1);
	u64 tcr = read_sysreg(tcr_el1) | TCR_DS;
	u64 mmfr0 = read_sysreg(id_aa64mmfr0_el1);
	u64 parange = cpuid_feature_extract_unsigned_field(mmfr0,
							   ID_AA64MMFR0_EL1_PARANGE_SHIFT);

	tcr &= ~TCR_IPS_MASK;
	tcr |= parange << TCR_IPS_SHIFT;

	asm("	msr	sctlr_el1, %0		;"
	    "	isb				;"
	    "   msr     ttbr0_el1, %1		;"
	    "   msr     tcr_el1, %2		;"
	    "	isb				;"
	    "	tlbi    vmalle1			;"
	    "	dsb     nsh			;"
	    "	isb				;"
	    "	msr     sctlr_el1, %3		;"
	    "	isb				;"
	    ::	"r"(sctlr & ~SCTLR_ELx_M), "r"(ttbr), "r"(tcr), "r"(sctlr));
}

static void __init remap_idmap_for_lpa2(void)
{
	/* clear the bits that change meaning once LPA2 is turned on */
	pteval_t mask = PTE_SHARED;

	/*
	 * We have to clear bits [9:8] in all block or page descriptors in the
	 * initial ID map, as otherwise they will be (mis)interpreted as
	 * physical address bits once we flick the LPA2 switch (TCR.DS). Since
	 * we cannot manipulate live descriptors in that way without creating
	 * potential TLB conflicts, let's create another temporary ID map in a
	 * LPA2 compatible fashion, and update the initial ID map while running
	 * from that.
	 */
	create_init_idmap(init_pg_dir, mask);
	dsb(ishst);
	set_ttbr0_for_lpa2((u64)init_pg_dir);

	/*
	 * Recreate the initial ID map with the same granularity as before.
	 * Don't bother with the FDT, we no longer need it after this.
	 */
	memset(init_idmap_pg_dir, 0,
	       (u64)init_idmap_pg_end - (u64)init_idmap_pg_dir);

	create_init_idmap(init_idmap_pg_dir, mask);
	dsb(ishst);

	/* switch back to the updated initial ID map */
	set_ttbr0_for_lpa2((u64)init_idmap_pg_dir);

	/* wipe the temporary ID map from memory */
	memset(init_pg_dir, 0, (u64)init_pg_end - (u64)init_pg_dir);
}

static void __init map_fdt(u64 fdt)
{
	static u8 ptes[INIT_IDMAP_FDT_SIZE] __initdata __aligned(PAGE_SIZE);
	u64 efdt = fdt + MAX_FDT_SIZE;
	u64 ptep = (u64)ptes;

	/*
	 * Map up to MAX_FDT_SIZE bytes, but avoid overlap with
	 * the kernel image.
	 */
	map_range(&ptep, fdt, (u64)_text > fdt ? min((u64)_text, efdt) : efdt,
		  fdt, PAGE_KERNEL, IDMAP_ROOT_LEVEL,
		  (pte_t *)init_idmap_pg_dir, false, 0);
	dsb(ishst);
}

/*
 * A trail mark from position-independent code: the same store-and-clean as
 * asm/tsi_diag.h, written inline because this object is renamed to __pi_* and
 * cannot call into the rest of the kernel.
 */
static void __init tsi_pi_mark(int id)
{
	asm volatile(
	"	adrp	x16, tsi_trail_idx		\n"
	"	add	x16, x16, :lo12:tsi_trail_idx	\n"
	"	ldr	x17, [x16]			\n"
	"	add	x17, x17, #1			\n"
	"	and	x17, x17, #0xff			\n"
	"	str	x17, [x16]			\n"
	"	dsb	sy				\n"
	"	dc	civac, x16			\n"
	"	adrp	x16, tsi_trail			\n"
	"	add	x16, x16, :lo12:tsi_trail	\n"
	"	add	x16, x16, x17			\n"
	"	strb	%w0, [x16]			\n"
	"	dsb	sy				\n"
	"	dc	civac, x16			\n"
	"	dsb	sy				\n"
	: : "r"(id) : "x16", "x17", "memory");
}

/*
 * Stash the registers that decide how memset behaves, so the host can read them
 * instead of us guessing at what the core reports.
 */
static void __init tsi_pi_info(void)
{
	asm volatile(
	"	adrp	x16, tsi_info			\n"
	"	add	x16, x16, :lo12:tsi_info	\n"
	"	mrs	x17, dczid_el0			\n"
	"	str	x17, [x16, #0]			\n"
	"	mrs	x17, ctr_el0			\n"
	"	str	x17, [x16, #8]			\n"
	"	mrs	x17, midr_el1			\n"
	"	str	x17, [x16, #16]			\n"
	"	mrs	x17, sctlr_el1			\n"
	"	str	x17, [x16, #24]			\n"
	"	mrs	x17, tcr_el1			\n"
	"	str	x17, [x16, #32]			\n"
	"	mrs	x17, ttbr0_el1			\n"
	"	str	x17, [x16, #40]			\n"
	"	mrs	x17, id_aa64mmfr0_el1		\n"
	"	str	x17, [x16, #48]			\n"
	"	mov	x17, #0x4649			\n"
	"	movk	x17, #0x5453, lsl #16		\n"
	"	str	x17, [x16, #56]			\n"
	"	dsb	sy				\n"
	"	dc	civac, x16			\n"
	"	add	x16, x16, #56			\n"
	"	dc	civac, x16			\n"
	"	dsb	sy				\n"
	: : : "x16", "x17", "memory");
}

asmlinkage void __init early_map_kernel(u64 boot_status, void *fdt)
{
	static char const chosen_str[] __initconst = "/chosen";
	u64 va_base, pa_base = (u64)&_text;
	u64 kaslr_offset = pa_base % MIN_KIMG_ALIGN;
	int root_level = 4 - CONFIG_PGTABLE_LEVELS;
	int va_bits = VA_BITS;
	int chosen;

	tsi_pi_mark(0x40);
	map_fdt((u64)fdt);
	tsi_pi_mark(0x41);

	/*
	 * Clear BSS and the initial page tables. Written as a marked loop of plain
	 * stores rather than one memset call: run 11 hung somewhere in this clear
	 * with no exception, and the library memset uses DC ZVA, so this separates
	 * "the memory will not take stores" from "that instruction does not work".
	 * The trail lives in .data, above __bss_start, so the clear cannot erase it.
	 */
	tsi_pi_info();
	{
		u64 s = (u64)__bss_start, e = (u64)init_pg_end;
		int chunk = 0;

		while (s < e) {
			u64 n = (e - s) > (64 * 1024) ? (64 * 1024) : (e - s);
			volatile u64 *p = (volatile u64 *)s;
			volatile u64 *q = (volatile u64 *)(s + (n & ~7UL));

			while (p < q)
				*p++ = 0;
			if (n & 7) {
				volatile u8 *b = (volatile u8 *)q;

				while ((u64)b < s + n)
					*b++ = 0;
			}
			tsi_pi_mark(0x50 + chunk);
			chunk++;
			s += n;
		}
	}
	tsi_pi_mark(0x42);

	/*
	 * Now exercise the library memset on a range we just cleared by hand. If the
	 * trail stops between these two marks, DC ZVA is the thing that hangs.
	 */
	tsi_pi_mark(0x60);
	memset(__bss_start, 0, 64 * 1024);
	tsi_pi_mark(0x61);

	/* Parse the command line for CPU feature overrides */
	chosen = fdt_path_offset(fdt, chosen_str);
	tsi_pi_mark(0x43);
	init_feature_override(boot_status, fdt, chosen);
	tsi_pi_mark(0x44);

	if (IS_ENABLED(CONFIG_ARM64_64K_PAGES) && !cpu_has_lva()) {
		va_bits = VA_BITS_MIN;
	} else if (IS_ENABLED(CONFIG_ARM64_LPA2) && !cpu_has_lpa2()) {
		va_bits = VA_BITS_MIN;
		root_level++;
	}

	tsi_pi_mark(0x45);
	if (va_bits > VA_BITS_MIN)
		sysreg_clear_set(tcr_el1, TCR_T1SZ_MASK, TCR_T1SZ(va_bits));
	tsi_pi_mark(0x46);

	/*
	 * The virtual KASLR displacement modulo 2MiB is decided by the
	 * physical placement of the image, as otherwise, we might not be able
	 * to create the early kernel mapping using 2 MiB block descriptors. So
	 * take the low bits of the KASLR offset from the physical address, and
	 * fill in the high bits from the seed.
	 */
	if (IS_ENABLED(CONFIG_RANDOMIZE_BASE)) {
		u64 kaslr_seed = kaslr_early_init(fdt, chosen);

		if (kaslr_seed && kaslr_requires_kpti())
			arm64_use_ng_mappings = true;

		kaslr_offset |= kaslr_seed & ~(MIN_KIMG_ALIGN - 1);
	}

	if (IS_ENABLED(CONFIG_ARM64_LPA2) && va_bits > VA_BITS_MIN)
		remap_idmap_for_lpa2();
	tsi_pi_mark(0x47);

	va_base = KIMAGE_VADDR + kaslr_offset;
	map_kernel(kaslr_offset, va_base - pa_base, root_level);
	tsi_pi_mark(0x48);
}
