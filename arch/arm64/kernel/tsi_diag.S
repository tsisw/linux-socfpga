/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI pro-FPGA bring-up diagnostics.
 *
 * The board has no working console for the kernel yet and the host can only
 * look at DDR through a backdoor while the core is stopped. Anything the
 * kernel writes after it turns the MMU and caches on can sit in a cache and
 * never reach DDR, so a plain memory footprint proves nothing once the MMU is
 * up. Everything here therefore pushes each store to the point of coherency
 * before moving on, and lives inside the kernel image so it is addressable
 * with the MMU off (the identity map) and with it on (the kernel map).
 *
 * Two things are recorded:
 *   tsi_trail      a byte per checkpoint, in order, so the last mark written
 *                  says exactly how far the kernel got.
 *   tsi_el1_rec    the first exception taken at EL1 while the kernel's own
 *   tsi_el2_rec    vectors are not installed yet. Between the eret at the end
 *                  of init_kernel_el and the "msr vbar_el1" in
 *                  __primary_switched, VBAR_EL1 still holds its reset value,
 *                  so a fault there jumps into the weeds and loops forever
 *                  with nothing to show for it. tsi_el1_vectors is installed
 *                  over that window instead; hyp-stub.S covers the EL2 side.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/tsi_diag.h>

	.section ".data", "aw"
	.balign	64
	.globl	tsi_trail
tsi_trail:
	.space	256, 0
	.balign	64
	.globl	tsi_trail_idx
tsi_trail_idx:
	.quad	0
	.balign	64
	.globl	tsi_el1_rec
tsi_el1_rec:
	.space	64, 0
	.balign	64
	.globl	tsi_el2_rec
tsi_el2_rec:
	.space	64, 0
	.balign	64
	.globl	tsi_info
tsi_info:
	.space	64, 0

	.section ".text", "ax"

/* the same, with the mark in w0, for callers written in C */
SYM_FUNC_START(tsi_mark_c)
	adr_l	x16, tsi_trail_idx
	ldr	x17, [x16]
	add	x17, x17, #1
	and	x17, x17, #0xff
	str	x17, [x16]
	dsb	sy
	dc	civac, x16
	adr_l	x16, tsi_trail
	add	x16, x16, x17
	strb	w0, [x16]
	dsb	sy
	dc	civac, x16
	dsb	sy
	ret
SYM_FUNC_END(tsi_mark_c)

/*
 * Recording vector table for EL1, installed over the window in which the
 * kernel has no vectors of its own. Marker "TSI1" says the record is real, so
 * an all-zero tsi_el1_rec means no EL1 exception was taken.
 */
	.macro	tsi_vec_el1, idx
	.balign	0x80
	tsi_rec_vec	tsi_el1_rec, \idx, el1, 0x54534931
	.endm

	/* VBAR_EL1 ignores bits [10:0], so the table itself must be 2 KiB aligned;
	 * aligning after the label would silently point VBAR at the wrong address. */
	.balign	0x800
SYM_CODE_START(tsi_el1_vectors)
	tsi_vec_el1	0		/* current EL, SP_EL0, sync  */
	tsi_vec_el1	1		/* current EL, SP_EL0, irq   */
	tsi_vec_el1	2		/* current EL, SP_EL0, fiq   */
	tsi_vec_el1	3		/* current EL, SP_EL0, serr  */
	tsi_vec_el1	4		/* current EL, SP_ELx, sync  */
	tsi_vec_el1	5		/* current EL, SP_ELx, irq   */
	tsi_vec_el1	6		/* current EL, SP_ELx, fiq   */
	tsi_vec_el1	7		/* current EL, SP_ELx, serr  */
	tsi_vec_el1	8		/* lower EL, aarch64, sync   */
	tsi_vec_el1	9		/* lower EL, aarch64, irq    */
	tsi_vec_el1	10		/* lower EL, aarch64, fiq    */
	tsi_vec_el1	11		/* lower EL, aarch64, serr   */
	tsi_vec_el1	12		/* lower EL, aarch32, sync   */
	tsi_vec_el1	13		/* lower EL, aarch32, irq    */
	tsi_vec_el1	14		/* lower EL, aarch32, fiq    */
	tsi_vec_el1	15		/* lower EL, aarch32, serr   */
SYM_CODE_END(tsi_el1_vectors)
