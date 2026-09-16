/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TSI pro-FPGA bring-up diagnostics: see arch/arm64/kernel/tsi_diag.S.
 * Every store is pushed to the point of coherency, because the host reads
 * these through a DDR backdoor that does not see the CPU caches.
 */
#ifndef __ASM_TSI_DIAG_H
#define __ASM_TSI_DIAG_H

#ifdef __ASSEMBLY__

	.macro	tsi_mark, id
	adr_l	x16, tsi_trail_idx
	ldr	x17, [x16]
	add	x17, x17, #1
	and	x17, x17, #0xff
	str	x17, [x16]
	dsb	sy
	dc	civac, x16
	adr_l	x16, tsi_trail
	add	x16, x16, x17
	mov	w17, #\id
	strb	w17, [x16]
	dsb	sy
	dc	civac, x16
	dsb	sy
	.endm

/*
 * Record one exception into \rec (8 x 64 bits) and park. Used for the EL1 and
 * EL2 windows where the kernel has not installed its own vectors yet, so a
 * fault would otherwise vanish into an unset VBAR.
 */
	.macro	tsi_rec_vec, rec, idx, el, marker
	mrs	x0, esr_\el
	mrs	x1, elr_\el
	mrs	x2, far_\el
	mrs	x3, spsr_\el
	adr_l	x4, \rec
	str	x0, [x4, #0]
	str	x1, [x4, #8]
	str	x2, [x4, #16]
	str	x3, [x4, #24]
	mov	x5, #\idx
	str	x5, [x4, #32]
	mrs	x5, sctlr_\el
	str	x5, [x4, #40]
	mrs	x5, CurrentEL
	str	x5, [x4, #48]
	movz	x5, #(\marker & 0xffff)
	movk	x5, #(\marker >> 16), lsl #16
	str	x5, [x4, #56]
	dsb	sy
	dc	civac, x4
	add	x4, x4, #56
	dc	civac, x4
	dsb	sy
.Ltsi_park_\el\()_\idx:
	wfi
	b	.Ltsi_park_\el\()_\idx
	.endm

#else

void tsi_mark_c(int id);

#endif /* __ASSEMBLY__ */
#endif /* __ASM_TSI_DIAG_H */
