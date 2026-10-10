# SPDX-License-Identifier: GPL-2.0-only
# Build the SkyLP UDI SSDT and the enumeration-check init.
IASL ?= iasl
CROSS_COMPILE ?= aarch64-linux-gnu-

all: skylp-udi.aml init

skylp-udi.aml: skylp-udi.asl
	$(IASL) -oa -p skylp-udi $<

init: enum-init.c
	$(CROSS_COMPILE)gcc -static -Os -Wall -o $@ $<

tables.cpio: skylp-udi.aml
	rm -rf tbl && mkdir -p tbl/kernel/firmware/acpi
	cp skylp-udi.aml tbl/kernel/firmware/acpi/
	cd tbl && find kernel | cpio -o -H newc > ../$@

clean:
	rm -rf skylp-udi.aml skylp-udi.hex init tables.cpio tbl

.PHONY: all clean
