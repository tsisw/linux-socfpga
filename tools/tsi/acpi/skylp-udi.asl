/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * TSI SkyLP: ACPI description of the Innosilicon UDI subsystem (USB 3.2 DRD,
 * DisplayPort TX, video bridge) and its interrupt collector, for one chiplet.
 *
 * This SSDT is the firmware-side contract the UEFI BIOS must provide. Until
 * the BIOS carries it, the same AML can be injected at boot through the
 * kernel's ACPI table upgrade (CONFIG_ACPI_TABLE_UPGRADE): place skylp-udi.aml
 * in an uncompressed cpio at kernel/firmware/acpi/ prepended to the initrd.
 *
 * Driver matching uses the PRP0001 mechanism: each device's _DSD carries the
 * same "compatible" string as the devicetree binding, so the existing
 * of_match_table entries match under ACPI and the same property names apply
 * (Documentation/firmware-guide/acpi/enumeration.rst, "PRP0001").
 *
 * Addresses are the LP0 chiplet's (databook Table 19, RAL). LP1 (the chiplet
 * that carries the J43 DP receptacle on Helix-M) needs its own copy of the
 * block with its base and interrupt rollup once HW-35 is answered.
 *
 * Interrupt numbers:
 *  - The collector's own line is GIC SPI 76 = GSI 108 (HW-1, GPP group 0).
 *  - Children name the collector as ResourceSource; the number is then the
 *    udi_intr bit, translated by the collector's irq domain:
 *      0 usb, 1 dptx, 2/3 video bridge 0/1 frame done, 4/5 underflow, 6/7 audio.
 *
 * Resource ordering for the USB node (ACPI resources are unnamed, so the
 * driver takes them by index): 0 controller window, 1 lane mux word,
 * 2 tsar_control, 3 udi_clk_sel.
 */
DefinitionBlock ("skylp-udi.aml", "SSDT", 2, "TSI   ", "SKYLPUDI", 0x00000001)
{
    External (\_SB, DeviceObj)

    Scope (\_SB)
    {
        Device (LP0)
        {
            Name (_HID, "TSI00000")            /* SkyLP chiplet 0 container */
            Name (_UID, 0)
            Name (_CCA, 0)                      /* UDI DMA is not coherent (HW-12) */

            /* ---- udi_intr collector, destination group 0 --------------- */
            Device (INTC)
            {
                Name (_HID, "PRP0001")
                Name (_UID, 0)
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite, 0x24000108, 0x00000040)
                    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 108 }
                })
                Name (_DSD, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "compatible", "tsi,skylp-intc" },
                        Package () { "interrupt-controller", 1 },
                        Package () { "#interrupt-cells", 2 },
                        Package () { "tsi,dest-group", 0 },
                        Package () { "tsi,num-sources", 8 },
                    }
                })
            }

            /* ---- USB 3.2 DRD controller + SkyLP init hooks ---------------- */
            Device (USB0)
            {
                Name (_HID, "TSI00001")
                Name (_CID, "PRP0001")
                Name (_UID, 0)
                Name (_CCA, 0)
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite, 0x24000150, 0x00010000)   /* 0: controller */
                    Memory32Fixed (ReadWrite, 0x24021714, 0x00000004)   /* 1: lane mux */
                    Memory32Fixed (ReadWrite, 0x24000008, 0x00000004)   /* 2: tsar_control */
                    Memory32Fixed (ReadWrite, 0x2400014C, 0x00000004)   /* 3: udi_clk_sel */
                    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, 0,
                               "\\_SB.LP0.INTC", ) { 0 }
                })
                Name (_DSD, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "compatible", Package () { "tsi,skylp-usb", "soc,soc_drd3" } },
                        Package () { "dr_mode", "host" },
                        Package () { "maximum-speed", "super-speed-plus" },
                        Package () { "tsi,mux-mode", 1 },              /* USB3.2 lanes */
                        /* tsi,tsar-init / tsi,clksel-init: values pending HW-8/HW-4 bit maps */
                    }
                })
            }

            /* ---- Type-C plug/orientation glue (lane mux owner) ------------- */
            Device (TYPC)
            {
                Name (_HID, "TSI00002")
                Name (_CID, "PRP0001")
                Name (_UID, 0)
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite, 0x24021714, 0x00000004)
                })
                Name (_DSD, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "compatible", "tsi,skylp-typec" },
                        Package () { "tsi,mux-mode", 1 },
                        /* plug-event-gpios / plug-flip-gpios: need the SkyLP GPIO
                         * controller described in ACPI first (pinctrl-tsi). */
                    }
                })
            }

            /* ---- DisplayPort PHY (Comb PHY, pixel clock provider) ----------- */
            Device (DPHY)
            {
                Name (_HID, "TSI00004")
                Name (_CID, "PRP0001")
                Name (_UID, 0)
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite, 0x24020150, 0x00010000)
                })
                Name (_DSD, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "compatible", "soc,dp-phy" },
                        Package () { "ref_clock", 50000 },              /* kHz, HW-4 */
                        /* ACPI has no phy/clock phandles: name the consumer so the
                         * PHY driver can register "phy" and "pixel-N" lookups. */
                        Package () { "tsi,consumer", "TSI00003:00" },
                    }
                })
            }

            /* ---- DisplayPort TX controller (DRM encoder + connector) -------- */
            Device (DP00)
            {
                Name (_HID, "TSI00003")
                Name (_CID, "PRP0001")
                Name (_UID, 0)
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite, 0x24050150, 0x00040000)
                    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, 0,
                               "\\_SB.LP0.INTC", ) { 1 }
                })
                Name (_DSD, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "compatible", "soc,dp" },
                        Package () { "only-sst", 1 },
                    },
                    ToUUID ("dbb8e3e6-5886-4ba6-8795-1319f52a966b"),
                    Package ()
                    {
                        Package () { "port@0", "PRT0" },
                    }
                })
                Name (PRT0, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "port", 0 },
                    },
                    ToUUID ("dbb8e3e6-5886-4ba6-8795-1319f52a966b"),
                    Package ()
                    {
                        Package () { "endpoint@0", "EP00" },
                    }
                })
                Name (EP00, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "reg", 0 },
                        Package () { "remote-endpoint", "\\_SB.LP0.VBR0.PRT0.EP00" },
                    }
                })
            }

            /* ---- Video bridge 0 = DRM master, CRTC and primary plane -------- */
            Device (VBR0)
            {
                Name (_HID, "TSI00005")
                Name (_CID, "PRP0001")
                Name (_UID, 0)
                Name (_CCA, 0)
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite, 0x24010550, 0x00000400)
                    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, 0,
                               "\\_SB.LP0.INTC", ) { 2 }
                })
                Name (_DSD, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "compatible", "tsi,skylp-video-bridge" },
                    },
                    ToUUID ("dbb8e3e6-5886-4ba6-8795-1319f52a966b"),
                    Package ()
                    {
                        Package () { "port@0", "PRT0" },
                    }
                })
                Name (PRT0, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "port", 0 },
                    },
                    ToUUID ("dbb8e3e6-5886-4ba6-8795-1319f52a966b"),
                    Package ()
                    {
                        Package () { "endpoint@0", "EP00" },
                    }
                })
                Name (EP00, Package ()
                {
                    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                    Package ()
                    {
                        Package () { "reg", 0 },
                        Package () { "remote-endpoint", "\\_SB.LP0.DP00.PRT0.EP00" },
                    }
                })
            }
        }
    }
}
