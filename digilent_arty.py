#!/usr/bin/env python3

# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex_boards.platforms import digilent_arty
from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.uart import *

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, with_rst=True):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        # Clk/Rst.
        clk100 = platform.request("clk100")
        rst    = ~platform.request("cpu_reset") if with_rst else 0

        # PLL.
        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(rst | self.rst)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    mem_map = {**SoCCore.mem_map, **{
        "csr": 0x10000000,
    }}
    def __init__(self, variant="a7-35", toolchain="vivado", sys_clk_freq=int(100e6), with_led_chaser=True):
        platform = digilent_arty.Platform(variant=variant, toolchain=toolchain)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteX standalone SoC generator on Arty A7")

        # JTAGBone ---------------------------------------------------------------------------------
        self.add_jtagbone()

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.submodules.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)


        # Standalone SoC Generation/Re-Integration -------------------------------------------------

        # Shared UART.
        uart_pads = platform.request("serial")
        uart_sel  = platform.request("user_sw", 0)
        uart_mux_pads = [UARTPads() for _ in range(2)]
        uart_mux      = UARTMultiplexer(uart_mux_pads, uart_pads)
        self.comb += uart_mux.sel.eq(uart_sel)
        self.submodules += uart_mux

        # Shared RAM.
        self.add_ram("shared_ram", 0x0000_0000, 0x1000, contents=[i for i in range(16)])

        # FemtoRV SoC.
        # ------------

        # Generate standalone SoC.
        os.system("litex_soc_gen --cpu-type=femtorv --bus-standard=wishbone --sys-clk-freq=100e6 --name=femtorv_soc --build")

        # Add standalone SoC sources.
        platform.add_source("build/femtorv_soc/gateware/femtorv_soc.v")
        platform.add_source("build/femtorv_soc/gateware/femtorv_soc_rom.init", copy=True)

        # Add CPU sources.
        from litex.soc.cores.cpu.femtorv import FemtoRV
        FemtoRV.add_sources(platform, "standard")

        # Do standalone SoC instance.
        mmap_wb = wishbone.Interface(data_width=32, adr_width=29)
        self.specials += Instance("femtorv_soc",
            # Clk/Rst.
            i_clk     = ClockSignal("sys"),
            i_rst     = ResetSignal("sys"),

            # UART.
            o_uart_tx = uart_mux_pads[0].tx,
            i_uart_rx = uart_mux_pads[0].rx,

            # MMAP.
            o_mmap_m_adr   = mmap_wb.adr[:28], # CHECKME/FIXME: Base address.
            o_mmap_m_dat_w = mmap_wb.dat_w,
            i_mmap_m_dat_r = mmap_wb.dat_r,
            o_mmap_m_sel   = mmap_wb.sel,
            o_mmap_m_cyc   = mmap_wb.cyc,
            o_mmap_m_stb   = mmap_wb.stb,
            i_mmap_m_ack   = mmap_wb.ack,
            o_mmap_m_we    = mmap_wb.we,
            o_mmap_m_cti   = mmap_wb.cti,
            o_mmap_m_bte   = mmap_wb.bte,
            i_mmap_m_err   = mmap_wb.err,
        )
        self.bus.add_master(master=mmap_wb)

        # FireV SoC.
        # ----------

        # Generate standalone SoC.
        os.system("litex_soc_gen --cpu-type=firev --bus-standard=wishbone --sys-clk-freq=100e6 --name=firev_soc --build")

        # Add standalone SoC sources.
        platform.add_source("build/firev_soc/gateware/firev_soc.v")
        platform.add_source("build/firev_soc/gateware/firev_soc_rom.init", copy=True)

        # Add CPU sources.
        from litex.soc.cores.cpu.firev import FireV
        FireV.add_sources(platform, "standard")

        # Do standalone SoC instance.
        mmap_wb = wishbone.Interface(data_width=32, adr_width=29)
        self.specials += Instance("firev_soc",
            # Clk/Rst.
            i_clk     = ClockSignal("sys"),
            i_rst     = ResetSignal("sys"),

            # UART.
            o_uart_tx = uart_mux_pads[1].tx,
            i_uart_rx = uart_mux_pads[1].rx,

            # MMAP.
            o_mmap_m_adr   = mmap_wb.adr[:28], # CHECKME/FIXME: Base address.
            o_mmap_m_dat_w = mmap_wb.dat_w,
            i_mmap_m_dat_r = mmap_wb.dat_r,
            o_mmap_m_sel   = mmap_wb.sel,
            o_mmap_m_cyc   = mmap_wb.cyc,
            o_mmap_m_stb   = mmap_wb.stb,
            i_mmap_m_ack   = mmap_wb.ack,
            o_mmap_m_we    = mmap_wb.we,
            o_mmap_m_cti   = mmap_wb.cti,
            o_mmap_m_bte   = mmap_wb.bte,
            i_mmap_m_err   = mmap_wb.err,
        )
        self.bus.add_master(master=mmap_wb)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LiteX standalone SoC generator on Arty A7")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--toolchain",    default="vivado",    help="FPGA toolchain (vivado, symbiflow or yosys+nextpnr).")
    target_group.add_argument("--build",        action="store_true", help="Build bitstream.")
    target_group.add_argument("--load",         action="store_true", help="Load bitstream.")
    target_group.add_argument("--flash",        action="store_true", help="Flash bitstream.")
    target_group.add_argument("--variant",      default="a7-35",     help="Board variant (a7-35 or a7-100).")
    target_group.add_argument("--sys-clk-freq", default=100e6,       help="System clock frequency.")
    builder_args(parser)
    vivado_build_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(
        variant        = args.variant,
        toolchain      = args.toolchain,
        sys_clk_freq   = int(float(args.sys_clk_freq)),
    )

    builder = Builder(soc, **builder_argdict(args))
    builder_kwargs = vivado_build_argdict(args) if args.toolchain == "vivado" else {}
    builder.build(**builder_kwargs, run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
