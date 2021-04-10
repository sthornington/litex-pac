#!/usr/bin/env python3

#
# Forked from  LiteX-Boards, with additions to support USB debugging.
#
# Copyright (c) 2020 Simon Thornington <simon.thornington@gmail.com>
# Copyright (c) 2018-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2018 David Shah <dave@ds0.me>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse
import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.io import DDROutput

from litex_boards.platforms import ulx3s

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import *
from litex.soc.cores.video import VideoECP5HDMIPHY
from litex.soc.cores.led import LedChaser
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.gpio import GPIOOut

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY
from litescope import LiteScopeAnalyzer

class Matrix8x8(Module, AutoCSR):
    def __init__(self, cd, rst, pads):
        self.pads = pads # for o_matrix_clk/o_matrix_latch/o_matrix_mosi
        self.bus = bus = wishbone.Interface(data_width = 32)
        self.speed = CSRStorage(2) # for i_refresh_speed
        self.specials += Instance("matrix",
                                  i_clk = ClockSignal("sys"), #ClockSignal(cd), # figure out how to get a ClockSignal from the domain itself?
                                  i_reset = rst, #| self.rst
                                  i_i_refresh_speed = self.speed.storage,
                                  o_o_matrix_clk = pads.clk,
                                  o_o_matrix_latch = pads.latch,
                                  o_o_matrix_mosi = pads.mosi,
                                  i_i_wb_cyc = bus.cyc,
                                  i_i_wb_stb = bus.stb,
                                  i_i_wb_we = bus.we,
                                  i_i_wb_addr = bus.adr,
                                  i_i_wb_sel = bus.sel,
                                  i_i_wb_wdata = bus.dat_w,
                                  o_o_wb_ack = bus.ack,
#                                  o_wb_stall = # no stall in regular wishbone?
                                  o_o_wb_rdata = bus.dat_r)

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, with_usb_pll=False, with_video_pll=False, sdram_rate="1:1"):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()
        if sdram_rate == "1:2":
            self.clock_domains.cd_sys2x    = ClockDomain()
            self.clock_domains.cd_sys2x_ps = ClockDomain(reset_less=True)
        else:
            self.clock_domains.cd_sys_ps = ClockDomain(reset_less=True)

        # # #

        # Clk / Rst
        clk25 = platform.request("clk25")
        rst   = platform.request("rst")

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(rst | self.rst)
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.
        else:
           pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=90)

        # USB PLL
        if with_usb_pll:
            self.submodules.usb_pll = usb_pll = ECP5PLL()
            self.comb += usb_pll.reset.eq(rst | self.rst)
            usb_pll.register_clkin(clk25, 25e6)
            self.clock_domains.cd_usb_12 = ClockDomain()
            self.clock_domains.cd_usb_48 = ClockDomain()
            usb_pll.create_clkout(self.cd_usb_12, 12e6, margin=0)
            usb_pll.create_clkout(self.cd_usb_48, 48e6, margin=0)

        # Video PLL
        if with_video_pll:
            self.submodules.video_pll = video_pll = ECP5PLL()
            self.comb += video_pll.reset.eq(rst | self.rst)
            video_pll.register_clkin(clk25, 25e6)
            self.clock_domains.cd_hdmi   = ClockDomain()
            self.clock_domains.cd_hdmi5x = ClockDomain()
            video_pll.create_clkout(self.cd_hdmi,    40e6, margin=0)
            video_pll.create_clkout(self.cd_hdmi5x, 200e6, margin=0)

        # SDRAM clock
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)

        # Prevent ESP32 from resetting FPGA
        self.comb += platform.request("wifi_gpio0").eq(1)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, device="LFE5U-45F", revision="2.0", toolchain="trellis",
                 sys_clk_freq=int(50e6), sdram_module_cls="MT48LC16M16", sdram_rate="1:1",
                 with_video_terminal=False, with_video_framebuffer=False, spiflash=False,
                 usb_debug=False,
                 **kwargs):

        platform = ulx3s.Platform(device=device, revision=revision, toolchain=toolchain)

        if spiflash:
            self.mem_map = {**SoCCore.mem_map, **{"spiflash": 0x80000000}}

        if usb_debug:
            # TODO import this properly somehow?
#            os.system("git clone https://github.com/gregdavill/valentyusb -b hw_cdc_eptri")
            os.system("git clone git@github.com:im-tomu/valentyusb.git")
#            os.system("git clone https://github.com/litex-hub/valentyusb -b hw_cdc_eptri")
            sys.path.append("valentyusb")

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
                         ident          = "LiteX SoC on ULX3S",
                         ident_version  = True,
                         **kwargs)

        # CRG --------------------------------------------------------------------------------------
        with_usb_pll = kwargs.get("uart_name", None) == "usb_acm" or usb_debug
        with_video_pll = with_video_terminal or with_video_framebuffer
        self.submodules.crg = _CRG(platform, sys_clk_freq, with_usb_pll, with_video_pll, sdram_rate=sdram_rate)

        # USB Debug on US2 -------------------------------------------------------------------------
        # This enables the use of wishbone-tool to poke memory via US2 USB interface, and hopefully
        # litescope
        if usb_debug:
            from valentyusb.usbcore.cpu import epfifo, dummyusb
            from valentyusb.usbcore import io as usbio
            usb_pads = self.platform.request("usb")
            usb_iobuf = usbio.IoBuf(usb_pads.d_p, usb_pads.d_n, usb_pads.pullup)
            # self.submodules.usb = epfifo.PerEndpointFifoInterface(usb_iobuf, debug=True)
            # just enumerate and hook up dummy debug wishbone
            # random valenty
#            self.submodules.usb = dummyusb.DummyUsb(usb_iobuf,
#                                                    debug=True)
            # default valenty
            self.submodules.usb = dummyusb.DummyUsb(usb_iobuf,
                                                    debug=True,
                                                    cdc=True,
                                                    relax_timing=True)

            self.add_wb_master(self.usb.debug_bridge.wishbone)
            if not hasattr(self.cpu, 'debug_bus'):
                raise RuntimeError('US2 Debug requires a CPU variant with +debug')

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            sdrphy_cls = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
            self.submodules.sdrphy = sdrphy_cls(platform.request("sdram"))
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = getattr(litedram_modules, sdram_module_cls)(sys_clk_freq, sdram_rate),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )

        # Video ------------------------------------------------------------------------------------
        if with_video_terminal or with_video_framebuffer:
            self.submodules.videophy = VideoECP5HDMIPHY(platform.request("gpdi"), clock_domain="hdmi")
            if with_video_terminal:
                self.add_video_terminal(phy=self.videophy, timings="800x600@60Hz", clock_domain="hdmi")
            if with_video_framebuffer:
                self.add_video_framebuffer(phy=self.videophy, timings="800x600@60Hz", clock_domain="hdmi")

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)


    def add_oled(self):
        pads = self.platform.request("oled_spi")
        pads.miso = Signal()
        self.submodules.oled_spi = SPIMaster(pads, 8, self.sys_clk_freq, 8e6)
        self.oled_spi.add_clk_divider()
        self.submodules.oled_ctl = GPIOOut(self.platform.request("oled_ctl"))

    def add_matrix(self):
        pads = self.platform.request("matrix")
        # https://github.com/sthornington/matrix8x8
        matrix8x8_path = "/home/sthornington/git/matrix8x8/src"
        self.platform.add_verilog_include_path(matrix8x8_path)
        self.platform.add_sources(matrix8x8_path, "matrix.sv", "mod3.sv")
        region_size = 4 * 8 # four bytes per row (8 nibbles), 8 rows
        mem_map = { "matrix": 0xc0000000 }
        self.mem_map.update(mem_map)
        cd = self.crg.cd_sys
        self.add_memory_region("matrix", self.mem_map["matrix"], region_size, type="io")
        matrix = Matrix8x8(cd, self.crg.rst, pads)
        self.submodules.matrix = matrix
        self.add_wb_slave(self.mem_map["matrix"], matrix.bus)

        # LiteScope Analyzer -----------------------------------------------------------------------
        count = Signal(8)
        self.sync += count.eq(count + 1)
        analyzer_signals = [
            matrix.bus,
            count,
        ]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                                                     depth        = 1024,
                                                     clock_domain = "sys", # why can I not use the ClockDomain here?
                                                     csr_csv      = "analyzer.csv")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on ULX3S")
    parser.add_argument("--build",           action="store_true",   help="Build bitstream")
    parser.add_argument("--load",            action="store_true",   help="Load bitstream")
    parser.add_argument("--toolchain",       default="trellis",     help="FPGA toolchain: trellis (default) or diamond")
    parser.add_argument("--device",          default="LFE5U-45F",   help="FPGA device: LFE5U-12F, LFE5U-25F, LFE5U-45F (default)  or LFE5U-85F")
    parser.add_argument("--revision",        default="2.0",         help="Board revision: 2.0 (default) or 1.7")
    parser.add_argument("--sys-clk-freq",    default=50e6,          help="System clock frequency  (default: 50MHz)")
    parser.add_argument("--sdram-module",    default="MT48LC16M16", help="SDRAM module: MT48LC16M16 (default), AS4C32M16 or AS4C16M16")
    parser.add_argument("--with-spiflash",   action="store_true",   help="Make the SPI Flash accessible from the SoC")
    parser.add_argument("--flash-boot-adr",  type=lambda x: int(x,0), default=None, help="Flash boot address")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard", action="store_true",   help="Enable SPI-mode SDCard support")
    sdopts.add_argument("--with-sdcard",     action="store_true",   help="Enable SDCard support")
    parser.add_argument("--with-oled",       action="store_true",   help="Enable SDD1331 OLED support")
    parser.add_argument("--with-matrix",     action="store_true",   help="Enable matrix (shift registered) support")
    parser.add_argument("--sdram-rate",      default="1:1",         help="SDRAM Rate: 1:1 Full Rate (default), 1:2 Half Rate")
    viopts = parser.add_mutually_exclusive_group()
    viopts.add_argument("--with-video-terminal",    action="store_true", help="Enable Video Terminal (HDMI)")
    viopts.add_argument("--with-video-framebuffer", action="store_true", help="Enable Video Framebuffer (HDMI)")
    parser.add_argument("--with-us2-debug",  action="store_true",   help="Enable Wishbone debug bridge on US2")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(
        device           = args.device,
        revision         = args.revision,
        toolchain        = args.toolchain,
        sys_clk_freq     = int(float(args.sys_clk_freq)),
        sdram_module_cls = args.sdram_module,
        sdram_rate       = args.sdram_rate,
        with_video_terminal    = args.with_video_terminal,
        with_video_framebuffer = args.with_video_framebuffer,
        spiflash               = args.with_spiflash,
        usb_debug        = args.with_us2_debug,
        **soc_core_argdict(args))
    assert not (args.with_spi_sdcard and args.with_sdcard)
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()
    if args.with_oled:
        soc.add_oled()
    if args.with_matrix:
        soc.add_matrix()
    if args.with_spiflash:
        soc.add_spi_flash(mode="1x", dummy_cycles=8)
        if args.flash_boot_adr:
            soc.add_constant("FLASH_BOOT_ADDRESS", args.flash_boot_adr)

    builder = Builder(soc, **builder_argdict(args))
    builder_kargs = trellis_argdict(args) if args.toolchain == "trellis" else {}
    builder.build(**builder_kargs, run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".svf"))

if __name__ == "__main__":
    main()
