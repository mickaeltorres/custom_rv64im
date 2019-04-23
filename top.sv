/*
*
* Copyright (c) 2019, Mickael Torres
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

`timescale 1ns / 1ps

module TOP(
    input               SYS_RST,
    input               SYS_CLK_P,
    input               SYS_CLK_N,
    input               SER_RX,
    output              SER_TX,
    output [7:0]        LEDS,
    output              SD_CS,
    output              SD_MOSI,
    output              SD_SCK,
    input               SD_MISO,
    input               RND_IN,
    output              RTC_SCL,
    inout               RTC_SDA,
    inout               ETH1_MDIO,
    output              ETH1_MDC,
    output              ETH1_RST_N,
    input [3:0]         ETH1_RX,
    input               ETH1_RX_CTL,
    input               ETH1_RX_CLK,
    output [3:0]        ETH1_TX,
    output              ETH1_TX_CTL,
    output              ETH1_TX_CLK,
    output              c0_ddr4_act_n,
    output [16:0]       c0_ddr4_adr,
    output [1:0]        c0_ddr4_ba,
    output [0:0]        c0_ddr4_bg,
    output [0:0]        c0_ddr4_cke,
    output [0:0]        c0_ddr4_odt,
    output [0:0]        c0_ddr4_cs_n,
    output [0:0]        c0_ddr4_ck_t,
    output [0:0]        c0_ddr4_ck_c,
    output              c0_ddr4_reset_n,
    inout  [3:0]        c0_ddr4_dm_dbi_n,
    inout  [31:0]       c0_ddr4_dq,
    inout  [3:0]        c0_ddr4_dqs_c,
    inout  [3:0]        c0_ddr4_dqs_t,
    output              c0_ddr4_calib_complete,
    output              HDMI_CLK,
    output              HDMI_SPDIF_OUT,
    output              HDMI_VSYNC,
    output              HDMI_HSYNC,
    output              HDMI_DE,
    output [15:0]       HDMI_DATA,
    inout               HDMI_SCL,
    inout               HDMI_SDA,
    input               HDMI_INT,
    input               HDMI_SPDIF_IN,
    output              USB_Z,
    output              USB_DP_OUT,
    output              USB_DN_OUT,
    input               USB_DP_IN,
    input               USB_DN_IN
);

// ETH1 MDI
wire                    ETH1_MDIO_IN;
wire                    ETH1_MDIO_OUT;
wire                    ETH1_MDIO_STATE;
assign ETH1_MDIO = ETH1_MDIO_STATE ? 1'bZ : ETH1_MDIO_OUT;
assign ETH1_MDIO_IN = ETH1_MDIO;

// RTC
wire                    RTC_SCL_OUT;
wire                    RTC_SDA_IN;
wire                    RTC_SDA_OUT;
assign RTC_SDA = RTC_SDA_OUT ? 1'bZ : 0;
assign RTC_SDA_IN = RTC_SDA;
assign RTC_SCL = RTC_SCL_OUT ? 1'bZ : 0;

// HDMI
wire                    HDMI_SCL_OUT;
wire                    HDMI_SDA_IN;
wire                    HDMI_SDA_OUT;
assign HDMI_SDA = HDMI_SDA_OUT ? 1'bZ : 0;
assign HDMI_SDA_IN = HDMI_SDA;
assign HDMI_SCL = HDMI_SCL_OUT ? 1'bZ : 0;

// DDR4
wire                    DDR4_CALIB_DONE;
wire                    CLK200;
wire                    CLK_RST_SYNC;
wire                    CLK125;
wire                    CLK111;
wire                    CLK50;
wire                    CLK25;
wire                    DDR4_DBG_CLK;
wire [27:0]             DDR4_ADDR;
wire [2:0]              DDR4_CMD;
wire                    DDR4_EN;
wire                    DDR4_HIPRIO;
wire [255:0]            DDR4_WDF_DATA;
wire                    DDR4_WDF_END;
wire [31:0]             DDR4_WDF_MASK;
wire                    DDR4_WDF_EN;
wire [255:0]            DDR4_DATA;
wire                    DDR4_DATA_END;
wire                    DDR4_DATA_VALID;
wire                    DDR4_RDY;
wire                    DDR4_WDF_RDY;
wire [511:0]            DDR4_DBG_BUS;
DDRC ddr4(  SYS_RST, SYS_CLK_P, SYS_CLK_N, c0_ddr4_act_n, c0_ddr4_adr, c0_ddr4_ba, c0_ddr4_bg, c0_ddr4_cke, c0_ddr4_odt, c0_ddr4_cs_n, c0_ddr4_ck_t,
            c0_ddr4_ck_c, c0_ddr4_reset_n, c0_ddr4_dm_dbi_n, c0_ddr4_dq, c0_ddr4_dqs_c, c0_ddr4_dqs_t,
            DDR4_CALIB_DONE, CLK200, CLK_RST_SYNC, CLK125, CLK111, CLK50, CLK25, DDR4_DBG_CLK, DDR4_ADDR, DDR4_CMD, DDR4_EN, DDR4_HIPRIO, DDR4_WDF_DATA,
            DDR4_WDF_END, DDR4_WDF_MASK, DDR4_WDF_EN, DDR4_DATA, DDR4_DATA_END, DDR4_DATA_VALID, DDR4_RDY, DDR4_WDF_RDY, DDR4_DBG_BUS);

assign c0_ddr4_calib_complete = DDR4_CALIB_DONE;

// ROM for IPL
wire [8:0]              ROM_ADDR1;
wire [255:0]            ROM_DATA1;
IPLROM iplrom(CLK200, ROM_ADDR1, ROM_DATA1);

// UART
wire                    UART_RST;
wire [7:0]              UARTRDIN;
wire                    UARTRDWE;
wire                    UARTRDRE;
wire [7:0]              UARTRDOUT;
wire                    UARTRDFULL;
wire                    UARTRDEMPTY;
wire                    UARTRDWRBSY;
wire                    UARTRDRDBSY;
UARTFIFO uartrdfifo(CLK200, !UART_RST, UARTRDIN, UARTRDWE, UARTRDRE, UARTRDOUT, UARTRDFULL, UARTRDEMPTY, UARTRDWRBSY, UARTRDRDBSY);
wire [7:0]              UARTWRIN;
wire                    UARTWRWE;
wire                    UARTWRRE;
wire [7:0]              UARTWROUT;
wire                    UARTWRFULL;
wire                    UARTWREMPTY;
wire                    UARTWRWRBSY;
wire                    UARTWRRDBSY;
UARTFIFO uartwrfifo(CLK200, !UART_RST, UARTWRIN, UARTWRWE, UARTWRRE, UARTWROUT, UARTWRFULL, UARTWREMPTY, UARTWRWRBSY, UARTWRRDBSY);
UART uart(CLK200, UART_RST, SER_RX, SER_TX, UARTWRRE, UARTWREMPTY, UARTWROUT, UARTRDIN, UARTRDWE);

// RV64I CORE
wire                    C0_RST;
wire [1:0]              C0_STATE;
wire [26:0]             C0_ADDR;
wire [255:0]            C0_DOUT;
wire [31:0]             C0_DMASK;
wire                    C0_BUS;
wire [255:0]            C0_DIN;
wire                    C0_INTR;
wire [27:0]             C0_CC_ADDR;
R64 #(.COREn(0)) core0(CLK200, C0_RST, C0_STATE, C0_ADDR, C0_DOUT, C0_DMASK, C0_BUS, C0_DIN, C0_INTR, C0_CC_ADDR);

// RV64I CORE
wire                    C1_RST;
wire [1:0]              C1_STATE;
wire [26:0]             C1_ADDR;
wire [255:0]            C1_DOUT;
wire [31:0]             C1_DMASK;
wire                    C1_BUS;
wire [255:0]            C1_DIN;
wire                    C1_INTR;
wire [27:0]             C1_CC_ADDR;
R64 #(.COREn(1)) core1(CLK200, C1_RST, C1_STATE, C1_ADDR, C1_DOUT, C1_DMASK, C1_BUS, C1_DIN, C1_INTR, C1_CC_ADDR);

// SD CONTROLER
wire                    SD_RST;
wire [39:0]             SDCMDIN;
wire                    SDCMDWE;
wire                    SDCMDRE;
wire [39:0]             SDCMDOUT;
wire                    SDCMDFULL;
wire                    SDCMDEMPTY;
wire                    SDCMDWBSY;
wire                    SDCMDRBSY;
SDCMDFIFO sdcmdfifo(!SD_RST, CLK200, CLK25, SDCMDIN, SDCMDWE, SDCMDRE, SDCMDOUT, SDCMDFULL, SDCMDEMPTY, SDCMDWBSY, SDCMDRBSY);
wire [7:0]              SDDATIN;
wire                    SDDATWE;
wire                    SDDATRE;
wire [7:0]              SDDATOUT;
wire                    SDDATFULL;
wire                    SDDATEMPTY;
wire                    SDDATWBSY;
wire                    SDDATRBSY;
SDDATFIFO sddatfifo(!SD_RST, CLK25, CLK200, SDDATIN, SDDATWE, SDDATRE, SDDATOUT, SDDATFULL, SDDATEMPTY, SDDATWBSY, SDDATRBSY);
SDCTRL sd(CLK25, SD_RST, SD_CS, SD_MOSI, SD_SCK, SD_MISO, SDCMDRE, SDCMDOUT, SDCMDEMPTY, SDDATIN, SDDATWE, SDDATFULL);

// RNG
wire                    RNG_RST;
wire [7:0]              RNGF_IN;
wire                    RNGF_WE;
wire                    RNGF_RE;
wire [7:0]              RNGF_OUT;
wire                    RNGF_FULL;
wire                    RNGF_EMPTY;
wire                    RNGF_WBSY;
wire                    RNGF_RBSY;
RNGFIFO rngfifo(CLK200, !RNG_RST, RNGF_IN, RNGF_WE, RNGF_RE, RNGF_OUT, RNGF_FULL, RNGF_EMPTY, RNGF_WBSY, RNGF_RBSY);
RNG rng(CLK200, RNG_RST, RND_IN, RNGF_IN, RNGF_WE, RNGF_FULL);

// ETHERNET
wire                    ETH_RST;
wire                    ETH_RX_RST;
wire [15:0]             ETH_STATUS;
wire [63:0]             ETHCMDF_IN;
wire                    ETHCMDF_WE;
wire                    ETHCMDF_RE;
wire [63:0]             ETHCMDF_OUT;
wire                    ETHCMDF_FULL;
wire                    ETHCMDF_EMPTY;
wire                    ETHCMDF_WBSY;
wire                    ETHCMDF_RBSY;
ETH_CMD_FIFO ethcmdf(!ETH_RST, CLK200, CLK125, ETHCMDF_IN, ETHCMDF_WE, ETHCMDF_RE, ETHCMDF_OUT, ETHCMDF_FULL, ETHCMDF_EMPTY, ETHCMDF_WBSY, ETHCMDF_RBSY);
wire [31:0]             ETHACKF_IN;
wire                    ETHACKF_WE;
wire                    ETHACKF_RE;
wire [31:0]             ETHACKF_OUT;
wire                    ETHACKF_FULL;
wire                    ETHACKF_EMPTY;
wire                    ETHACKF_WBSY;
wire                    ETHACKF_RBSY;
ETH_ACK_FIFO ethackf(!ETH_RX_RST, ETH1_RX_CLK, CLK200, ETHACKF_IN, ETHACKF_WE, ETHACKF_RE, ETHACKF_OUT, ETHACKF_FULL, ETHACKF_EMPTY, ETHACKF_WBSY, ETHACKF_RBSY);
wire [7:0]              ETHRCVF_IN;
wire                    ETHRCVF_WE;
wire                    ETHRCVF_RE;
wire [7:0]              ETHRCVF_OUT;
wire                    ETHRCVF_FULL;
wire                    ETHRCVF_EMPTY;
wire                    ETHRCVF_WBSY;
wire                    ETHRCVF_RBSY;
ETH_DAT_RCV_FIFO ethrcvf(!ETH_RX_RST, ETH1_RX_CLK, CLK200, ETHRCVF_IN, ETHRCVF_WE, ETHRCVF_RE, ETHRCVF_OUT, ETHRCVF_FULL, ETHRCVF_EMPTY, ETHRCVF_WBSY, ETHRCVF_RBSY);
wire [7:0]              ETHSNDF_IN;
wire                    ETHSNDF_WE;
wire                    ETHSNDF_RE;
wire [7:0]              ETHSNDF_OUT;
wire                    ETHSNDF_FULL;
wire                    ETHSNDF_EMPTY;
wire                    ETHSNDF_WBSY;
wire                    ETHSNDF_RBSY;
ETH_DAT_SND_FIFO ethsndf(!ETH_RST, CLK200, CLK125, ETHSNDF_IN, ETHSNDF_WE, ETHSNDF_RE, ETHSNDF_OUT, ETHSNDF_FULL, ETHSNDF_EMPTY, ETHSNDF_WBSY, ETHSNDF_RBSY);
ETHDRV ethdrv0(CLK200, CLK125, ETH_RST, ETH_RX_RST, ETH1_MDIO_IN, ETH1_MDIO_OUT, ETH1_MDIO_STATE, ETH1_MDC, ETH1_RST_N, ETH1_RX, ETH1_RX_CTL, ETH1_RX_CLK, ETH1_TX, ETH1_TX_CTL, ETH1_TX_CLK,
               ETH_STATUS, ETHCMDF_OUT, ETHCMDF_RE, ETHCMDF_EMPTY, ETHACKF_IN, ETHACKF_WE, ETHACKF_FULL,
               ETHRCVF_IN, ETHRCVF_WE, ETHRCVF_FULL, ETHSNDF_OUT, ETHSNDF_RE, ETHSNDF_EMPTY); 

// RTC
wire [63:0]             RTC_OUT;
wire [63:0]             RTC_CTL;
RTC rtc(CLK200, RTC_SCL_OUT, RTC_SDA_IN, RTC_SDA_OUT, RTC_OUT, RTC_CTL);

// HDMI
wire                    CLK108;
CLK hdmiclk(CLK108, CLK200);
wire                    HDMITFB_WE;
wire [13:0]             HDMITFB_WADDR;
wire [15:0]             HDMITFB_WDAT;
wire [13:0]             HDMITFB_RADDR;
wire [15:0]             HDMITFB_RDAT;
HDMITFB hdmitfb(CLK200, HDMITFB_WE, HDMITFB_WADDR, HDMITFB_WDAT, CLK108, HDMITFB_RADDR, HDMITFB_RDAT);
wire [10:0]             HDMIFONT_ADDR;
wire [7:0]              HDMIFONT_DATA;
HDMIFONT hdmifont(CLK108, HDMIFONT_ADDR, HDMIFONT_DATA);
wire                    HDMI_RST;
wire [7:0]              HDMIWRIN;
wire                    HDMIWRWE;
wire                    HDMIWRRE;
wire [7:0]              HDMIWROUT;
wire                    HDMIWRFULL;
wire                    HDMIWREMPTY;
wire                    HDMIWRWRBSY;
wire                    HDMIWRRDBSY;
UARTFIFO hdmiwrfifo(CLK200, !HDMI_RST, HDMIWRIN, HDMIWRWE, HDMIWRRE, HDMIWROUT, HDMIWRFULL, HDMIWREMPTY, HDMIWRWRBSY, HDMIWRRDBSY);
HDMI hdmi(CLK200, CLK108, HDMI_RST, HDMI_CLK, HDMI_SPDIF_OUT, HDMI_VSYNC, HDMI_HSYNC, HDMI_DE, HDMI_DATA, HDMI_INT, HDMI_SPDIF_IN, HDMI_SCL_OUT, HDMI_SDA_OUT, HDMI_SDA_IN,
          HDMITFB_WE, HDMITFB_WADDR, HDMITFB_WDAT, HDMITFB_RADDR, HDMITFB_RDAT, HDMIFONT_ADDR, HDMIFONT_DATA, HDMIWRRE, HDMIWREMPTY, HDMIWROUT);

// USB
wire                    USB_RST;
wire                    USB_FIFO_RST;
wire                    USB_SND_RST;
wire [15:0]             USB_RCVF_IN;
wire                    USB_RCVF_WE;
wire                    USB_RCVF_RE;
wire [15:0]             USB_RCVF_OUT;
wire                    USB_RCVF_FULL;
wire                    USB_RCVF_EMPTY;
wire                    USB_RCVF_WBSY;
wire                    USB_RCVF_RBSY;
USB_FIFO_RCV usb_rcv(!USB_FIFO_RST, CLK111, CLK200, USB_RCVF_IN, USB_RCVF_WE, USB_RCVF_RE, USB_RCVF_OUT, USB_RCVF_FULL, USB_RCVF_EMPTY, USB_RCVF_WBSY, USB_RCVF_RBSY);
wire [15:0]             USB_SNDF_IN;
wire                    USB_SNDF_WE;
wire                    USB_SNDF_RE;
wire [15:0]             USB_SNDF_OUT;
wire                    USB_SNDF_FULL;
wire                    USB_SNDF_EMPTY;
wire                    USB_SNDF_WBSY;
wire                    USB_SNDF_RBSY;
USB_FIFO usb_snd(!USB_SND_RST, CLK200, CLK111, USB_SNDF_IN, USB_SNDF_WE, USB_SNDF_RE, USB_SNDF_OUT, USB_SNDF_FULL, USB_SNDF_EMPTY, USB_SNDF_WBSY, USB_SNDF_RBSY);
wire                    UFI0_RESET;
wire [8:0]              UFI0_DIN;
wire [8:0]              UFI0_DOUT;
wire                    UFI0_WE;
wire                    UFI0_RE;
wire                    UFI0_FULL;
wire                    UFI0_EMPTY;
wire                    UFI0_RBUSY;
wire                    UFI0_WBUSY;
USBFIFOINT usbfifoint0(CLK111, UFI0_RESET, UFI0_DIN, UFI0_WE, UFI0_RE, UFI0_DOUT, UFI0_FULL, UFI0_EMPTY, UFI0_RBUSY, UFI0_WBUSY);
wire                    UFI1_RESET;
wire [8:0]              UFI1_DIN;
wire [8:0]              UFI1_DOUT;
wire                    UFI1_WE;
wire                    UFI1_RE;
wire                    UFI1_FULL;
wire                    UFI1_EMPTY;
wire                    UFI1_RBUSY;
wire                    UFI1_WBUSY;
USBFIFOINT usbfifoint1(CLK111, UFI1_RESET, UFI1_DIN, UFI1_WE, UFI1_RE, UFI1_DOUT, UFI1_FULL, UFI1_EMPTY, UFI1_RBUSY, UFI1_WBUSY);
wire                    USB_DRV;
assign USB_Z = USB_DRV;
wire [3:0]              USBRAM_WE;
wire [10:0]             USBRAM_ADDR;
wire [31:0]             USBRAM_IN;
wire [31:0]             USBRAM_OUT;
USBRAM usbram(CLK111, USBRAM_WE, USBRAM_ADDR, USBRAM_IN, USBRAM_OUT);
wire                    USBPROM_WE;
wire [11:0]             USBPROM_ADDR_A;
wire [31:0]             USBPROM_DIN_A;
wire [11:0]             USBPROM_ADDR_B;
wire [31:0]             USBPROM_DOUT_B;
USBPROM usbprom(CLK200, USBPROM_WE, USBPROM_ADDR_A, USBPROM_DIN_A, CLK111, USBPROM_ADDR_B, USBPROM_DOUT_B);
USBC usbc(CLK200, CLK111, USB_RST, USB_FIFO_RST, USB_RCVF_IN, USB_RCVF_WE, USB_RCVF_FULL, USB_SNDF_OUT, USB_SNDF_RE, USB_SNDF_EMPTY,
          USB_DP_IN, USB_DN_IN, USB_DRV, USB_DP_OUT, USB_DN_OUT,
          UFI0_RESET, UFI0_DIN, UFI0_DOUT, UFI0_WE, UFI0_RE, UFI0_FULL, UFI0_EMPTY,
          UFI1_RESET, UFI1_DIN, UFI1_DOUT, UFI1_WE, UFI1_RE, UFI1_FULL, UFI1_EMPTY,
          USBRAM_WE, USBRAM_ADDR, USBRAM_IN, USBRAM_OUT, USBPROM_ADDR_B, USBPROM_DOUT_B);

// BUS ARBITER / INTERRUPT CONTROLLER
BUS bus(CLK200, CLK_RST_SYNC, UART_RST, SD_RST, RNG_RST, ETH_RST, USB_RST, USB_SND_RST, HDMI_RST,
        C0_RST, C0_STATE, C0_ADDR, C0_DOUT, C0_DMASK, C0_BUS, C0_DIN, C0_INTR, C0_CC_ADDR,
        C1_RST, C1_STATE, C1_ADDR, C1_DOUT, C1_DMASK, C1_BUS, C1_DIN, C1_INTR, C1_CC_ADDR,
        DDR4_ADDR, DDR4_CMD, DDR4_EN, DDR4_HIPRIO, DDR4_WDF_DATA, DDR4_WDF_END, DDR4_WDF_MASK,
        DDR4_WDF_EN, DDR4_DATA, DDR4_DATA_END, DDR4_DATA_VALID, DDR4_RDY, DDR4_WDF_RDY,
        ROM_ADDR1, ROM_DATA1, UARTRDRE, UARTRDEMPTY, UARTRDOUT, UARTWRIN, UARTWRWE, UARTWRFULL,
        SDCMDIN, SDCMDWE, SDCMDFULL, SDDATRE, SDDATOUT, SDDATEMPTY,
        RNGF_OUT, RNGF_RE, RNGF_EMPTY, LEDS,
        ETH_STATUS, ETHCMDF_IN, ETHCMDF_WE, ETHCMDF_FULL, ETHACKF_OUT, ETHACKF_RE, ETHACKF_EMPTY,
        ETHRCVF_OUT, ETHRCVF_RE, ETHRCVF_EMPTY, ETHSNDF_IN, ETHSNDF_WE, ETHSNDF_FULL,
        USB_RCVF_OUT, USB_RCVF_RE, USB_RCVF_EMPTY, USB_SNDF_IN, USB_SNDF_WE, USB_SNDF_FULL,
        RTC_OUT, RTC_CTL, HDMIWRIN, HDMIWRWE, HDMIWRFULL,
        USBPROM_WE, USBPROM_ADDR_A, USBPROM_DIN_A);

endmodule
