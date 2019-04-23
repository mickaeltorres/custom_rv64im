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

module BUS(
    input              CLK,
	input              RST,
	output reg         UART_RST,
	output reg         SD_RST,
	output reg         RNG_RST,
	output reg         ETH_RST,
	output reg         USB_RST,
	output reg         USB_SND_RST,
	output reg         HDMI_RST,
    (* KEEP *) output reg         xCORE0_RST,
    input [1:0]        xCORE0_STATE,
    input [26:0]       xCORE0_ADDR,
    input [255:0]      xCORE0_DOUT,
	input [31:0]       xCORE0_DMASK,
    (* KEEP *) output reg         xCORE0_BUS,
    (* KEEP *) output reg [255:0] xCORE0_DIN,
    (* KEEP *) output reg         xCORE0_INTR,
    (* KEEP *) output reg [27:0]  xCORE0_CC_ADDR,
	(* KEEP *) output reg         xCORE1_RST,
    input [1:0]        xCORE1_STATE,
    input [26:0]       xCORE1_ADDR,
    input [255:0]      xCORE1_DOUT,
	input [31:0]       xCORE1_DMASK,
    (* KEEP *) output reg         xCORE1_BUS,
    (* KEEP *) output reg [255:0] xCORE1_DIN,
    (* KEEP *) output reg         xCORE1_INTR,
    (* KEEP *) output reg [27:0]  xCORE1_CC_ADDR,
	output reg [27:0]  DDR4_ADDR,
	output reg [2:0]   DDR4_CMD,
	output reg         DDR4_EN,
	output reg         DDR4_HIPRIO,
	output [255:0]     DDR4_WDF_DATA_C,
	output reg 	       DDR4_WDF_END,
	output reg [31:0]  DDR4_WDF_MASK,
	output reg 	       DDR4_WDF_EN,
	input [255:0]      DDR4_DATA_C,
	input              DDR4_DATA_END,
	input              DDR4_DATA_VALID,
	input			   DDR4_RDY,
	input              DDR4_WDF_RDY,
	output reg [8:0]   ROM_ADDR1,
	input [255:0]      ROM_DATA1,
    output reg         UART_RD_RE,
    input              UART_RD_EMPTY,
    input [7:0]        UART_RD_DATA,
    output reg [7:0]   UART_WR_DATA,
    output reg         UART_WR_EN,
    input              UART_WR_FULL,
    output reg [39:0]  SDCMDIN,
    output reg         SDCMDWE,
    input              SDCMDFULL,
    output reg         SDDATRE,
    input [7:0]        SDDATOUT,
    input              SDDATEMPTY,
    input [7:0]        RNGF_IN,
    output reg         RNGF_RE,
    input              RNGF_EMPTY,
    output reg [7:0]   LEDS,
    input [15:0]       ETH_STATUS,
    output reg [63:0]  ETHCMDF_IN,
    output reg         ETHCMDF_WE,
    input              ETHCMDF_FULL,
    input [31:0]       ETHACKF_OUT,
    output reg         ETHACKF_RE,
    input              ETHACKF_EMPTY,
    input [7:0]        ETHRCVF_OUT,
    output reg         ETHRCVF_RE,
    input              ETHRCVF_EMPTY,
    output reg [7:0]   ETHSNDF_IN,
    output reg         ETHSNDF_WE,
    input              ETHSNDF_FULL,
    input [15:0]       USB_RCVF_OUT,
    output reg         USB_RCVF_RE,
    input              USB_RCVF_EMPTY,
    output reg [15:0]  USB_SNDF_IN,
    output reg         USB_SNDF_WE,
    input              USB_SNDF_FULL,
    input [63:0]       RTC_IN,
    output reg [63:0]  RTC_CTL,
(* KEEP="TRUE" *) output reg [7:0]   HDMI_WR_DATA,
(* KEEP="TRUE" *) output reg         HDMI_WR_EN,
    input              HDMI_WR_FULL,
    output reg         USBPROM_WE,
    output reg [11:0]  USBPROM_ADDR,
    output reg [31:0]  USBPROM_DIN
);

reg                     CORE0_BUSY;
reg                     CORE0_CLR;
reg                     CORE1_BUSY;
reg                     CORE1_CLR;
(* KEEP *)reg         CORE0_RST;
(* KEEP *)reg [1:0]        CORE0_STATE;
(* KEEP *)reg [26:0]       CORE0_ADDR;
(* KEEP *)reg [255:0]      CORE0_DOUT;
(* KEEP *)reg [31:0]       CORE0_DMASK;
(* KEEP *)reg         CORE0_BUS;
(* KEEP *)reg [255:0] CORE0_DIN;
(* KEEP *)reg         CORE0_INTR;
(* KEEP *)reg [27:0]  CORE0_CC_ADDR;
(* KEEP *)reg         CORE1_RST;
(* KEEP *)reg [1:0]        CORE1_STATE;
(* KEEP *)reg [26:0]       CORE1_ADDR;
(* KEEP *)reg [255:0]      CORE1_DOUT;
(* KEEP *)reg [31:0]       CORE1_DMASK;
(* KEEP *)reg         CORE1_BUS;
(* KEEP *)reg [255:0] CORE1_DIN;
(* KEEP *)reg         CORE1_INTR;
(* KEEP *)reg [27:0]  CORE1_CC_ADDR;

// attempt to pass timing requirements
always @(posedge CLK)
begin
    if (RST == 1) begin
        CORE0_BUSY <= 0;
        CORE1_BUSY <= 0;
        CORE0_STATE <= 0;
        CORE1_STATE <= 0;
        xCORE0_RST <= 0;
        xCORE1_RST <= 0;
    end else begin
        xCORE0_RST <= CORE0_RST;
        xCORE1_RST <= CORE1_RST;
        if ((CORE0_BUSY == 0) && (xCORE0_STATE != 0)) begin
            CORE0_STATE <= xCORE0_STATE;
            CORE0_ADDR <= xCORE0_ADDR;
            CORE0_DOUT <= xCORE0_DOUT;
            CORE0_DMASK <= xCORE0_DMASK;
            CORE0_BUSY <= 1;
        end
        xCORE0_BUS <= CORE0_BUS;
        xCORE0_DIN <= CORE0_DIN;
        xCORE0_INTR <= CORE0_INTR;
        xCORE0_CC_ADDR <= CORE0_CC_ADDR;
        if ((CORE1_BUSY == 0) && (xCORE1_STATE != 0)) begin
            CORE1_STATE <= xCORE1_STATE;
            CORE1_ADDR <= xCORE1_ADDR;
            CORE1_DOUT <= xCORE1_DOUT;
            CORE1_DMASK <= xCORE1_DMASK;
            CORE1_BUSY <= 1;
        end
        xCORE1_BUS <= CORE1_BUS;
        xCORE1_DIN <= CORE1_DIN;
        xCORE1_INTR <= CORE1_INTR;
        xCORE1_CC_ADDR <= CORE1_CC_ADDR;
        if (CORE0_CLR == 1) begin
            CORE0_BUSY <= 0;
            CORE0_STATE <= 0;
        end
        if (CORE1_CLR == 1) begin
            CORE1_BUSY <= 0;
            CORE1_STATE <= 0;
        end
    end
end

// L2 CACHE
reg [36:0]              L2_0_WE;
reg [10:0]              L2_0_ADDR;
reg [295:0]             L2_0_DIN;
reg [295:0]             L2_0_DOUT;
L2CACHE l2_0(CLK, L2_0_WE, L2_0_ADDR, L2_0_DIN, L2_0_DOUT);
reg [36:0]              L2_1_WE;
reg [10:0]              L2_1_ADDR;
reg [295:0]             L2_1_DIN;
reg [295:0]             L2_1_DOUT;
L2CACHE l2_1(CLK, L2_1_WE, L2_1_ADDR, L2_1_DIN, L2_1_DOUT);
reg [36:0]              L2_2_WE;
reg [10:0]              L2_2_ADDR;
reg [295:0]             L2_2_DIN;
reg [295:0]             L2_2_DOUT;
L2CACHE l2_2(CLK, L2_2_WE, L2_2_ADDR, L2_2_DIN, L2_2_DOUT);
reg [36:0]              L2_3_WE;
reg [10:0]              L2_3_ADDR;
reg [295:0]             L2_3_DIN;
reg [295:0]             L2_3_DOUT;
L2CACHE l2_3(CLK, L2_3_WE, L2_3_ADDR, L2_3_DIN, L2_3_DOUT);
reg [2:0]               L2_CNT;

reg [7:0]               IPI;

reg [7:0]               CORE0_IRQ;
(* KEEP *) reg          CORE0_ACK;
reg [7:0]               CORE1_IRQ;
(* KEEP *) reg          CORE1_ACK;
wire [7:0]              CORE_IRQ;

reg [5:0]               IRQ0_IN;
reg [5:0]               IRQ0_EN;
reg [5:0]               IRQ0_TYPE;
reg [5:0]               IRQ1_IN;
reg [5:0]               IRQ1_EN;
reg [5:0]               IRQ1_TYPE;
wire [5:0]              IRQ_EN;
wire [5:0]              IRQ_TYPE;

reg [31:0]              TMR0_COMP;
reg                     TMR0_CLEAR;
reg [31:0]              TMR1_COMP;
reg                     TMR1_CLEAR;
wire [31:0]             TMR_COMP;


reg [63:0]              rtc_ctl;
reg [63:0]              rtc_ctl2;
reg [63:0]              rtc_in;
reg [63:0]              rtc_in2;

reg [31:0]              addr;
reg [255:0]             dout;
reg [31:0]              dmask;
reg [15:0]              eth_status;
reg [15:0]              eth_status2;

wire [255:0]            DDR4_DATA;
reg [255:0]             DDR4_WDF_DATA;

assign DDR4_DATA = DDR4_DATA_C;
assign DDR4_WDF_DATA_C = DDR4_WDF_DATA;

typedef enum
{
    ST_INIT, ST_IDLE, ST_WAIT,
    ST_RD, ST_WR, ST_INT,
    ST_ASID_FLUSH, ST_ADDR_FLUSH,
    ST_RDSRAM1, ST_RDSRAM2, ST_RDSRAM3,
    ST_RDROM1, ST_RDROM2, ST_RDROM3,
    ST_RDCACHE0, ST_RDCACHE1, ST_RDRAM1, ST_RDRAM2, ST_WRRAM1, 
    ST_RDSD1, ST_RDSD2, ST_RDSD3,
    ST_UARTRD1, ST_UARTRD2, ST_UARTRD3,
    ST_RDRNG1, ST_RDRNG2, ST_RDRNG3,
    ST_RDETHACK1, ST_RDETHACK2, ST_RDETHACK3,
    ST_RDETHRCV1, ST_RDETHRCV2, ST_RDETHRCV3,
    ST_RDUSB1, ST_RDUSB2, ST_RDUSB3
} state_t;
state_t                 state;

reg [11:0]              init_cnt;

reg                     core_n;

assign IRQ_EN = core_n == 0 ? IRQ0_EN : IRQ1_EN;
assign IRQ_TYPE = core_n == 0 ? IRQ0_TYPE : IRQ1_TYPE;
assign TMR_COMP = core_n == 0 ? TMR0_COMP : TMR1_COMP;
assign CORE_IRQ = core_n == 0 ? CORE0_IRQ : CORE1_IRQ;

`define CC_BUS(DATA) if (core_n == 0) begin CORE0_BUS <= DATA; CORE0_CLR <= 1; end else begin CORE1_BUS <= DATA; CORE1_CLR <= 1; end
`define CC_DIN(DATA) if (core_n == 0) CORE0_DIN <= DATA; else CORE1_DIN <= DATA
`define CC_TMR_CLR(DATA) if (core_n == 0) TMR0_CLEAR <= DATA; TMR1_CLEAR <= DATA
`define CC_IRQ_EN(DATA) if (core_n == 0) IRQ0_EN <= DATA; else IRQ1_EN <= DATA
`define CC_IRQ_TYPE(DATA) if (core_n == 0) IRQ0_TYPE <= DATA; else IRQ1_TYPE <= DATA
`define CC_TMR_COMP(DATA) if (core_n == 0) TMR0_COMP <= DATA; else TMR1_COMP <= DATA
`define CC_IRQ_EN_BIT(IRQ,DATA) if (core_n == 0) IRQ0_EN[IRQ] <= DATA; else IRQ1_EN[IRQ] <= DATA
`define CC_CORE_ACK(DATA) if (core_n == 0) CORE0_ACK <= DATA; else CORE1_ACK <= DATA
`define CC_CORE_CC_ADDR(DATA) if (core_n == 0) CORE1_CC_ADDR <= DATA; else CORE0_CC_ADDR <= DATA

always @(posedge CLK)
begin
    if (RST == 1)
    begin
        DDR4_ADDR <= 0;
        DDR4_CMD <= 0;
        DDR4_EN <= 0;
        DDR4_HIPRIO <= 0;
        DDR4_WDF_DATA <= 0;
        DDR4_WDF_END <= 0;
        DDR4_WDF_MASK <= 0;
        DDR4_WDF_EN <= 0;
        UART_WR_EN <= 0;
        UART_RD_RE <= 0;
        SDCMDWE <= 0;
        UART_RST <= 0;
        SD_RST <= 0;
        RNG_RST <= 0;
        ETH_RST <= 0;
        USB_RST <= 0;
        USB_SND_RST <= 0;
        HDMI_RST <= 0;
        ETHSNDF_WE <= 0;
        ETHCMDF_WE <= 0;
        ETHACKF_RE <= 0;
        ETHRCVF_RE <= 0;
        USB_RCVF_RE <= 0;
        USB_SNDF_WE <= 0;
        rtc_ctl <= 0;
        HDMI_WR_EN <= 0;
        CORE0_RST <= 0;
        CORE0_BUS <= 0;
        CORE0_DIN <= 0;
        CORE1_RST <= 0;
        CORE1_BUS <= 0;
        CORE1_DIN <= 0;
        state <= ST_INIT;
        TMR0_CLEAR <= 0;
        TMR0_COMP <= 0;
        TMR1_CLEAR <= 0;
        TMR1_COMP <= 0;
        USBPROM_WE <= 0;
        IRQ0_EN <= 0;
        IRQ0_TYPE <= 0;
        IRQ1_EN <= 0;
        IRQ1_TYPE <= 0;
        init_cnt <= 0;
        USBPROM_ADDR <= 0;
        CORE0_CLR <= 0;
        CORE1_CLR <= 0;
        CORE0_ACK <= 0;
        CORE1_ACK <= 0;
        IRQ0_IN[0] <= 0;
        IRQ1_IN[0] <= 0;
        CORE0_CC_ADDR <= 0;
        CORE1_CC_ADDR <= 0;
    end else case (state)
        ST_INIT: begin
            init_cnt <= init_cnt + 1;
            L2_0_ADDR <= init_cnt;
            L2_1_ADDR <= init_cnt;
            L2_2_ADDR <= init_cnt;
            L2_3_ADDR <= init_cnt;
            if (init_cnt == 0) begin
                L2_0_WE <= 'h1fffffffff;
                L2_1_WE <= 'h1fffffffff;
                L2_2_WE <= 'h1fffffffff;
                L2_3_WE <= 'h1fffffffff;
                L2_0_DIN <= 0;
                L2_1_DIN <= 0;
                L2_2_DIN <= 0;
                L2_3_DIN <= 0;
            end else if (init_cnt == 2048) begin
                L2_0_WE <= 0;
                L2_1_WE <= 0;
                L2_2_WE <= 0;
                L2_3_WE <= 0;
            end else if (init_cnt == 2097)
                HDMI_RST <= 1;
            else if (init_cnt == 2117)
                RNG_RST <= 1;
            else if (init_cnt == 2137)
                SD_RST <= 1;
            else if (init_cnt == 2157)
                UART_RST <= 1;
            else if (init_cnt == 2177)
                ETH_RST <= 1;
            else if (init_cnt == 2197)
                USB_SND_RST <= 1;
            if (init_cnt == 2560) begin
                init_cnt <= 0;
                CORE0_RST <= 1;
                CORE1_RST <= 1;
                L2_CNT <= 0;
                state <= ST_IDLE;
            end
        end
        ST_IDLE: begin
            CORE0_CLR <= 0;
            CORE1_CLR <= 0;
            if (CORE0_STATE) begin
                addr <= CORE0_ADDR;
                L2_0_ADDR <= CORE0_ADDR[10:0];
                L2_1_ADDR <= CORE0_ADDR[10:0];
                L2_2_ADDR <= CORE0_ADDR[10:0];
                L2_3_ADDR <= CORE0_ADDR[10:0];
                dout <= CORE0_DOUT;
                dmask <= CORE0_DMASK;
                case (CORE0_STATE)
                    1: state <= ST_RD;
                    2: state <= ST_WR;
                    3: state <= ST_INT;
                    default: begin end
                endcase
                core_n <= 0;
            end else if (CORE1_STATE) begin
                addr <= CORE1_ADDR;
                L2_0_ADDR <= CORE1_ADDR[10:0];
                L2_1_ADDR <= CORE1_ADDR[10:0];
                L2_2_ADDR <= CORE1_ADDR[10:0];
                L2_3_ADDR <= CORE1_ADDR[10:0];
                dout <= CORE1_DOUT;
                dmask <= CORE1_DMASK;
                case (CORE1_STATE)
                    1: state <= ST_RD;
                    2: state <= ST_WR;
                    3: state <= ST_INT;
                    default: begin end
                endcase
                core_n <= 1;
            end
        end
        ST_WAIT: begin              // wait ACK from core
            if (core_n == 0) begin
                CORE0_BUS <= 0;
                CORE0_CLR <= 0;
            end else begin
                CORE1_BUS <= 0;
                CORE1_CLR <= 0;
            end
            SDCMDWE <= 0;
            UART_WR_EN <= 0;
            ETHCMDF_WE <= 0;
            ETHSNDF_WE <= 0;
            USB_SNDF_WE <= 0;
            DDR4_WDF_EN <= 0;
            DDR4_WDF_END <= 0;
            DDR4_EN <= 0;
            TMR0_CLEAR <= 0;
            TMR1_CLEAR <= 0;
            HDMI_WR_EN <= 0;
            rtc_ctl <= 0;
            USBPROM_WE <= 0;
            state <= ST_IDLE;
            CORE0_ACK <= 0;
            CORE1_ACK <= 0;
            L2_0_WE <= 0;
            L2_1_WE <= 0;
            L2_2_WE <= 0;
            L2_3_WE <= 0;
            L2_CNT <= 0;
        end
        ST_RD: begin
            if (addr < 'h2000000) begin                                    // RAM h0 - h40000000
                state <= ST_RDCACHE0;
            end else if ((addr >= 'h4000000) && (addr < 'h4000200)) begin // ROM h80000000 - h80004000
                ROM_ADDR1 <= addr[8:0];
                state <= ST_RDROM1;
            end else if (addr == 'h5000000) begin                          // UART ha0000000
                if (UART_RD_EMPTY) begin
                    `CC_DIN('h100);
                    state <= ST_WAIT;
                    `CC_BUS(1);
                end else begin
                    state <= ST_UARTRD1;
                    UART_RD_RE <= 1;
                end
            end else if (addr == 'h5000400) begin                        // SD ha0008000
                if (SDDATEMPTY == 1) begin
                    `CC_DIN('h100);
                    state <= ST_WAIT;
                    `CC_BUS(1);
                end else begin
                    state <= ST_RDSD1;
                    SDDATRE <= 1;
                end
            end else if (addr == 'h5000800) begin                        // RNG ha0010000
                if (RNGF_EMPTY == 0) begin
                    state <= ST_RDRNG1;
                    RNGF_RE <= 1;
                end
            end else if (addr == 'h5000C00) begin                        // LEDs ha0018000
                `CC_DIN({56'b0, LEDS});
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h5001000) begin                        // ETH ACK ha0020000
                if (ETHACKF_EMPTY == 1) begin
                    `CC_DIN('h100000000);
                    state <= ST_WAIT;
                    `CC_BUS(1);
                end else begin
                    state <= ST_RDETHACK1;
                    ETHACKF_RE <= 1;
                end
            end else if (addr == 'h5001001) begin                        // ETH RCV ha0020020
                if (ETHRCVF_EMPTY == 1) begin
                    `CC_DIN('h100);
                    state <= ST_WAIT;
                    `CC_BUS(1);
                end else begin
                    state <= ST_RDETHRCV1;
                    ETHRCVF_RE <= 1;
                end
            end else if (addr == 'h5001002) begin                        // ETH STATUS ha0020040
                `CC_DIN({48'b0, eth_status});
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h5001402) begin                        // USB RCV ha0028040
                if (USB_RCVF_EMPTY == 1) begin
                    `CC_DIN('h10000);
                    state <= ST_WAIT;
                    `CC_BUS(1);
                end else begin
                    state <= ST_RDUSB1;
                    USB_RCVF_RE <= 1;
                end
            end else if (addr == 'h5001800) begin                        // RTC STATUS ha0030000
                `CC_DIN(rtc_in);
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h7fffff7) begin                        // IRQ EN hfffffee0
                state <= ST_WAIT;
                `CC_BUS(1);
                `CC_DIN(IRQ_EN);
            end else if (addr == 'h7fffff6) begin                        // IRQ TYPE hfffffec0
                state <= ST_WAIT;
                `CC_BUS(1);
                `CC_DIN(IRQ_TYPE);
            end else if (addr == 'h7fffff4) begin                        // TIMER COMPARATOR hfffffe80
                `CC_DIN(TMR_COMP);
                `CC_TMR_CLR(1);
                `CC_BUS(1);
                state <= ST_WAIT;
            end else begin                                                  // DEFAULT
                `CC_DIN('hffffffffffffffff);
                state <= ST_WAIT;
                `CC_BUS(1);
            end
        end
        ST_UARTRD1: begin
            UART_RD_RE <= 0;
            state <= ST_UARTRD2;
        end
        ST_UARTRD2: begin
            state <= ST_UARTRD3;
        end
        ST_UARTRD3: begin
            `CC_DIN({56'b0, UART_RD_DATA});
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_RDSD1: begin
            SDDATRE <= 0;
            state <= ST_RDSD2;
        end
        ST_RDSD2: begin
            state <= ST_RDSD3;
        end
        ST_RDSD3: begin
            `CC_DIN({56'b0, SDDATOUT});
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_RDRNG1: begin
            RNGF_RE <= 0;
            state <= ST_RDRNG2;
        end
        ST_RDRNG2: begin
            state <= ST_RDRNG3;
        end
        ST_RDRNG3: begin
            `CC_DIN({56'b0, RNGF_IN});
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_RDETHACK1: begin
            ETHACKF_RE <= 0;
            state <= ST_RDETHACK2;
        end
        ST_RDETHACK2: begin
            state <= ST_RDETHACK3;
        end
        ST_RDETHACK3: begin
            `CC_DIN({31'b0, ETHACKF_OUT});
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_RDETHRCV1: begin
            ETHRCVF_RE <= 0;
            state <= ST_RDETHRCV2;
        end
        ST_RDETHRCV2: begin
            state <= ST_RDETHRCV3;
        end
        ST_RDETHRCV3: begin
            `CC_DIN({56'b0, ETHRCVF_OUT});
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_RDUSB1: begin
            USB_RCVF_RE <= 0;
            state <= ST_RDUSB2;
        end
        ST_RDUSB2: begin
            state <= ST_RDUSB3;
        end
        ST_RDUSB3: begin
            `CC_DIN({48'b0, USB_RCVF_OUT});
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_RDCACHE0: begin
            state <= ST_RDCACHE1;
        end
        ST_RDCACHE1: begin
            state <= ST_WAIT;
            if ((L2_0_DOUT[281] == 1) && (addr[24:0] == L2_0_DOUT[280:256])) begin
                `CC_DIN(L2_0_DOUT[255:0]);
                `CC_BUS(1);
            end else if ((L2_1_DOUT[281] == 1) && (addr[24:0] == L2_1_DOUT[280:256])) begin
                `CC_DIN(L2_1_DOUT[255:0]);
                `CC_BUS(1);
            end else if ((L2_2_DOUT[281] == 1) && (addr[24:0] == L2_2_DOUT[280:256])) begin
                `CC_DIN(L2_2_DOUT[255:0]);
                `CC_BUS(1);
            end else if ((L2_3_DOUT[281] == 1) && (addr[24:0] == L2_3_DOUT[280:256])) begin
                `CC_DIN(L2_3_DOUT[255:0]);
                `CC_BUS(1);
            end else begin
                DDR4_ADDR <= {addr[24:0], 3'b0};
                DDR4_CMD <= 1;
                DDR4_EN <= 1;
                state <= ST_RDRAM1;
            end
        end
        ST_RDRAM1: begin
            if (DDR4_RDY && DDR4_EN)
                DDR4_EN <= 0;
            if (DDR4_DATA_VALID)
            begin
                `CC_DIN(DDR4_DATA);
                state <= ST_WAIT;
                `CC_BUS(1);
                case (L2_0_DOUT[289:288])
                    0: begin L2_0_WE <= 'h1fffffffff; L2_0_DIN <= {2'b01, 7'b0000001, addr[24:0], DDR4_DATA}; end
                    1: begin L2_1_WE <= 'h1fffffffff; L2_1_DIN <= {2'b00, 7'b0000001, addr[24:0], DDR4_DATA};
                             L2_0_WE <= 'h1000000000; L2_0_DIN[289:288] <= 2'b10;
                    end
                    2: begin L2_2_WE <= 'h1fffffffff; L2_2_DIN <= {2'b00, 7'b0000001, addr[24:0], DDR4_DATA};
                             L2_0_WE <= 'h1000000000; L2_0_DIN[289:288] <= 2'b11;
                    end
                    3: begin L2_3_WE <= 'h1fffffffff; L2_3_DIN <= {2'b00, 7'b0000001, addr[24:0], DDR4_DATA};
                             L2_0_WE <= 'h1000000000; L2_0_DIN[289:288] <= 2'b00;
                    end
                endcase
            end
        end
        ST_RDROM1: begin
            state <= ST_RDROM2;
        end
        ST_RDROM2: begin
            state <= ST_RDROM3;
        end
        ST_RDROM3: begin
            `CC_DIN(ROM_DATA1);
            state <= ST_WAIT;
            `CC_BUS(1);
        end
        ST_WR: begin
            if (addr < 'h2000000) begin                                    // RAM
                if (L2_CNT < 2)
                    L2_CNT <= L2_CNT + 1;
                if (DDR4_RDY && DDR4_WDF_RDY) begin
                    DDR4_ADDR <= {addr[24:0], 3'b0};
                    DDR4_CMD <= 0;
                    state <= ST_WRRAM1;
                    DDR4_WDF_DATA <= dout;
                    DDR4_WDF_MASK <= ~dmask;
                    DDR4_EN <= 1;
                    DDR4_WDF_EN <= 1;
                    DDR4_WDF_END <= 1;
                    `CC_CORE_CC_ADDR({1'b1, addr[31:5]});
                end
            end else if (addr == 'h5000000) begin                          // UART
                if (!UART_WR_FULL && !HDMI_WR_FULL) begin
                    state <= ST_WAIT;
                    UART_WR_EN <= 1;
                    UART_WR_DATA <= dout[7:0];
                    HDMI_WR_EN <= 1;
                    HDMI_WR_DATA <= dout[7:0];
                    `CC_BUS(1);
                end
            end else if (addr == 'h5000400) begin                          // SD
                if (!SDCMDFULL) begin
                    state <= ST_WAIT;
                    SDCMDWE <= 1;
                    SDCMDIN <= dout[39:0];
                    `CC_BUS(1);
                end
            end else if (addr == 'h5000C00) begin                          // LEDs
                state <= ST_WAIT;
                `CC_BUS(1);
                LEDS <= dout[7:0];
            end else if (addr == 'h5001000) begin                          // ETH CMD
                if (!ETHCMDF_FULL) begin
                    state <= ST_WAIT;
                    ETHCMDF_WE <= 1;
                    ETHCMDF_IN <= dout;
                    `CC_BUS(1);
                end
            end else if (addr == 'h5001001) begin                          // ETH SND
                if (!ETHSNDF_FULL) begin
                    state <= ST_WAIT;
                    ETHSNDF_WE <= 1;
                    ETHSNDF_IN <= dout[7:0];
                    `CC_BUS(1);
                end
            end else if (addr == 'h5001400) begin    // USB "PROM"
                USBPROM_WE <= 1;
                USBPROM_ADDR <= USBPROM_ADDR + 1;
                USBPROM_DIN <= dout[31:0];
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h5001401) begin    // USB RESET
                USB_RST <= dout[0];
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h5001402) begin    // USB SND
                if (!USB_SNDF_FULL) begin
                    state <= ST_WAIT;
                    USB_SNDF_WE <= 1;
                    USB_SNDF_IN <= dout[15:0];
                    `CC_BUS(1);
                end
            end else if (addr == 'h5001800) begin    // RTC CTL
                state <= ST_WAIT;
                `CC_BUS(1);
                rtc_ctl <= dout;
            end else if (addr == 'h7fffff7) begin    // IRQ EN
                `CC_IRQ_EN(dout);
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h7fffff6) begin    // IRQ TYPE
                `CC_IRQ_TYPE(dout);
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h7fffff5) begin    // send IPI
                IPI <= dout[7:0];
                if (dout[8] == 0)
                    IRQ0_IN[0] <= dout[16];
                else
                    IRQ1_IN[0] <= dout[16];
                state <= ST_WAIT;
                `CC_BUS(1);
            end else if (addr == 'h7fffff4) begin    // TIMER COMPARATOR
                `CC_TMR_COMP(dout[31:0]);
                state <= ST_WAIT;
                `CC_BUS(1);
            end else begin                              // DEFAULT
                state <= ST_WAIT;
                `CC_BUS(1);
            end
        end
        ST_WRRAM1: begin
            if (L2_CNT < 2)
                L2_CNT <= L2_CNT + 1;
            `CC_CORE_CC_ADDR(0);
            if (DDR4_RDY && DDR4_EN)
                DDR4_EN <= 0;
            if (DDR4_WDF_RDY && DDR4_WDF_EN)
                DDR4_WDF_EN <= 0;
            if (!DDR4_EN && !DDR4_WDF_EN)
            begin
                if (L2_CNT == 2) begin
                    state <= ST_WAIT;
                    `CC_BUS(1);
                    if ((L2_0_DOUT[281] == 1) && (addr[24:0] == L2_0_DOUT[280:256])) begin
                        L2_0_WE[31:0] <= dmask;
                        L2_0_DIN[255:0] <= dout;
                    end else if ((L2_1_DOUT[281] == 1) && (addr[24:0] == L2_1_DOUT[280:256])) begin
                        L2_1_WE[31:0] <= dmask;
                        L2_1_DIN[255:0] <= dout;
                    end else if ((L2_2_DOUT[281] == 1) && (addr[24:0] == L2_2_DOUT[280:256])) begin
                        L2_2_WE[31:0] <= dmask;
                        L2_2_DIN[255:0] <= dout;
                    end else if ((L2_3_DOUT[281] == 1) && (addr[24:0] == L2_3_DOUT[280:256])) begin
                        L2_3_WE[31:0] <= dmask;
                        L2_3_DIN[255:0] <= dout;
                    end
                end
            end
        end
        ST_INT: begin
            state <= ST_WAIT;
            `CC_BUS(1);
            `CC_DIN(CORE_IRQ);
            `CC_IRQ_EN_BIT(CORE_IRQ, 0);
            `CC_CORE_ACK(1);
        end
        default: begin
        end
    endcase
end

// TIMER INTERRUPT core 0
reg [7:0]       TMR0_CNT;
reg [31:0]      TMR0_TICK;
always @(posedge CLK)
begin
    if (CORE0_RST == 0) begin
        TMR0_CNT <= 0;
        TMR0_TICK <= 0;
        IRQ0_IN[1] <= 0;
    end else begin
        if (TMR0_CLEAR == 1) begin
            IRQ0_IN[1] <= 0;
            TMR0_TICK <= 0;
            TMR0_CNT <= 0;
        end else if ((IRQ0_IN[1] == 0) && (TMR0_COMP != 0)) begin
            if (TMR0_CNT == 199) begin
                TMR0_CNT <= 0;
                TMR0_TICK <= TMR0_TICK + 1;
            end else
                TMR0_CNT <= TMR0_CNT + 1;
            if (TMR0_TICK == TMR0_COMP) begin
                TMR0_TICK <= 0;
                IRQ0_IN[1] <= 1;
            end
        end
    end
end

// TIMER INTERRUPT core 1
reg [7:0]       TMR1_CNT;
reg [31:0]      TMR1_TICK;
always @(posedge CLK)
begin
    if (CORE1_RST == 0) begin
        TMR1_CNT <= 0;
        TMR1_TICK <= 0;
        IRQ1_IN[1] <= 0;
    end else begin
        if (TMR1_CLEAR == 1) begin
            IRQ1_IN[1] <= 0;
            TMR1_TICK <= 0;
            TMR1_CNT <= 0;
        end else if ((IRQ1_IN[1] == 0) && (TMR1_COMP != 0)) begin
            if (TMR1_CNT == 199) begin
                TMR1_CNT <= 0;
                TMR1_TICK <= TMR1_TICK + 1;
            end else
                TMR1_CNT <= TMR1_CNT + 1;
            if (TMR1_TICK == TMR1_COMP) begin
                TMR1_TICK <= 0;
                IRQ1_IN[1] <= 1;
            end
        end
    end
end

// INTERRUPT HANDLING FOR UART
always @(posedge CLK)
begin
    if (CORE0_RST == 0) begin
        IRQ0_IN[4] <= 0;
        IRQ1_IN[4] <= 0;
    end else begin
        if (UART_RD_EMPTY == 0) begin
            IRQ0_IN[4] <= 1;
            IRQ1_IN[4] <= 1;
        end else begin
            IRQ0_IN[4] <= 0;
            IRQ1_IN[4] <= 0;
        end
    end
end

// INTERRUPT HANDLING FOR USB
always @(posedge CLK)
begin
    if (CORE0_RST == 0) begin
        IRQ0_IN[3] <= 0;
        IRQ1_IN[3] <= 0;
    end else begin
        if (USB_RCVF_EMPTY == 0) begin
            IRQ0_IN[3] <= 1;
            IRQ1_IN[3] <= 1;
        end else begin
            IRQ0_IN[3] <= 0;
            IRQ1_IN[3] <= 0;
        end
    end
end

// INTERRUPT HANDLING FOR SD
always @(posedge CLK)
begin
    if (CORE0_RST == 0) begin
        IRQ0_IN[5] <= 0;
        IRQ1_IN[5] <= 0;
    end else begin
        if (SDDATEMPTY == 0) begin
            IRQ0_IN[5] <= 1;
            IRQ1_IN[5] <= 1;
        end else begin
            IRQ0_IN[5] <= 0;
            IRQ1_IN[5] <= 0;
        end
    end
end

// INTERRUPT HANDLING FOR ETH
always @(posedge CLK)
begin
    if (CORE0_RST == 0) begin
        IRQ0_IN[2] <= 0;
        IRQ1_IN[2] <= 0;
    end else begin
        if (ETHACKF_EMPTY == 0) begin
            IRQ0_IN[2] <= 1;
            IRQ1_IN[2] <= 1;
        end else begin
            IRQ0_IN[2] <= 0;
            IRQ1_IN[2] <= 0;
        end
    end
end

// INTERRUPT CONTROLLER core 0
reg [5:0]           IRQ0_OLD;
always @(posedge CLK)
begin
    if (CORE0_RST == 0) begin
        CORE0_INTR <= 0;
        IRQ0_OLD <= 0;
    end else begin
        IRQ0_OLD <= IRQ0_IN;
        if (CORE0_INTR == 1) begin
            if (CORE0_ACK == 1)
                CORE0_INTR <= 0;
        end else if ((IRQ0_IN[0] == 1) && (IRQ0_EN[0] == 1) && ((IRQ0_TYPE[0] == 0) || ((IRQ0_TYPE[0] == 1) && (IRQ0_OLD[0] == 0)))) begin
            CORE0_IRQ <= IPI;
            CORE0_INTR <= 1;
        end else if ((IRQ0_IN[1] == 1) && (IRQ0_EN[1] == 1) && ((IRQ0_TYPE[1] == 0) || ((IRQ0_TYPE[1] == 1) && (IRQ0_OLD[1] == 0)))) begin
            CORE0_IRQ <= 1;
            CORE0_INTR <= 1;
        end else if ((IRQ0_IN[2] == 1) && (IRQ0_EN[2] == 1) && ((IRQ0_TYPE[2] == 0) || ((IRQ0_TYPE[2] == 1) && (IRQ0_OLD[2] == 0)))) begin
            CORE0_IRQ <= 2;
            CORE0_INTR <= 1;
        end else if ((IRQ0_IN[3] == 1) && (IRQ0_EN[3] == 1) && ((IRQ0_TYPE[3] == 0) || ((IRQ0_TYPE[3] == 1) && (IRQ0_OLD[3] == 0)))) begin
            CORE0_IRQ <= 3;
            CORE0_INTR <= 1;
        end else if ((IRQ0_IN[4] == 1) && (IRQ0_EN[4] == 1) && ((IRQ0_TYPE[4] == 0) || ((IRQ0_TYPE[4] == 1) && (IRQ0_OLD[4] == 0)))) begin
            CORE0_IRQ <= 4;
            CORE0_INTR <= 1;
        end else if ((IRQ0_IN[5] == 1) && (IRQ0_EN[5] == 1) && ((IRQ0_TYPE[5] == 0) || ((IRQ0_TYPE[5] == 1) && (IRQ0_OLD[5] == 0)))) begin
            CORE0_IRQ <= 5;
            CORE0_INTR <= 1;
        end
    end
end

// INTERRUPT CONTROLLER core 1
reg [5:0]           IRQ1_OLD;
always @(posedge CLK)
begin
    if (CORE1_RST == 0) begin
        CORE1_INTR <= 0;
        IRQ1_OLD <= 0;
    end else begin
        IRQ1_OLD <= IRQ1_IN;
        if (CORE1_INTR == 1) begin
            if (CORE1_ACK == 1)
                CORE1_INTR <= 0;
        end else if ((IRQ1_IN[0] == 1) && (IRQ1_EN[0] == 1) && ((IRQ1_TYPE[0] == 0) || ((IRQ1_TYPE[0] == 1) && (IRQ1_OLD[0] == 0)))) begin
            CORE1_IRQ <= IPI;
            CORE1_INTR <= 1;
        end else if ((IRQ1_IN[1] == 1) && (IRQ1_EN[1] == 1) && ((IRQ1_TYPE[1] == 0) || ((IRQ1_TYPE[1] == 1) && (IRQ1_OLD[1] == 0)))) begin
            CORE1_IRQ <= 1;
            CORE1_INTR <= 1;
        end else if ((IRQ1_IN[2] == 1) && (IRQ1_EN[2] == 1) && ((IRQ1_TYPE[2] == 0) || ((IRQ1_TYPE[2] == 1) && (IRQ1_OLD[2] == 0)))) begin
            CORE1_IRQ <= 2;
            CORE1_INTR <= 1;
        end else if ((IRQ1_IN[3] == 1) && (IRQ1_EN[3] == 1) && ((IRQ1_TYPE[3] == 0) || ((IRQ1_TYPE[3] == 1) && (IRQ1_OLD[3] == 0)))) begin
            CORE1_IRQ <= 3;
            CORE1_INTR <= 1;
        end else if ((IRQ1_IN[4] == 1) && (IRQ1_EN[4] == 1) && ((IRQ1_TYPE[4] == 0) || ((IRQ1_TYPE[4] == 1) && (IRQ1_OLD[4] == 0)))) begin
            CORE1_IRQ <= 4;
            CORE1_INTR <= 1;
        end else if ((IRQ1_IN[5] == 1) && (IRQ1_EN[5] == 1) && ((IRQ1_TYPE[5] == 0) || ((IRQ1_TYPE[5] == 1) && (IRQ1_OLD[5] == 0)))) begin
            CORE1_IRQ <= 5;
            CORE1_INTR <= 1;
        end
    end
end

// ETH status pipelining for timing
always @(posedge CLK)
begin
    eth_status2 <= ETH_STATUS;
    eth_status <= eth_status2;
end

// RTC pipelining for timing
always @(posedge CLK)
begin
    RTC_CTL <= rtc_ctl2;
    rtc_ctl2 <= rtc_ctl;
    rtc_in2 <= RTC_IN;
    rtc_in <= rtc_in2;
end

endmodule
