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

module ETHDRV(
    input               CLK200,
    input               CLK125,
    input               RST,
    output reg          FIFO_RST,
    input               eth1_mdio_in,
    output              eth1_mdio_out,
    output              eth1_mdio_state,
    output              ETH1_MDC,
    output              ETH1_RST_N,
    input [3:0]         ETH1_RX,
    input               ETH1_RX_CTL,
    input               ETH1_RX_CLK,
    output [3:0]        ETH1_TX,
    output              ETH1_TX_CTL,
    output              ETH1_TX_CLK,
    output reg [15:0]   ETH_STATUS,
    input [63:0]        ETHCMDF_OUT,
    output reg          ETHCMDF_RE,
    input               ETHCMDF_EMPTY,
    output reg [31:0]   ETHACKF_IN,
    output reg          ETHACKF_WE,
    input               ETHACKF_FULL,
    output reg [7:0]    ETHRCVF_IN,
    output reg          ETHRCVF_WE,
    input               ETHRCVF_FULL,
    input [7:0]         ETHSNDF_OUT,
    output reg          ETHSNDF_RE,
    input               ETHSNDF_EMPTY
);

assign ETH1_RST_N = RST;

reg [15:0]  eth_status;
reg [15:0]  eth_status2;

reg [15:0]  frm_cnt;
reg [31:0]  rst_cnt;
reg [15:0]  ig_cnt;
reg [7:0]   data;
reg         ctl;
wire [7:0]  data_in;
reg [1:0]   rxl;
reg [7:0]   rx_step;
reg [15:0]  rcv_cnt;
reg         sync_rst;
reg [1:0]   sync_rst_rx;
reg [1:0]   sync_rst_tx;
wire        rx_rst;
wire        tx_rst;
reg [63:0]  cmd;
reg [15:0]  pkt_size;

assign rx_rst = sync_rst_rx[1];
assign tx_rst = sync_rst_tx[1];

ODDRE1 tx0(.Q(ETH1_TX[0]), .C(CLK125), .D1(data[0]), .D2(data[4]), .SR(1'b0));
ODDRE1 tx1(.Q(ETH1_TX[1]), .C(CLK125), .D1(data[1]), .D2(data[5]), .SR(1'b0));
ODDRE1 tx2(.Q(ETH1_TX[2]), .C(CLK125), .D1(data[2]), .D2(data[6]), .SR(1'b0));
ODDRE1 tx3(.Q(ETH1_TX[3]), .C(CLK125), .D1(data[3]), .D2(data[7]), .SR(1'b0));
ODDRE1 txt(.Q(ETH1_TX_CTL), .C(CLK125), .D1(ctl), .D2(ctl), .SR(1'b0));
ODDRE1 txc(.Q(ETH1_TX_CLK), .C(CLK125), .D1(1'b1), .D2(1'b0), .SR(1'b0));

IDDRE1 rx0(.Q1(data_in[0]), .Q2(data_in[4]), .C(ETH1_RX_CLK), .CB(~ETH1_RX_CLK), .D(ETH1_RX[0]), .R(1'b0));
IDDRE1 rx1(.Q1(data_in[1]), .Q2(data_in[5]), .C(ETH1_RX_CLK), .CB(~ETH1_RX_CLK), .D(ETH1_RX[1]), .R(1'b0));
IDDRE1 rx2(.Q1(data_in[2]), .Q2(data_in[6]), .C(ETH1_RX_CLK), .CB(~ETH1_RX_CLK), .D(ETH1_RX[2]), .R(1'b0));
IDDRE1 rx3(.Q1(data_in[3]), .Q2(data_in[7]), .C(ETH1_RX_CLK), .CB(~ETH1_RX_CLK), .D(ETH1_RX[3]), .R(1'b0));
IDDRE1 rxc(.Q1(rxl[0]), .Q2(rxl[1]), .C(ETH1_RX_CLK), .CB(~ETH1_RX_CLK), .D(ETH1_RX_CTL), .R(1'b0));

initial
begin
    frm_cnt <= 0;
    rst_cnt <= 0;
    data <= 0;
    ctl <= 0;
    rx_step <= 0;
    sync_rst_rx <= 0;
    sync_rst_tx <= 0;
    sync_rst <= 0;
    ETHACKF_WE <= 0;
    ETHRCVF_WE <= 0;
    ETHCMDF_RE <= 0;
    ETHSNDF_RE <= 0;
end

// RST dual-FF synchro
always @(posedge CLK200)
begin
    sync_rst <= RST;
end

always @(posedge ETH1_RX_CLK)
begin
    sync_rst_rx[0] <= sync_rst;
    sync_rst_rx[1] <= sync_rst_rx[0];
end

always @(posedge CLK125)
begin
    sync_rst_tx[0] <= sync_rst;
    sync_rst_tx[1] <= sync_rst_tx[0];
end

// ETH RX
always @(posedge ETH1_RX_CLK)
begin
    if (!rx_rst) begin
        FIFO_RST <= 0;
        rx_step <= 0;
        ETHACKF_WE <= 0;
        ETHRCVF_WE <= 0;
    end else begin
        case (rx_step)
            0: begin
                FIFO_RST <= 1;
                rcv_cnt <= 0;
                ETHACKF_WE <= 0;
                if ((rxl[0] == 1) && (rxl[1] == 1)) begin
                    if (data_in == 'hd5)
                        rx_step <= 1;
                    else if (data_in != 'h55)
                        rx_step <= 127;
                end
            end
            1: begin
                if (rxl[0] == 0) begin
                    rx_step <= 0;
                    ETHACKF_WE <= 1;
                    ETHACKF_IN <= {16'h1, rcv_cnt};
                    ETHRCVF_WE <= 0;
                end else begin
                    rcv_cnt <= rcv_cnt + 1;
                    ETHRCVF_WE <= 1;
                    ETHRCVF_IN <= data_in;
                end
            end
            127: if (rxl[0] == 0) rx_step <= 0;
        endcase
    end
end

// ETH TX
always @(posedge CLK125)
begin
    if (!tx_rst) begin
        frm_cnt <= 0;
        ETHCMDF_RE <= 0;
        ETHSNDF_RE <= 0;
    end else begin
        case (frm_cnt)
            0: begin
                if (ETHCMDF_EMPTY == 0) begin
                    ETHCMDF_RE <= 1;
                    frm_cnt <= 1;
                end
            end
            1: begin
                ETHCMDF_RE <= 0;
                frm_cnt <= 2;
            end
            2: begin
                frm_cnt <= 3;
                cmd <= ETHCMDF_OUT;
            end
            3: begin
                case (cmd[7:0])
                    1: begin        // SEND packet
                        pkt_size <= cmd[23:8];
                        frm_cnt <= 5;
                    end
                    default: frm_cnt <= 0;
                endcase
            end
            5: begin
                ctl <= 1;
                data <= 'h55;
                frm_cnt <= 6;
            end
            6: begin
                frm_cnt <= 7;
            end
            7: begin
                frm_cnt <= 8;
            end
            8: begin
                frm_cnt <= 9;
            end
            9: begin
                frm_cnt <= 10;
            end
            10: begin
                frm_cnt <= 11;
            end
            11: begin
                frm_cnt <= 12;
                ETHSNDF_RE <= 1;
            end
            12: begin
                frm_cnt <= 13;
                data <= 'hd5;
            end
            13: begin
                data <= ETHSNDF_OUT;
                pkt_size <= pkt_size - 1;
                if (pkt_size == 1) begin
                    ETHSNDF_RE <= 0;
                    frm_cnt <= 100;
                end
            end
            100: begin
                ctl <= 0;
                frm_cnt <= 101;
                ig_cnt <= 0;
                data <= 0;
            end
            101: begin
                if (ig_cnt < 24)
                    ig_cnt <= ig_cnt + 1;
                else
                    frm_cnt <= 0;
            end
        endcase
    end
end

// PHY Status Register continuous updating
reg [31:0]  smi_cnt;
reg         smi_out;
reg [15:0]  smi_in;
reg         smi_rd;
reg         smi_ck;

assign eth1_mdio_state = smi_rd;
assign eth1_mdio_out = smi_out;
assign ETH1_MDC = smi_ck;

always @(posedge CLK200)
begin
    if (!RST) begin
        smi_cnt <= 0;
        eth_status <= 0;
    end else begin
        ETH_STATUS <= eth_status2;
        eth_status2 <= eth_status;
        smi_cnt <= smi_cnt + 1;
        case (smi_cnt)
            0: begin
                smi_rd <= 1;
                smi_ck <= 0;
                smi_out <= 0;
            end
            // IDLE
            49999670: smi_out <= 1;
            49999671: smi_rd <= 0;
            49999680: smi_ck <= 1;
            49999685: smi_ck <= 0;
            49999690: smi_ck <= 1;
            49999695: smi_ck <= 0;
            49999700: smi_ck <= 1;
            49999705: smi_ck <= 0;
            49999710: smi_ck <= 1;
            49999715: smi_ck <= 0;
            49999720: smi_ck <= 1;
            49999725: smi_ck <= 0;
            49999730: smi_ck <= 1;
            49999735: smi_ck <= 0;
            49999740: smi_ck <= 1;
            49999745: smi_ck <= 0;
            49999750: smi_ck <= 1;
            49999755: smi_ck <= 0;
            49999760: smi_ck <= 1;
            49999765: smi_ck <= 0;
            49999770: smi_ck <= 1;
            49999775: smi_ck <= 0;
            49999780: smi_ck <= 1;
            49999785: smi_ck <= 0;
            49999790: smi_ck <= 1;
            49999795: smi_ck <= 0;
            49999800: smi_ck <= 1;
            49999805: smi_ck <= 0;
            49999810: smi_ck <= 1;
            49999815: smi_ck <= 0;
            49999820: smi_ck <= 1;
            49999825: smi_ck <= 0;
            49999830: smi_ck <= 1;
            49999835: smi_ck <= 0;
            49999840: smi_ck <= 1;
            49999845: smi_ck <= 0;
            49999850: smi_ck <= 1;
            49999855: smi_ck <= 0;
            49999860: smi_ck <= 1;
            49999865: smi_ck <= 0;
            49999870: smi_ck <= 1;
            49999875: smi_ck <= 0;
            49999880: smi_ck <= 1;
            49999885: smi_ck <= 0;
            49999890: smi_ck <= 1;
            49999895: smi_ck <= 0;
            49999900: smi_ck <= 1;
            49999905: smi_ck <= 0;
            49999910: smi_ck <= 1;
            49999915: smi_ck <= 0;
            49999920: smi_ck <= 1;
            49999925: smi_ck <= 0;
            49999930: smi_ck <= 1;
            49999935: smi_ck <= 0;
            49999940: smi_ck <= 1;
            49999945: smi_ck <= 0;
            49999950: smi_ck <= 1;
            49999955: smi_ck <= 0;
            49999960: smi_ck <= 1;
            49999965: smi_ck <= 0;
            49999970: smi_ck <= 1;
            49999975: smi_ck <= 0;
            49999980: smi_ck <= 1;
            49999985: smi_ck <= 0;
            49999990: smi_ck <= 1;
            49999995: smi_ck <= 0;
            50000000: smi_ck <= 1;
            50000005: smi_ck <= 0;
            // START (01)
            50000006: smi_out <= 0;
            50000010: smi_ck <= 1;
            50000015: smi_ck <= 0;
            50000016: smi_out <= 1;
            50000020: smi_ck <= 1;
            50000025: smi_ck <= 0;
            // READ (10)
            50000030: smi_ck <= 1;
            50000035: smi_ck <= 0;
            50000036: smi_out <= 0;
            50000040: smi_ck <= 1;
            50000045: smi_ck <= 0;
            // PHY ADDR (00001)
            50000050: smi_ck <= 1;
            50000055: smi_ck <= 0;
            50000060: smi_ck <= 1;
            50000065: smi_ck <= 0;
            50000070: smi_ck <= 1;
            50000075: smi_ck <= 0;
            50000080: smi_ck <= 1;
            50000085: smi_ck <= 0;
            50000086: smi_out <= 1;
            50000090: smi_ck <= 1;
            50000095: smi_ck <= 0;
            // PHY REG (10001)
            50000100: smi_ck <= 1;
            50000105: smi_ck <= 0;
            50000106: smi_out <= 0;
            50000110: smi_ck <= 1;
            50000115: smi_ck <= 0;
            50000120: smi_ck <= 1;
            50000125: smi_ck <= 0;
            50000130: smi_ck <= 1;
            50000135: smi_ck <= 0;
            50000136: smi_out <= 1;
            50000140: smi_ck <= 1;
            50000144: smi_rd <= 1;
            50000145: smi_ck <= 0;
            // TA
            50000150: smi_ck <= 1;
            50000155: smi_ck <= 0;
            50000160: smi_ck <= 1;
            50000165: smi_ck <= 0;
            // DATA READ
            50000170: begin smi_ck <= 1; smi_in[15] <= eth1_mdio_in; end
            50000175: smi_ck <= 0;
            50000180: begin smi_ck <= 1; smi_in[14] <= eth1_mdio_in; end
            50000185: smi_ck <= 0;
            50000190: begin smi_ck <= 1; smi_in[13] <= eth1_mdio_in; end
            50000195: smi_ck <= 0;
            50000200: begin smi_ck <= 1; smi_in[12] <= eth1_mdio_in; end
            50000205: smi_ck <= 0;
            50000210: begin smi_ck <= 1; smi_in[11] <= eth1_mdio_in; end
            50000215: smi_ck <= 0;
            50000220: begin smi_ck <= 1; smi_in[10] <= eth1_mdio_in; end
            50000225: smi_ck <= 0;
            50000230: begin smi_ck <= 1; smi_in[9] <= eth1_mdio_in; end
            50000235: smi_ck <= 0;
            50000240: begin smi_ck <= 1; smi_in[8] <= eth1_mdio_in; end
            50000245: smi_ck <= 0;
            50000250: begin smi_ck <= 1; smi_in[7] <= eth1_mdio_in; end
            50000255: smi_ck <= 0;
            50000260: begin smi_ck <= 1; smi_in[6] <= eth1_mdio_in; end
            50000265: smi_ck <= 0;
            50000270: begin smi_ck <= 1; smi_in[5] <= eth1_mdio_in; end
            50000275: smi_ck <= 0;
            50000280: begin smi_ck <= 1; smi_in[4] <= eth1_mdio_in; end
            50000285: smi_ck <= 0;
            50000290: begin smi_ck <= 1; smi_in[3] <= eth1_mdio_in; end
            50000295: smi_ck <= 0;
            50000300: begin smi_ck <= 1; smi_in[2] <= eth1_mdio_in; end
            50000305: smi_ck <= 0;
            50000310: begin smi_ck <= 1; smi_in[1] <= eth1_mdio_in; end
            50000315: smi_ck <= 0;
            50000320: begin smi_ck <= 1; smi_in[0] <= eth1_mdio_in; end
            50000325: smi_ck <= 0;
            // IDLE
            50000330: smi_ck <= 1;
            50000331: eth_status <= smi_in; 
            50000335: smi_ck <= 0;
            50000340: smi_ck <= 1;
            50000345: smi_ck <= 0;
            50000346: smi_cnt <= 0;
            default: begin end
        endcase
    end
end

endmodule
