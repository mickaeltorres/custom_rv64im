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

module RTC(
    input               CLK200,
    output reg          RTC_SCL,
    input               RTC_SDA_IN,
    output reg          RTC_SDA_OUT,
    output reg [63:0]   RTC_OUT,
    input [63:0]        RTC_CTL
);

reg [63:0]  ctl;
reg [63:0]  ctl1;
reg [63:0]  ctl2;
reg [63:0]  out;
reg [63:0]  out1;
reg [63:0]  out2;
reg [31:0]  cnt;
reg [3:0]   bcnt;
reg [7:0]   CMD_WR = 'b11011110;
reg [7:0]   CMD_RD = 'b11011111;
reg [7:0]   CONF_ADDR = 'h07;
reg [7:0]   CONF_VAL = 'h0;
reg [7:0]   START_ADDR = 'h00;
reg [55:0]  reg_in;
reg [55:0]  reg_out;
reg [5:0]   reg_cnt;

initial
begin
    RTC_SCL <= 1;
    RTC_SDA_OUT <= 1;
    cnt <= 0;
    out <= 0;
    out1 <= 0;
    out2 <= 0;
    RTC_OUT <= 0;
    ctl <= 0;
    ctl1 <= 0;
    ctl2 <= 0;
end

always @(posedge CLK200)
begin
    ctl2 <= RTC_CTL;
    ctl1 <= ctl2;
    ctl <= ctl1;
    RTC_OUT <= out2;
    out2 <= out1;
    out1 <= out;
    case (cnt)
        0: begin
            RTC_SDA_OUT <= 1;
            RTC_SCL <= 1;
            if (ctl[0] == 1) begin
                if (ctl[1] == 1) begin
                    cnt <= 1;
                    out <= 'b10;
                end else if (ctl[2] == 1) begin
                    cnt <= 10000;
                    out <= 'b100;
                end else if (ctl[3] == 1) begin
                    cnt <= 20000;
                    out <= 'b1000;
                    reg_out[0] <= 1; // enable oscii
                    reg_out[1] <= ctl[14];
                    reg_out[2] <= ctl[13];
                    reg_out[3] <= ctl[12];
                    reg_out[4] <= ctl[11];
                    reg_out[5] <= ctl[10];
                    reg_out[6] <= ctl[9];
                    reg_out[7] <= ctl[8];
                    reg_out[8] <= 0;
                    reg_out[9] <= ctl[22];
                    reg_out[10] <= ctl[21];
                    reg_out[11] <= ctl[20];
                    reg_out[12] <= ctl[19];
                    reg_out[13] <= ctl[18];
                    reg_out[14] <= ctl[17];
                    reg_out[15] <= ctl[16];
                    reg_out[16] <= 0;
                    reg_out[17] <= 0; // 24 hours mode
                    reg_out[18] <= ctl[29];
                    reg_out[19] <= ctl[28];
                    reg_out[20] <= ctl[27];
                    reg_out[21] <= ctl[26];
                    reg_out[22] <= ctl[25];
                    reg_out[23] <= ctl[24];
                    reg_out[24] <= 0;
                    reg_out[25] <= 0;
                    reg_out[26] <= 0;
                    reg_out[27] <= 0;
                    reg_out[28] <= 0;
                    reg_out[29] <= ctl[34];
                    reg_out[30] <= ctl[33];
                    reg_out[31] <= ctl[32];
                    reg_out[32] <= 0;
                    reg_out[33] <= 0;
                    reg_out[34] <= ctl[45];
                    reg_out[35] <= ctl[44];
                    reg_out[36] <= ctl[43];
                    reg_out[37] <= ctl[42];
                    reg_out[38] <= ctl[41];
                    reg_out[39] <= ctl[40];
                    reg_out[40] <= 0;
                    reg_out[41] <= 0;
                    reg_out[42] <= 0;
                    reg_out[43] <= ctl[52];
                    reg_out[44] <= ctl[51];
                    reg_out[45] <= ctl[50];
                    reg_out[46] <= ctl[49];
                    reg_out[47] <= ctl[48];
                    reg_out[48] <= ctl[63];
                    reg_out[49] <= ctl[62];
                    reg_out[50] <= ctl[61];
                    reg_out[51] <= ctl[60];
                    reg_out[52] <= ctl[59];
                    reg_out[53] <= ctl[58];
                    reg_out[54] <= ctl[57];
                    reg_out[55] <= ctl[56];
                end
            end
        end
        
        // configure everything off
        // start bit
        125: begin
            RTC_SDA_OUT <= 0;
            cnt <= cnt + 1;
        end
        250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
            bcnt <= 7;
        end
        // send ADDR + WR
        375: begin
            RTC_SDA_OUT <= CMD_WR[bcnt];
            cnt <= cnt + 1;
        end
        500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        750: begin
            RTC_SCL <= 0;
            if (bcnt == 0) begin
                cnt <= cnt + 1;
                RTC_SDA_OUT <= 1;
            end else begin
                cnt <= 251;
                bcnt <= bcnt - 1;
            end
        end
        // check ACK
        1000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        1125: begin
            if (RTC_SDA_IN == 0) begin
                bcnt <= 7;
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        // send CONFIG reg
        1250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        1375: begin
            RTC_SDA_OUT <= CONF_ADDR[bcnt];
            cnt <= cnt + 1;
        end
        1500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        1625: begin
            if (bcnt > 0) begin
                bcnt <= bcnt - 1;
                cnt <= 1126;
            end else begin
                cnt <= cnt + 1;
            end
        end
        // check ACK
        1750: begin
            RTC_SDA_OUT <= 1;
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        2000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        2125: begin
            if (RTC_SDA_IN == 0) begin
                bcnt <= 7;
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        // send CONFIG value
        2250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        2375: begin
            RTC_SDA_OUT <= CONF_VAL[bcnt];
            cnt <= cnt + 1;
        end
        2500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        2625: begin
            if (bcnt > 0) begin
                bcnt <= bcnt - 1;
                cnt <= 2126;
            end else begin
                cnt <= cnt + 1;
            end
        end
        // check ACK
        2750: begin
            RTC_SDA_OUT <= 1;
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        3000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        3125: begin
            if (RTC_SDA_IN == 0) begin
                out[7] <= 0;
                cnt <= cnt + 1;
            end else begin
                out[7] <= 1;
                cnt <= 50000;
            end
        end
        3250: begin
            RTC_SDA_OUT <= 0;
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        3500: begin
            RTC_SCL <= 1;
            cnt <= 49751;
        end
        
        // READ all time/date registers
        // start bit
        10125: begin
            RTC_SDA_OUT <= 0;
            cnt <= cnt + 1;
        end
        10250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
            bcnt <= 7;
        end
        // send ADDR + WR
        10375: begin
            RTC_SDA_OUT <= CMD_WR[bcnt];
            cnt <= cnt + 1;
        end
        10500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        10750: begin
            RTC_SCL <= 0;
            if (bcnt == 0) begin
                cnt <= cnt + 1;
                RTC_SDA_OUT <= 1;
            end else begin
                cnt <= 10251;
                bcnt <= bcnt - 1;
            end
        end
        // check ACK
        11000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        11125: begin
            if (RTC_SDA_IN == 0) begin
                bcnt <= 7;
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        // send reg 0 ADDR
        11250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        11375: begin
            RTC_SDA_OUT <= START_ADDR[bcnt];
            cnt <= cnt + 1;
        end
        11500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        11625: begin
            if (bcnt > 0) begin
                bcnt <= bcnt - 1;
                cnt <= 11126;
            end else begin
                cnt <= cnt + 1;
            end
        end
        // check ACK
        11750: begin
            RTC_SDA_OUT <= 1;
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        12000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        12125: begin
            if (RTC_SDA_IN == 0) begin
                bcnt <= 7;
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        12250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        12500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        // start bit
        12626: begin
            RTC_SDA_OUT <= 0;
            cnt <= cnt + 1;
        end
        12750: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
            bcnt <= 7;
        end
        // send ADDR + RD
        12875: begin
            RTC_SDA_OUT <= CMD_RD[bcnt];
            cnt <= cnt + 1;
        end
        13000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        13250: begin
            RTC_SCL <= 0;
            if (bcnt == 0) begin
                cnt <= cnt + 1;
                RTC_SDA_OUT <= 1;
            end else begin
                cnt <= 12751;
                bcnt <= bcnt - 1;
            end
        end
        // check ACK
        13500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        13625: begin
            if (RTC_SDA_IN == 0) begin
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        13750: begin
            RTC_SCL <= 0;
            reg_cnt <= 0;
            bcnt <= 0;
            cnt <= cnt + 1;
        end
        // read
        14000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        14125: begin
            reg_in[reg_cnt] <= RTC_SDA_IN;
            reg_cnt <= reg_cnt + 1;
            cnt <= cnt + 1;
        end
        14250: begin
            RTC_SCL <= 0;
            bcnt <= bcnt + 1;
            if (bcnt == 7)
                cnt <= cnt + 1;
            else
                cnt <= 13751;
        end
        14325: begin
            if (reg_cnt != 56)
                RTC_SDA_OUT <= 0;
            cnt <= cnt + 1;
        end
        14500: begin
            bcnt <= 0;
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        14750: begin
            RTC_SCL <= 0;
            if (reg_cnt != 56) begin
                RTC_SDA_OUT <= 1;
                cnt <= 13751;
            end else begin
                cnt <= cnt + 1;
            end
        end
        14751: begin
            RTC_SDA_OUT <= 0;
            out[4] <= reg_in[26];
            out[5] <= reg_in[42];
            out[14] <= reg_in[1];
            out[13] <= reg_in[2];
            out[12] <= reg_in[3];
            out[11] <= reg_in[4];
            out[10] <= reg_in[5];
            out[9] <= reg_in[6];
            out[8] <= reg_in[7];
            out[22] <= reg_in[9];
            out[21] <= reg_in[10];
            out[20] <= reg_in[11];
            out[19] <= reg_in[12];
            out[18] <= reg_in[13];
            out[17] <= reg_in[14];
            out[16] <= reg_in[15];
            out[29] <= reg_in[18];
            out[28] <= reg_in[19];
            out[27] <= reg_in[20];
            out[26] <= reg_in[21];
            out[25] <= reg_in[22];
            out[24] <= reg_in[23];
            out[34] <= reg_in[29];
            out[33] <= reg_in[30];
            out[32] <= reg_in[31];
            out[45] <= reg_in[34];
            out[44] <= reg_in[35];
            out[43] <= reg_in[36];
            out[42] <= reg_in[37];
            out[41] <= reg_in[38];
            out[40] <= reg_in[39];
            out[52] <= reg_in[43];
            out[51] <= reg_in[44];
            out[50] <= reg_in[45];
            out[49] <= reg_in[46];
            out[48] <= reg_in[47];
            out[63] <= reg_in[48];
            out[62] <= reg_in[49];
            out[61] <= reg_in[50];
            out[60] <= reg_in[51];
            out[59] <= reg_in[52];
            out[58] <= reg_in[53];
            out[57] <= reg_in[54];
            out[56] <= reg_in[55];
            cnt <= cnt + 1;
        end
        15000: begin
            RTC_SCL <= 1;
            cnt <= 49751;
        end

        // WRITE all time/date registers
        // start bit
        20125: begin
            RTC_SDA_OUT <= 0;
            cnt <= cnt + 1;
        end
        20250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
            bcnt <= 7;
        end
        // send ADDR + WR
        20375: begin
            RTC_SDA_OUT <= CMD_WR[bcnt];
            cnt <= cnt + 1;
        end
        20500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        20750: begin
            RTC_SCL <= 0;
            if (bcnt == 0) begin
                cnt <= cnt + 1;
                RTC_SDA_OUT <= 1;
            end else begin
                cnt <= 20251;
                bcnt <= bcnt - 1;
            end
        end
        // check ACK
        21000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        21125: begin
            if (RTC_SDA_IN == 0) begin
                bcnt <= 7;
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        // send reg 0 ADDR
        21250: begin
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        21375: begin
            RTC_SDA_OUT <= START_ADDR[bcnt];
            cnt <= cnt + 1;
        end
        21500: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        21625: begin
            if (bcnt > 0) begin
                bcnt <= bcnt - 1;
                cnt <= 21126;
            end else begin
                cnt <= cnt + 1;
            end
        end
        // check ACK
        21750: begin
            RTC_SDA_OUT <= 1;
            RTC_SCL <= 0;
            cnt <= cnt + 1;
        end
        22000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        22125: begin
            if (RTC_SDA_IN == 0) begin
                bcnt <= 0;
                reg_cnt <= 0;
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        // write
        22250: begin
            RTC_SCL <= 0;
            cnt <= 23751;
        end
        23825: begin
            RTC_SDA_OUT <= reg_out[reg_cnt];
            reg_cnt <= reg_cnt + 1;
            cnt <= cnt + 1;
        end
        24000: begin
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        24250: begin
            RTC_SCL <= 0;
            bcnt <= bcnt + 1;
            if (bcnt == 7)
                cnt <= cnt + 1;
            else
                cnt <= 23751;
        end
        // check ACK
        24251: begin
            RTC_SDA_OUT <= 1;
            cnt <= cnt + 1;
        end
        24500: begin
            bcnt <= 0;
            RTC_SCL <= 1;
            cnt <= cnt + 1;
        end
        24625: begin
            if (RTC_SDA_IN == 0) begin
                cnt <= cnt + 1;
            end else begin
                cnt <= 50000;
                out[7] <= 1;
            end
        end
        24750: begin
            RTC_SCL <= 0;
            if (reg_cnt != 56) begin
                cnt <= 23751;
            end else
                cnt <= cnt + 1;
        end
        25000: begin
            RTC_SCL <= 1;
            cnt <= 49751;
        end

        // STOP bit
        50000: begin
            RTC_SDA_OUT <= 1;
            cnt <= 0;
            out[3] <= 0;
            out[2] <= 0;
            out[1] <= 0;
        end
        default: cnt <= cnt + 1;
    endcase
end

endmodule
