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

module USBC(
    input               CLK200,
    input               CLK111,
    input               USB_RST,
(* KEEP="TRUE" *)output reg rst,
    output reg [15:0]   USB_RCVF_IN,
    output reg          USB_RCVF_WE,
    input               USB_RCVF_FULL,
    input [15:0]        USB_SNDF_OUT,
    output reg          USB_SNDF_RE,
    input               USB_SNDF_EMPTY,
    input               USB_DPp,
    input               USB_DNp,
    output reg          USB_DRVp,
    output reg          USB_OUT_DPp,
    output reg          USB_OUT_DNp,
    output reg          UFI0_RESET,
    output reg [8:0]    UFI0_DIN,
    input [8:0]         UFI0_DOUT,
    output reg          UFI0_WE,
    output reg          UFI0_RE,
    input               UFI0_FULL,
    input               UFI0_EMPTY,
    output reg          UFI1_RESET,
    output reg [8:0]    UFI1_DIN,
    input [8:0]         UFI1_DOUT,
    output reg          UFI1_WE,
    output reg          UFI1_RE,
    input               UFI1_FULL,
    input               UFI1_EMPTY,
    output reg [3:0]    USBRAM_WE,
    output reg [10:0]   USBRAM_ADDR,
    output reg [31:0]   USBRAM_IN,
    input [31:0]        USBRAM_OUT,
    output reg [11:0]   USBROM_ADDR,
    input [31:0]        USBROM_OUT
);

(* KEEP="TRUE" *)reg [1:0]   rst_sync;

always @(posedge CLK200)
begin
    rst_sync[1] <= USB_RST;
end

reg USB_DP;
reg USB_DN;
reg USB_DRV;
reg USB_OUT_DP;
reg USB_OUT_DN;

always @(posedge CLK111)
begin
    USB_DP <= USB_DPp;
    USB_DN <= USB_DNp;
    USB_DRVp <= !USB_DRV;
    USB_OUT_DPp <= USB_OUT_DP;
    USB_OUT_DNp <= USB_OUT_DN;
    rst_sync[0] <= rst_sync[1];
    rst <= rst_sync[0];
end

reg         sie_reset;

///////////////////////////////////////////////////////////////////
//
// IO controller core (custom pico-RV32I)
//
typedef enum {
    ST_INIT, ST_FETCH, ST_FETCH1, ST_FETCH2, ST_FETCH3, ST_DECODE, ST_EXEC,
    ST_READ, ST_READ1, ST_READ2, ST_READ3, ST_READ4, ST_USBREAD0, ST_USBREAD1, ST_ROMREAD0, ST_ROMREAD1, ST_CPUREAD0, ST_CPUREAD1,
    ST_WRITE
} state_t;
state_t state;

reg [7:0]       reset;
reg [31:0]      r[32];
reg [31:0]      pc;
reg [31:0]      new_pc;
reg [31:0]      insn;
reg [31:0]      rs1;
reg [31:0]      rs2;
reg [4:0]       rd;
reg [31:0]      imm20;
reg [31:0]      eimm12;
reg [31:0]      simm12;
reg [31:0]      shamtd;
reg [31:0]      calc4;
reg [31:0]      calc5;
reg [31:0]      mem_addr;
reg [1:0]       mem_size;
reg             mem_u;
reg             mem_t;
reg [31:0]      mem_dat;
reg [31:0]      mem_wr;
reg [31:0]      res;
reg [31:0]      mem_res;
reg [1:0]       res_ctl;
reg [7:0]       dump_cnt;
reg [31:0]      dump_val;
reg [3:0]       dump_delay;
reg [31:0]      cycle_cnt;

always @(posedge CLK111)
begin
    cycle_cnt <= cycle_cnt + 1;
    if (!rst) begin
        UFI0_RESET <= 1;
        UFI0_WE <= 0;
        UFI1_RE <= 0;
        USB_RCVF_WE <= 0;
        USB_SNDF_RE <= 0;
        state <= ST_INIT;
        sie_reset <= 1;
        reset <= 0;
    end else case (state)
        ST_INIT: begin
            if (reset == 0) begin
                sie_reset <= 0;
                new_pc <= 0;
                r[0] <= 0;
                r[1] <= 0;
                r[2] <= 'h6000;
                rd <= 0;
                cycle_cnt <= 0;
            end else if (reset == 77)
                UFI0_RESET <= 0;
            if (reset < 128)
                reset <= reset + 1;
            else
                state <= ST_FETCH;
        end
        ST_FETCH: begin
            if (rd != 0) begin
                if (res_ctl == 1)
                    r[rd] <= res;
                else if (res_ctl == 2)
                    r[rd] <= mem_res;
            end
            state <= ST_FETCH1;
            USBRAM_WE <= 0;
            UFI0_WE <= 0;
            USB_RCVF_WE <= 0;
            if (new_pc < 'h4000)
                USBROM_ADDR <= new_pc[13:2];
        end
        ST_FETCH1: begin
            pc <= new_pc;
            state <= ST_FETCH2;
        end
        ST_FETCH2: begin
            state <= ST_FETCH3;
        end
        ST_FETCH3: begin
            insn <= USBROM_OUT;
            state <= ST_DECODE;
        end
        ST_DECODE: begin
            rs1 <= r[insn[19:15]];
            rs2 <= r[insn[24:20]];
            rd <= insn[11:7];
            imm20 <= insn[31:12];
            eimm12 <= { {20{insn[31]}}, insn[31:20] };
            simm12 <= { {20{insn[31]}}, insn[31:25], insn[11:7] };
            shamtd <= insn[25:20];
            calc4 <= { {11{insn[31]}}, insn[31], insn[19:12], insn[20], insn[30:21], 1'b0 };
            calc5 <= { {19{insn[31]}}, insn[31], insn[7], insn[30:25], insn[11:8], 1'b0 };
            state <= ST_EXEC;
        end
        ST_EXEC: begin
            state <= ST_FETCH;
            new_pc <= pc + 4;
            res_ctl <= 1;
            casez (insn)
                'b?????????????????????????0110111: begin // OP_LUI
                    res <= {imm20[19:0], 12'b0};
                end
                'b?????????????????????????0010111: begin // OP_AUIPC
                    res <= pc + {imm20[19:0], 12'b0};
                end
                'b?????????????????????????1101111: begin // OP_JAL
                    res <= pc + 4;
                    new_pc <= pc + calc4;
                end
                'b?????????????????000?????1100111: begin // OP_JALR
                    res <= pc + 4;
                    new_pc <= {rs1[31:1] + eimm12[31:1], 1'b0};
                end
                'b?????????????????000?????1100011: begin // OP_BEQ
                    if (rs1 == rs2)
                        new_pc <= pc + calc5;
                    res_ctl <= 0;
                end
                'b?????????????????001?????1100011: begin // OP_BNE
                    if (rs1 != rs2)
                        new_pc <= pc + calc5;
                    res_ctl <= 0;
                end
                'b?????????????????100?????1100011: begin // OP_BLT
                    if ($signed(rs1) < $signed(rs2))
                        new_pc <= pc + calc5;
                    res_ctl <= 0;
                end
                'b?????????????????101?????1100011: begin // OP_BGE
                    if ($signed(rs1) >= $signed(rs2))
                        new_pc <= pc + calc5;
                    res_ctl <= 0;
                end
                'b?????????????????110?????1100011: begin // OP_BLTU
                    if (rs1 < rs2)
                        new_pc <= pc + calc5;
                    res_ctl <= 0;
                end
                'b?????????????????111?????1100011: begin // OP_BGEU
                    if (rs1 >= rs2)
                        new_pc <= pc + calc5;
                    res_ctl <= 0;
                end
                'b?????????????????000?????0000011: begin // OP_LB
                    state <= ST_READ;
                    mem_addr <= rs1 + eimm12;
                    mem_size <= 1;
                    mem_u <= 0;
                    res_ctl <= 2;
                end
                'b?????????????????001?????0000011: begin // OP_LH
                    state <= ST_READ;
                    mem_addr <= rs1 + eimm12;
                    mem_size <= 2;
                    mem_u <= 0;
                    res_ctl <= 2;
                end
                'b?????????????????010?????0000011: begin // OP_LW
                    state <= ST_READ;
                    mem_addr <= rs1 + eimm12;
                    mem_size <= 4;
                    mem_u <= 0;
                    res_ctl <= 2;
                end
                'b?????????????????100?????0000011: begin // OP_LBU
                    state <= ST_READ;
                    mem_addr <= rs1 + eimm12;
                    mem_size <= 1;
                    mem_u <= 1;
                    res_ctl <= 2;
                end
                'b?????????????????101?????0000011: begin // OP_LHU
                    state <= ST_READ;
                    mem_addr <= rs1 + eimm12;
                    mem_size <= 2;
                    mem_u <= 1;
                    res_ctl <= 2;
                end
                'b?????????????????000?????0100011: begin // OP_SB
                    state <= ST_WRITE;
                    mem_addr <= rs1 + simm12;
                    mem_size <= 1;
                    mem_u <= 0;
                    mem_wr <= rs2;
                    res_ctl <= 0;
                end
                'b?????????????????001?????0100011: begin // OP_SH
                    state <= ST_WRITE;
                    mem_addr <= rs1 + simm12;
                    mem_size <= 2;
                    mem_u <= 0;
                    mem_wr <= rs2;
                    res_ctl <= 0;
                end
                'b?????????????????010?????0100011: begin // OP_SW
                    state <= ST_WRITE;
                    mem_addr <= rs1 + simm12;
                    mem_size <= 4;
                    mem_u <= 0;
                    mem_wr <= rs2;
                    res_ctl <= 0;
                end
                'b?????????????????000?????0010011: begin // OP_ADDI
                    res <= rs1 + eimm12;
                end
                'b000000???????????001?????0010011: begin // OP_SLLI
                    res <= rs1 << shamtd[4:0];
                end
                'b?????????????????010?????0010011: begin // OP_SLTI
                    res <= $signed(rs1) < $signed(eimm12) ? 1 : 0;
                end
                'b?????????????????011?????0010011: begin // OP_SLTIU
                    res <= rs1 < eimm12 ? 1 : 0;
                end
                'b?????????????????100?????0010011: begin // OP_XORI
                    res <= rs1 ^ eimm12;
                end
                'b000000???????????101?????0010011: begin // OP_SRLI
                    res <= rs1 >> shamtd[4:0];
                end
                'b010000???????????101?????0010011: begin // OP_SRAI
                    res <= $signed(rs1) >>> shamtd[4:0];
                end
                'b?????????????????110?????0010011: begin // OP_ORI
                    res <= rs1 | eimm12;
                end
                'b?????????????????111?????0010011: begin // OP_ANDI
                    res <= rs1 & eimm12;
                end
                'b0000000??????????000?????0110011: begin // OP_ADD
                    res <= rs1 + rs2;
                end
                'b0100000??????????000?????0110011: begin // OP_SUB
                    res <= rs1 - rs2;
                end
                'b0000000??????????001?????0110011: begin // OP_SLL
                    res <= rs1 << rs2[4:0];
                end
                'b0000000??????????010?????0110011: begin // OP_SLT
                    res <= $signed(rs1) < $signed(rs2) ? 1 : 0;
                end
                'b0000000??????????011?????0110011: begin // OP_SLTU
                    res <= rs1 < rs2 ? 1 : 0;
                end
                'b0000000??????????100?????0110011: begin // OP_XOR
                    res <= rs1 ^ rs2;
                end
                'b0000000??????????101?????0110011: begin // OP_SRL
                    res <= rs1 >> rs2[4:0];
                end
                'b0100000??????????101?????0110011: begin // OP_SRA
                    res <= $signed(rs1) >>> rs2[4:0];
                end
                'b0000000??????????110?????0110011: begin // OP_OR
                    res <= rs1 | rs2;
                end
                'b0000000??????????111?????0110011: begin // OP_AND
                    res <= rs1 & rs2;
                end
            endcase
        end
        ST_WRITE: begin
            if (mem_addr < 'h4000) begin
                state <= ST_FETCH;
            end else if (mem_addr < 'h6000) begin
                USBRAM_ADDR <= mem_addr[12:2];
                state <= ST_FETCH;
                case (mem_size)
                    1: case (mem_addr[1:0])
                        0: begin
                            USBRAM_WE <= 'b0001;
                            USBRAM_IN <= mem_wr;
                        end
                        1: begin
                            USBRAM_WE <= 'b0010;
                            USBRAM_IN <= mem_wr << 8;
                        end
                        2: begin
                            USBRAM_WE <= 'b0100;
                            USBRAM_IN <= mem_wr << 16;
                        end
                        3: begin
                            USBRAM_WE <= 'b1000;
                            USBRAM_IN <= mem_wr << 24;
                        end
                    endcase
                    2: case (mem_addr[1])
                        0: begin
                            USBRAM_WE <= 'b0011;
                            USBRAM_IN <= mem_wr;
                        end
                        1: begin
                            USBRAM_WE <= 'b1100;
                            USBRAM_IN <= mem_wr << 16;
                        end
                    endcase
                    default: begin
                        USBRAM_WE <= 'b1111;
                        USBRAM_IN <= mem_wr;
                    end
                endcase
            end else if (mem_addr == 'h8000) begin
                if (!UFI0_FULL) begin
                    UFI0_WE <= 1;
                    UFI0_DIN <= mem_wr;
                    state <= ST_FETCH;
                end
            end else if (mem_addr == 'h9000) begin
                if (!USB_RCVF_FULL) begin
                    USB_RCVF_WE <= 1;
                    USB_RCVF_IN <= mem_wr;
                    state <= ST_FETCH;
                end
            end else if (mem_addr == 'h10000) begin
                cycle_cnt <= mem_wr;
                state <= ST_FETCH;
            end else if (mem_addr == 'h11000) begin
                sie_reset <= mem_wr[0];
                UFI0_RESET <= mem_wr[0];
                state <= ST_FETCH;
            end else
                state <= ST_FETCH;
        end
        ST_READ: begin
            if (mem_addr < 'h4000) begin
                USBROM_ADDR <= mem_addr[13:2];
                state <= ST_READ1;
                mem_t <= 0;
            end else if (mem_addr < 'h6000) begin
                USBRAM_ADDR <= mem_addr[12:2];
                state <= ST_READ1;
                mem_t <= 1;
            end else if (mem_addr == 'h8000) begin
                if (!UFI1_EMPTY) begin
                    UFI1_RE <= 1;
                    state <= ST_USBREAD0;
                end
            end else if (mem_addr == 'h9000) begin
                if (!USB_SNDF_EMPTY) begin
                    USB_SNDF_RE <= 1;
                    state <= ST_CPUREAD0;
                end
            end else if (mem_addr == 'h10000) begin
                mem_res <= cycle_cnt;
                state <= ST_FETCH;
            end else
                state <= ST_FETCH;
        end
        ST_READ1: begin
            state <= ST_READ2;
        end
        ST_READ2: begin
            state <= ST_READ3;
        end
        ST_READ3: begin
            state <= ST_READ4;
            if (mem_t == 1)
                mem_dat <= USBRAM_OUT;
            else
                mem_dat <= USBROM_OUT;
        end
        ST_READ4: begin
            state <= ST_FETCH;
            case (mem_size)
                1: case(mem_addr[1:0])
                    0: mem_res <= mem_u ? {24'b0,mem_dat[7:0]} : {{24{mem_dat[7]}},mem_dat[7:0]};
                    1: mem_res <= mem_u ? {24'b0,mem_dat[15:8]} : {{24{mem_dat[15]}},mem_dat[15:8]};
                    2: mem_res <= mem_u ? {24'b0,mem_dat[23:16]} : {{24{mem_dat[23]}},mem_dat[23:16]};
                    3: mem_res <= mem_u ? {24'b0,mem_dat[31:24]} : {{24{mem_dat[31]}},mem_dat[31:24]};
                endcase
                2: case(mem_addr[1])
                    0: mem_res <= mem_u ? {16'b0,mem_dat[15:0]} : {{16{mem_dat[15]}},mem_dat[15:0]};
                    1: mem_res <= mem_u ? {16'b0,mem_dat[31:16]} : {{16{mem_dat[31]}},mem_dat[31:16]};
                endcase
                default: mem_res <= mem_dat[31:0];
            endcase
        end
        ST_USBREAD0: begin
            state <= ST_USBREAD1;
            UFI1_RE <= 0;
        end
        ST_USBREAD1: begin
            state <= ST_FETCH;
            mem_res <= {23'b0, UFI1_DOUT[8:0]};
        end
        ST_CPUREAD0: begin
            state <= ST_CPUREAD1;
            USB_SNDF_RE <= 0;
        end
        ST_CPUREAD1: begin
            state <= ST_FETCH;
            mem_res <= {16'b0, USB_SNDF_OUT[15:0]};
        end
    endcase
end

///////////////////////////////////////////////////////////////////
//
// USB SIE
//
typedef enum {
    SIE_INIT, SIE_IDLE,
    SIE_CMD0, SIE_CMD1, SIE_CMD2,
    SIE_WRITE, SIE_READ, SIE_READ1, SIE_EOP,
    SIE_RESET
} sie_state_t;
sie_state_t sie_st;

reg         sie_nop;
reg         sie_J;
reg [31:0]  sie_rst_cnt;
reg [7:0]   sie_rd_cnt;
reg [3:0]   sie_rd_bit;
reg         sie_rd_old;
reg [3:0]   sie_rd_n;
reg [1:0]   sie_rd_eop;
reg [15:0]  sie_eop_cnt;
reg [7:0]   sie_reset_cnt;
reg [7:0]   sie_wr_cnt;
reg [3:0]   sie_wr_bit;
reg [3:0]   sie_wr_n;
reg [8:0]   sie_wr_buf;
reg         sie_wr_stuff;
reg [31:0]  sie_rd_wait;
reg [2:0]   sie_cmd;

always @(posedge CLK111)
begin
    if (sie_reset) begin
        sie_st <= SIE_INIT;
        UFI1_RESET <= 1;
        sie_reset_cnt <= 0;
        UFI1_WE <= 0;
        UFI0_RE <= 0;
        USB_DRV <= 0;
    end else begin
        case (sie_st)
            SIE_INIT: begin
                if (sie_reset_cnt == 110)
                    sie_st <= SIE_IDLE;
                if (sie_reset_cnt == 77)
                    UFI1_RESET <= 0;
                if (sie_reset_cnt == 0) begin
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                    USB_DRV <= 0;
                    sie_J <= 0;
                    sie_nop <= 0;
                end
                sie_reset_cnt <= sie_reset_cnt + 1;
            end
            SIE_IDLE: begin
                UFI1_WE <= 0;
                if (sie_nop == 1)
                    sie_nop <= 0;
                else if (UFI0_EMPTY != 1) begin
                    sie_st <= SIE_CMD0;
                    UFI0_RE <= 1;
                end
            end
            SIE_CMD0: begin
                UFI0_RE <= 0;
                sie_st <= SIE_CMD1;
            end
            SIE_CMD1: begin
                sie_cmd <= UFI0_DOUT[2:0];
                sie_st <= SIE_CMD2;
            end
            SIE_CMD2: begin
                sie_cmd <= 0;
                case (sie_cmd)
                    0: begin    // NOP
                        sie_st <= SIE_IDLE;
                        sie_nop <= 1;
                    end
                    1: begin    // WRITE
                        sie_st <= SIE_WRITE;
                        sie_wr_cnt <= 69;
                        sie_wr_bit <= 7;
                        sie_wr_n <= 0;
                        sie_wr_stuff <= 0;
                        USB_OUT_DP <= sie_J;
                        USB_OUT_DN <= !sie_J;
                        //USB_OUT_DP <= 0;
                        //USB_OUT_DN <= 1;
                    end
                    2: begin    // READ
                        sie_st <= SIE_READ;
                        sie_rd_wait <= 111000000;
                    end
                    3: begin    // EOP
                        sie_st <= SIE_EOP;
                        sie_eop_cnt <= 0;
                    end
                    4: begin    // RESET
                        sie_st <= SIE_RESET;
                        sie_rst_cnt <= 0;
                        USB_DRV <= 1;
                        USB_OUT_DP <= 0;
                        USB_OUT_DN <= 0;
                    end
                    5: begin    // SET LOW-SPEED
                        sie_J <= 0;
                        sie_st <= SIE_IDLE;
                    end
                    6: begin    // SET FULL-SPEED
                        sie_J <= 1;
                        sie_st <= SIE_IDLE;
                    end
                    7: begin    // SENSE
                        UFI1_WE <= 1;
                        UFI1_DIN <= { 7'b0000000, USB_DN, USB_DP };
                        sie_st <= SIE_IDLE;
                    end
                endcase
            end
            SIE_WRITE: begin
                UFI1_WE <= 0;
                if (sie_wr_cnt == 73) begin
                    if (sie_wr_buf[8] == 1) begin
                        sie_eop_cnt <= 0;
                        sie_st <= SIE_EOP;
                    end
                    sie_wr_cnt <= 0;
                    if (!sie_wr_stuff) begin
                        if (sie_wr_bit == 7)
                            sie_wr_bit <= 0;
                        else
                            sie_wr_bit <= sie_wr_bit + 1;
                    end
                    sie_wr_stuff <= 0;
                end else if (sie_wr_cnt == 69) begin
                    if (sie_wr_stuff || (sie_wr_bit != 7) || ((sie_wr_bit == 7) && !UFI0_EMPTY))
                        sie_wr_cnt <= sie_wr_cnt + 1;
                end else begin
                    sie_wr_cnt <= sie_wr_cnt + 1;
                    if (sie_wr_cnt == 0) begin
                        USB_DRV <= 1;
                        if (sie_wr_buf[sie_wr_bit] == 0) begin
                            USB_OUT_DP <= !USB_OUT_DP;
                            USB_OUT_DN <= !USB_OUT_DN;
                            sie_wr_n <= 0;
                        end else begin
                            if (sie_wr_n == 6) begin
                                USB_OUT_DP <= !USB_OUT_DP;
                                USB_OUT_DN <= !USB_OUT_DN;
                                sie_wr_stuff <= 1;
                                sie_wr_n <= 0;
                            end else
                                sie_wr_n <= sie_wr_n + 1;
                        end
                    end else if ((sie_wr_bit == 7) && (sie_wr_cnt == 70) && (!sie_wr_stuff))
                        UFI0_RE <= 1;
                    else if ((sie_wr_bit == 7) && (sie_wr_cnt == 71) && (!sie_wr_stuff))
                        UFI0_RE <= 0;
                    else if ((sie_wr_bit == 7) && (sie_wr_cnt == 72) && (!sie_wr_stuff))
                        sie_wr_buf <= UFI0_DOUT;
                end
            end
            SIE_EOP: begin
                sie_eop_cnt <= sie_eop_cnt + 1;
                if (sie_eop_cnt == 0) begin
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_eop_cnt == 147) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
                    //USB_OUT_DP <= 0;
                    //USB_OUT_DN <= 1;
                end else if (sie_eop_cnt == 221)
                    USB_DRV <= 0;
                else if (sie_eop_cnt == 443)
                    sie_st <= SIE_IDLE;
            end
            SIE_READ: begin
                UFI1_WE <= 0;
                if (((USB_DP == !sie_J) && (USB_DN == sie_J)) || (sie_rd_wait == 0)) begin
                //if (((USB_DP == 1) && (USB_DN == 0)) || (sie_rd_wait == 0)) begin
                    sie_rd_n <= 0;
                    sie_rd_cnt <= 1;
                    sie_rd_bit <= 0;
                    sie_rd_old <= sie_J;
//                    sie_rd_old <= 0;
                    sie_rd_eop <= 0;
                    sie_st <= SIE_READ1;
                end
                sie_rd_wait <= sie_rd_wait - 1;
            end
            SIE_READ1: begin
                UFI1_WE <= 0;
                sie_rd_cnt <= sie_rd_cnt + 1;
                if (sie_rd_cnt == 57) begin
                    if ((USB_DP == 1) && (USB_DN == 1)) begin
                        UFI1_DIN <= 'b100000010;
                        UFI1_WE <= 1;
                        sie_st <= SIE_IDLE;
                    end else if ((USB_DP == 0) && (USB_DN == 0)) begin
                        sie_rd_eop <= sie_rd_eop + 1;
                        if (sie_rd_eop == 2) begin
                            UFI1_DIN <= 'b100000001;
                            UFI1_WE <= 1;
                            sie_st <= SIE_IDLE;
                        end
                    end else if ((USB_DP == sie_J) && (sie_rd_eop == 2)) begin
//                    end else if ((USB_DP == 0) && (sie_rd_eop == 2)) begin
                        UFI1_DIN <= 'b100000000;
                        UFI1_WE <= 1;
                        sie_st <= SIE_IDLE;
                    end else begin
                        sie_rd_eop <= 0;
                        UFI1_DIN[8] <= 0;
                        sie_rd_old <= USB_DP;
                        if (sie_rd_n != 6) begin
                            if (sie_rd_bit == 7) begin
                                UFI1_WE <= 1;
                                sie_rd_bit <= 0;
                            end else
                                sie_rd_bit <= sie_rd_bit + 1;
                        end
                        if (USB_DP == sie_rd_old) begin
                            if (sie_rd_n == 6) begin
                                UFI1_DIN <= 'b110000000;
                                UFI1_WE <= 1;
                                sie_st <= SIE_IDLE;
                            end else begin
                                UFI1_DIN[sie_rd_bit] <= 1;
                                sie_rd_n <= sie_rd_n + 1;
                            end
                        end else begin
                            UFI1_DIN[sie_rd_bit] <= 0;
                            sie_rd_n <= 0;
                        end
                    end
                end else if (sie_rd_cnt == 73)
                    sie_rd_cnt <= 0;
            end
            SIE_RESET: begin
                UFI1_WE <= 0;
                if (sie_rst_cnt == 2160000)                 // 20ms
                    USB_DRV <= 0;
                else if (sie_rst_cnt == 2268000) begin      // 21ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2268143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2268215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 2376000) begin  // 22ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2376143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2376215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 2484000) begin  // 23ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2484143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2484215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 2592000) begin  // 24ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2592143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2592215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 2700000) begin  // 25ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2700143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2700215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 2808000) begin  // 26ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2808143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2808215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 2916000) begin  // 27ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 2916143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 2916215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 3024000) begin  // 28ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 3024143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 3024215) begin
                    USB_DRV <= 0;
                end else if (sie_rst_cnt == 3132000) begin  // 29ms
                    USB_DRV <= 1;
                    USB_OUT_DP <= 0;
                    USB_OUT_DN <= 0;
                end else if (sie_rst_cnt == 3132143) begin
                    USB_OUT_DP <= sie_J;
                    USB_OUT_DN <= !sie_J;
//                    USB_OUT_DP <= 0;
//                    USB_OUT_DN <= 1;
                end else if (sie_rst_cnt == 3132215) begin
                    USB_DRV <= 0;
                end
                if (sie_rst_cnt <= 3240000)                 // 30ms
                    sie_rst_cnt <= sie_rst_cnt + 1;
                else
                    sie_st <= SIE_IDLE;
            end
        endcase
    end
end

endmodule
