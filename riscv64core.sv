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

module R64(
               input               CLK,
               input               xRST,
    (* KEEP *) output reg [1:0]    xACT,
    (* KEEP *) output reg [26:0]   xADDR,
    (* KEEP *) output reg [255:0]  xDOUT,
    (* KEEP *) output reg [31:0]   xMASK,
               input               xBUS,
               input [255:0]       xDIN,
               input               xINT,
               input [27:0]        xCC_ADDR
);
parameter COREn = 0;

reg [7:0] CORE_ID = COREn;

// BUS interface
           wire [1:0]      ACT;
           wire [26:0]     ADDR;
(* KEEP *) reg             RST;
(* KEEP *) reg [255:0]     DOUT;
(* KEEP *) reg [31:0]      MASK;
(* KEEP *) reg             BUS;
(* KEEP *) reg [255:0]     DIN;
(* KEEP *) reg             INT;
(* KEEP *) reg [27:0]      CC_ADDR;

always @(posedge CLK)
begin
    RST <= xRST;
    if (RST == 0)
    begin
        INT <= 0;
        BUS <= 0;
    end else begin
        xACT <= ACT;
        xADDR <= ADDR;
        xDOUT <= DOUT;
        xMASK <= MASK;
        BUS <= xBUS;
        DIN <= xDIN;
        INT <= xINT;
        CC_ADDR <= xCC_ADDR;
    end
end

`define IMM20   calc <= insn[31:12]
`define IMM12   calc <= insn[31:20]
`define SIMM12  calc <= { insn[31:25], insn[11:7] }
`define SHAMTD  calc <= insn[25:20]
`define CALC1   calc <= {{52{insn[31]}}, insn[31:20]}
`define CALC2   calc <= {{20{insn[31]}}, insn[31:20]}
`define CALC3   calc <= insn[24:20]
`define CALC4   calc <= { {43{insn[31]}}, insn[31], insn[19:12], insn[20], insn[30:21], 1'b0 }
`define CALC5   calc <= { {51{insn[31]}}, insn[31], insn[7], insn[30:25], insn[11:8], 1'b0 }

typedef enum {
    SQ_INIT, SQ_FETCH, SQ_DECODE, SQ_EXEC1, SQ_EXEC2, SQ_MEM
} seq_state_t;

typedef enum {
    OP_PAGEFAULT, OP_PROTFAULT, OP_ILLINSTR, OP_INTR,
    OP_LUI, OP_AUIPC, OP_JAL, OP_JALR,
    OP_BEQ, OP_BNE, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU,
    OP_LB, OP_LH, OP_LW, OP_LD, OP_LBU, OP_LHU, OP_LWU,
    OP_SB, OP_SH, OP_SW, OP_SD,
    OP_ADDI, OP_SLTI, OP_SLTIU, OP_XORI, OP_ORI, OP_ANDI, OP_SLLI, OP_SRLI, OP_SRAI,
    OP_ADD, OP_SUB, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_SRA, OP_OR, OP_AND,
    OP_ADDIW, OP_SLLIW, OP_SRLIW, OP_SRAIW,
    OP_ADDW, OP_SUBW, OP_SLLW, OP_SRLW, OP_SRAW,
    OP_FENCE, OP_FENCE_I,
    OP_ECALL, OP_SRET, OP_WFI, //OP_EBREAK,
    OP_CSRRW, OP_CSRRS, OP_CSRRC, OP_CSRRWI, OP_CSRRSI, OP_CSRRCI,
    OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU, OP_DIV, OP_DIVU, OP_REM, OP_REMU,
    OP_MULW, OP_DIVW, OP_DIVUW, OP_REMW, OP_REMUW,
    OP_INVASID, OP_INVADDR,
    OP_SWAP16, OP_SWAP32, OP_SWAP64, OP_ILSB, OP_TLSB, OP_ILCB, OP_TLCB, OP_BCNT,
    OP_BSET, OP_BCLR,
    OP_RC_WR, OP_RC_RD, OP_CHACHA, OP_CHACHA_INIT, OP_CHACHA_DONE, OP_CUBEH, OP_POLY1305MUL, OP_POLY1305INIT, OP_POLY1305DONE
} op_t;

seq_state_t seq_state;
op_t        op;
op_t        op2;
op_t        op3;

reg         sret;
reg         wfi;
reg [11:0]  fault;
reg         priv_lvl_dc;
reg         priv_lvl_fe;
reg         priv_lvl_ex1;
reg [63:0]  pc;
reg [63:0]  r[32];
reg [63:0]  rs1;
reg [63:0]  rs2;
reg [31:0]  rcs1;
reg [31:0]  rcs2;
reg [63:0]  rs1_2;
reg [63:0]  rs2_2;
reg [31:0]  rcs1_2;
reg [31:0]  rcs2_2;
reg [63:0]  rs1_3;
reg [63:0]  rs2_3;
reg [31:0]  rcs1_3;
reg [31:0]  rcs2_3;
reg [4:0]   rd;
reg [4:0]   rd2;
reg [4:0]   rs1i;
reg [4:0]   rs1i_2;
reg [4:0]   rs1i_3;
reg [31:0]  insn;
reg [31:0]  insn3;
reg [63:0]  calc;
reg [63:0]  calc2;
reg [63:0]  calc3;
reg [63:0]  res1;
reg [63:0]  res2;
reg [63:0]  ex_pc;
reg [63:0]  ex_pc2;
reg [63:0]  prev_pc;
reg [63:0]  new_pc;
reg [8:0]   inv_asid;
reg [63:0]  inv_addr;
reg [7:0]   time_inc;
reg [63:0]  cr[32];
reg [31:0]  rc[32];
reg [63:0]  t0;
reg [63:0]  t1;
reg [63:0]  t2;
reg [63:0]  t3;
reg [63:0]  t4;
reg [31:0]  poly_last;
reg [31:0]  t00;
reg [31:0]  t01;
reg [31:0]  t02;
reg [31:0]  t03;
reg [31:0]  t04;
reg [31:0]  tt;
reg [32:0]  et0;
reg [32:0]  et1;
reg [32:0]  et2;
reg [32:0]  et3;
reg [32:0]  et00;
reg [32:0]  et10;
reg [32:0]  et20;
reg [32:0]  et30;

// CACHES
reg  [36:0]             IC0_WE;
reg  [7:0]              IC0_ADDR;
reg  [295:0]            IC0_IN;
wire [295:0]            IC0_OUT;
reg  [36:0]             IC0_CC_WE;
reg  [7:0]              IC0_CC_ADDR;
reg  [295:0]            IC0_CC_IN;
wire [295:0]            IC0_CC_OUT;
CACHERAM icache0(CLK, IC0_WE, IC0_ADDR, IC0_IN, IC0_OUT, CLK, IC0_CC_WE, IC0_CC_ADDR, IC0_CC_IN, IC0_CC_OUT);
reg  [36:0]             IC1_WE;
reg  [7:0]              IC1_ADDR;
reg  [295:0]            IC1_IN;
wire [295:0]            IC1_OUT;
reg  [36:0]             IC1_CC_WE;
reg  [7:0]              IC1_CC_ADDR;
reg  [295:0]            IC1_CC_IN;
wire [295:0]            IC1_CC_OUT;
CACHERAM icache1(CLK, IC1_WE, IC1_ADDR, IC1_IN, IC1_OUT, CLK, IC1_CC_WE, IC1_CC_ADDR, IC1_CC_IN, IC1_CC_OUT);
reg  [36:0]             DC0_WE;
reg  [7:0]              DC0_ADDR;
reg  [295:0]            DC0_IN;
wire [295:0]            DC0_OUT;
reg  [36:0]             DC0_CC_WE;
reg  [7:0]              DC0_CC_ADDR;
reg  [295:0]            DC0_CC_IN;
wire [295:0]            DC0_CC_OUT;
CACHERAM dcache0(CLK, DC0_WE, DC0_ADDR, DC0_IN, DC0_OUT, CLK, DC0_CC_WE, DC0_CC_ADDR, DC0_CC_IN, DC0_CC_OUT);
reg  [36:0]             DC1_WE;
reg  [7:0]              DC1_ADDR;
reg  [295:0]            DC1_IN;
wire [295:0]            DC1_OUT;
reg  [36:0]             DC1_CC_WE;
reg  [7:0]              DC1_CC_ADDR;
reg  [295:0]            DC1_CC_IN;
wire [295:0]            DC1_CC_OUT;
CACHERAM dcache1(CLK, DC1_WE, DC1_ADDR, DC1_IN, DC1_OUT, CLK, DC1_CC_WE, DC1_CC_ADDR, DC1_CC_IN, DC1_CC_OUT);

assign IC0_CC_ADDR = CC_ADDR[12:5];
assign IC1_CC_ADDR = CC_ADDR[12:5];
assign DC0_CC_ADDR = CC_ADDR[12:5];
assign DC1_CC_ADDR = CC_ADDR[12:5];

typedef enum {
    ST_CC_IDLE, ST_CC_W0, ST_CC_W1
} st_cc_t;
st_cc_t st_cc;

// cache coherency mechanism
always @(posedge CLK)
begin
    if (RST == 0) begin
        st_cc <= ST_CC_IDLE;
        IC0_CC_WE <= 0;
        IC1_CC_WE <= 0;
        DC0_CC_WE <= 0;
        DC1_CC_WE <= 0;
    end else case (st_cc)
        ST_CC_IDLE: begin
            IC0_CC_WE <= 0;
            IC1_CC_WE <= 0;
            DC0_CC_WE <= 0;
            DC1_CC_WE <= 0;
            if (CC_ADDR[27] == 1)
                st_cc <= ST_CC_W0;
        end
        ST_CC_W0: begin
            st_cc <= ST_CC_W1;
        end
        ST_CC_W1: begin
            st_cc <= ST_CC_IDLE;
            if ((IC0_CC_OUT[283] == 1) && (CC_ADDR[26:0] == IC0_CC_OUT[282:256]))
            begin
                IC0_CC_WE <= 'h1fffffffff;
                IC0_CC_IN <= 0;
            end
            if ((IC1_CC_OUT[283] == 1) && (CC_ADDR[26:0] == IC1_CC_OUT[282:256]))
            begin
                IC1_CC_WE <= 'h1fffffffff;
                IC1_CC_IN <= 0;
            end
            if ((DC0_CC_OUT[283] == 1) && (CC_ADDR[26:0] == DC0_CC_OUT[282:256]))
            begin
                DC0_CC_WE <= 'h1fffffffff;
                DC0_CC_IN <= 0;
            end
            if ((DC1_CC_OUT[283] == 1) && (CC_ADDR[26:0] == DC1_CC_OUT[282:256]))
            begin
                DC1_CC_WE <= 'h1fffffffff;
                DC1_CC_IN <= 0;
            end
        end
    endcase
end

// user TLB
reg  [10:0]             IT0_WE;
reg  [7:0]              IT0_ADDR;
reg  [87:0]             IT0_IN;
wire [87:0]             IT0_OUT;
reg  [10:0]             IT1_WE;
reg  [7:0]              IT1_ADDR;
reg  [87:0]             IT1_IN;
wire [87:0]             IT1_OUT;
TLB itlb0(CLK, IT0_WE, {1'b0, IT0_ADDR}, IT0_IN, IT0_OUT, CLK, IT1_WE, {1'b1, IT1_ADDR}, IT1_IN, IT1_OUT);
reg  [10:0]             IT2_WE;
reg  [7:0]              IT2_ADDR;
reg  [87:0]             IT2_IN;
wire [87:0]             IT2_OUT;
reg  [10:0]             IT3_WE;
reg  [7:0]              IT3_ADDR;
reg  [87:0]             IT3_IN;
wire [87:0]             IT3_OUT;
TLB itlb1(CLK, IT2_WE, {1'b0, IT2_ADDR}, IT2_IN, IT2_OUT, CLK, IT3_WE, {1'b1, IT3_ADDR}, IT3_IN, IT3_OUT);
reg  [10:0]             DT0_WE;
reg  [7:0]              DT0_ADDR;
reg  [87:0]             DT0_IN;
wire [87:0]             DT0_OUT;
reg  [10:0]             DT1_WE;
reg  [7:0]              DT1_ADDR;
reg  [87:0]             DT1_IN;
wire [87:0]             DT1_OUT;
TLB dtlb0(CLK, DT0_WE, {1'b0, DT0_ADDR}, DT0_IN, DT0_OUT, CLK, DT1_WE, {1'b1, DT1_ADDR}, DT1_IN, DT1_OUT);
reg  [10:0]             DT2_WE;
reg  [7:0]              DT2_ADDR;
reg  [87:0]             DT2_IN;
wire [87:0]             DT2_OUT;
reg  [10:0]             DT3_WE;
reg  [7:0]              DT3_ADDR;
reg  [87:0]             DT3_IN;
wire [87:0]             DT3_OUT;
TLB dtlb1(CLK, DT2_WE, {1'b0, DT2_ADDR}, DT2_IN, DT2_OUT, CLK, DT3_WE, {1'b1, DT3_ADDR}, DT3_IN, DT3_OUT);

// kernel TLB
reg  [10:0]             IT4_WE;
reg  [7:0]              IT4_ADDR;
reg  [87:0]             IT4_IN;
wire [87:0]             IT4_OUT;
reg  [10:0]             IT5_WE;
reg  [7:0]              IT5_ADDR;
reg  [87:0]             IT5_IN;
wire [87:0]             IT5_OUT;
TLB itlb2(CLK, IT4_WE, {1'b0, IT4_ADDR}, IT4_IN, IT4_OUT, CLK, IT5_WE, {1'b1, IT5_ADDR}, IT5_IN, IT5_OUT);
reg  [10:0]             DT4_WE;
reg  [7:0]              DT4_ADDR;
reg  [87:0]             DT4_IN;
wire [87:0]             DT4_OUT;
reg  [10:0]             DT5_WE;
reg  [7:0]              DT5_ADDR;
reg  [87:0]             DT5_IN;
wire [87:0]             DT5_OUT;
TLB dtlb2(CLK, DT4_WE, {1'b0, DT4_ADDR}, DT4_IN, DT4_OUT, CLK, DT5_WE, {1'b1, DT5_ADDR}, DT5_IN, DT5_OUT);

// ICACHE
reg [1:0]               IC_MATCH;
wire [255:0]            IC_DAT;
assign IC_DAT = (IC_MATCH == 0) ? IC0_OUT :
                (IC_MATCH == 1) ? IC1_OUT :
                0;

// DCACHE
reg [1:0]               DC_MATCH;
wire [255:0]            DC_DAT;
assign DC_DAT = (DC_MATCH == 0) ? DC0_OUT :
                (DC_MATCH == 1) ? DC1_OUT :
                0;

reg                     do_int;
reg [3:0]               irq;
reg [3:0]               irq2;
reg [3:0]               irq3;
reg [3:0]               irq4;
reg [31:0]              iaddr;
reg [31:0]              daddr;
reg [63:0]              DATA_ADDR;
reg [63:0]              DATA_DIN;
reg [63:0]              DATA_DOUT;
reg [2:0]               DATA_ACT;
reg [1:0]               DATA_ST;
reg [8:0]               iCACHE_INIT_ADDR;
reg                     iCACHE_INIT;
reg [8:0]               dCACHE_INIT_ADDR;
reg                     dCACHE_INIT;
reg [8:0]               dasid_flush_cnt;
reg [8:0]               daddr_flush_cnt;
reg [8:0]               iasid_flush_cnt;
reg [8:0]               iaddr_flush_cnt;
assign IC0_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[12:5];
assign IC1_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[12:5];
assign DC0_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[12:5];
assign DC1_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[12:5];
assign IT0_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[19:13];
assign IT1_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[19:13];
assign IT2_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[19:13];
assign IT3_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[19:13];
assign IT4_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[19:13];
assign IT5_ADDR = (iCACHE_INIT == 1) ? iCACHE_INIT_ADDR : pc[19:13];
assign DT0_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[19:13];
assign DT1_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[19:13];
assign DT2_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[19:13];
assign DT3_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[19:13];
assign DT4_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[19:13];
assign DT5_ADDR = (dCACHE_INIT == 1) ? dCACHE_INIT_ADDR : DATA_ADDR[19:13];

// ARITH FUNCS
reg                     DIVS_EN;
wire                    DIVS_DS_VALIDx;
wire                    DIVS_DS_READY;
reg  [63:0]             DIVS_DS;
wire                    DIVS_DD_VALIDx;
wire                    DIVS_DD_READY;
reg  [63:0]             DIVS_DD;
wire                    DIVS_OUT_VALID;
wire [127:0]            DIVS_OUT_DATA;
DIVS divs(CLK, DIVS_EN, DIVS_DS_VALIDx, DIVS_DS_READY, DIVS_DS, DIVS_DD_VALIDx, DIVS_DD_READY, DIVS_DD, DIVS_OUT_VALID, DIVS_OUT_DATA);
reg                     DIV_STARTx;
reg                     DIV_START;
reg [63:0]              DIV_A;
reg [63:0]              DIV_B;
wire [63:0]             DIV_Q;
wire [63:0]             DIV_R;
reg [6:0]               DIV_CNT;
DIV64_64 div(CLK, DIV_STARTx, DIV_A, DIV_B, DIV_Q, DIV_R);
reg [63:0]              MULSS_A;
reg [63:0]              MULSS_B;
reg                     MULSS_EN;
wire [127:0]            MULSS_P;
MULSS mulss(CLK, MULSS_A, MULSS_B, MULSS_EN, MULSS_P);
reg [63:0]              MULSU_A;
reg [63:0]              MULSU_B;
reg                     MULSU_EN;
wire [127:0]            MULSU_P;
MULSU mulsu(CLK, MULSU_A, MULSU_B, MULSU_EN, MULSU_P);
reg [63:0]              MULUU_A;
reg [63:0]              MULUU_B;
reg                     MULUU_EN;
wire [127:0]            MULUU_P;
MULUU muluu(CLK, MULUU_A, MULUU_B, MULUU_EN, MULUU_P);

// CRYPTO MULTIPLIERS
reg [31:0]      mul0A;
reg [31:0]      mul0B;
reg             mul0CE;
wire [63:0]     mul0P;
MUL32x32_64 mul0(CLK, mul0A, mul0B, mul0CE, mul0P);
reg [31:0]      mul1A;
reg [31:0]      mul1B;
reg             mul1CE;
wire [63:0]     mul1P;
MUL32x32_64 mul1(CLK, mul1A, mul1B, mul1CE, mul1P);
reg [31:0]      mul2A;
reg [31:0]      mul2B;
reg             mul2CE;
wire [63:0]     mul2P;
MUL32x32_64 mul2(CLK, mul2A, mul2B, mul2CE, mul2P);
reg [31:0]      mul3A;
reg [31:0]      mul3B;
reg             mul3CE;
wire [63:0]     mul3P;
MUL32x32_64 mul3(CLK, mul3A, mul3B, mul3CE, mul3P);
reg [31:0]      mul4A;
reg [31:0]      mul4B;
reg             mul4CE;
wire [63:0]     mul4P;
MUL32x32_64 mul4(CLK, mul4A, mul4B, mul4CE, mul4P);

// SEQUENCER
reg             FE_EN;
reg             DE_EN;
reg             EX1_EN;
reg             EX2_EN;
reg             ME_EN;
reg [2:0]       ME_DO;
reg             FAULT;
reg             FAULT2;
reg             FAULT_DONE;
reg [1:0]       act_fe;
reg [1:0]       act_dc;
reg [26:0]      addr_fe;
reg [26:0]      addr_dc;
reg [63:0]      stack_addr;
reg [63:0]      dst1;
reg [2:0]       exc_cnt;
reg [2:0]       sret_cnt;
reg [63:0]      insn_cnt;
reg             i_pfault;
reg             DIVS_DS_VALID;
reg             DIVS_DD_VALID;
reg             DIVU_DS_VALID;
reg             DIVU_DD_VALID;
reg [4:0]       MUL_CNT;
reg [5:0]       CHACHA_CNT;
reg             CHACHA_EN;
reg [3:0]       CUBEH_CNT;
reg             CUBEH_EN;
reg [4:0]       POLY1305MUL_CNT;
reg             POLY1305MUL_EN;
reg [2:0]       POLY1305INIT_CNT;
reg             POLY1305INIT_EN;
reg [4:0]       POLY1305DONE_CNT;
reg             POLY1305DONE_EN;

assign ACT = (seq_state == SQ_MEM) ? act_dc : act_fe;
assign ADDR = (seq_state == SQ_MEM) ? addr_dc : addr_fe;
assign DIVS_DS_VALIDx = (seq_state == SQ_EXEC2) ? DIVS_DS_VALID : 0;
assign DIVS_DD_VALIDx = (seq_state == SQ_EXEC2) ? DIVS_DD_VALID : 0;

wire [8:0]  asid;
assign asid = cr[4][8:0];
wire [63:0] pt0_off;
assign pt0_off = { 1'b0, cr[4][62:35], 35'b0 };
wire [29:0] pt0;
assign pt0 = { cr[4][29:13], 13'b0 };
wire [29:0] pt1;
assign pt1 = { cr[5][29:13], 13'b0 };

typedef enum {
    FE_INIT, FE_START, FE_CACHE0, FE_CACHE1, FE_CACHE2, FE_WFI, FE_INT0, FE_PW0, FE_PW1, FE_PW2, FE_MEM0, FE_END, FE_INVASID, FE_INVADDR
} fetch_state;
fetch_state fe_st; 
reg [5:0]   iacc;
wire [31:0] iPT0_ENT;
wire [31:0] iPT1_ENT;
reg [8:0]   fe_init_cnt;
reg         fe_inv_done;
assign iPT0_ENT = (pc[26:24] == 0) ? DIN[31:0] :
                 (pc[26:24] == 1) ? DIN[63:32] :
                 (pc[26:24] == 2) ? DIN[95:64] :
                 (pc[26:24] == 3) ? DIN[127:96] :
                 (pc[26:24] == 4) ? DIN[159:128] :
                 (pc[26:24] == 5) ? DIN[191:160] :
                 (pc[26:24] == 6) ? DIN[223:192] :
                 (pc[26:24] == 7) ? DIN[255:224] :
                 0;
assign iPT1_ENT = (pc[15:13] == 0) ? DIN[31:0] :
                 (pc[15:13] == 1) ? DIN[63:32] :
                 (pc[15:13] == 2) ? DIN[95:64] :
                 (pc[15:13] == 3) ? DIN[127:96] :
                 (pc[15:13] == 4) ? DIN[159:128] :
                 (pc[15:13] == 5) ? DIN[191:160] :
                 (pc[15:13] == 6) ? DIN[223:192] :
                 (pc[15:13] == 7) ? DIN[255:224] :
                 0;

reg [7:0]   DMASK;

always @(posedge CLK)
begin
    if (RST == 0) begin
        seq_state <= SQ_INIT;
        FE_EN <= 0;
        DE_EN <= 0;
        EX1_EN <= 0;
        EX2_EN <= 0;
        ME_EN <= 0;
    end else case (seq_state)
        SQ_INIT: begin
            seq_state <= SQ_FETCH;
            FE_EN <= 1;
            DE_EN <= 0;
            EX1_EN <= 0;
            EX2_EN <= 0;
            ME_EN <= 0;
            insn_cnt <= 0;
        end
        SQ_FETCH: begin
            if (fe_st == FE_END) begin
                seq_state <= SQ_DECODE;
                FE_EN <= 0;
                DE_EN <= 1;
            end
        end
        SQ_DECODE: begin
            seq_state <= SQ_EXEC1;
            DE_EN <= 0;
            EX1_EN <= 1;
        end
        SQ_EXEC1: begin
            seq_state <= SQ_EXEC2;
            EX1_EN <= 0;
            EX2_EN <= 1;
            MUL_CNT <= 20;
            DIV_CNT <= 68;
            DIV_STARTx <= 0;
        end
        SQ_EXEC2: begin
            if (((MULUU_EN == 1) || (MULSU_EN == 1) || (MULSS_EN == 1)) && (MUL_CNT > 0))
                MUL_CNT <= MUL_CNT - 1;
            else if ((DIV_START == 1) && (DIV_CNT > 0)) begin
                DIV_STARTx <= 1;
                DIV_CNT <= DIV_CNT - 1;
            end else if ((DIVS_EN == 1) && (DIVS_OUT_VALID == 0))
                seq_state <= SQ_EXEC2;
            else begin
                EX2_EN <= 0;
                ME_EN <= 1;
                seq_state <= SQ_MEM;
            end
        end
        SQ_MEM: begin
            if ((FAULT == 1) || (FAULT2 == 1) || (sret == 1)) begin
                if (FAULT_DONE == 1) begin
                    seq_state <= SQ_FETCH;
                    ME_EN <= 0;
                    FE_EN <= 1;
                    insn_cnt <= cr[29] + 1;
                end
            end else if ((CHACHA_EN == 1) && (CHACHA_CNT > 0))
                seq_state <= SQ_MEM;
            else if ((CUBEH_EN == 1) && (CUBEH_CNT > 0))
                seq_state <= SQ_MEM;
            else if ((POLY1305MUL_EN == 1) && (POLY1305MUL_CNT > 0))
                seq_state <= SQ_MEM;
            else if ((POLY1305INIT_EN == 1) && (POLY1305INIT_CNT > 0))
                seq_state <= SQ_MEM;
            else if ((POLY1305DONE_EN == 1) && (POLY1305DONE_CNT > 0))
                seq_state <= SQ_MEM;
            else if (((ME_DO != 0) && (DATA_ST == 1)) || ((ME_DO == 0))) begin
                seq_state <= SQ_FETCH;
                ME_EN <= 0;
                FE_EN <= 1;
                insn_cnt <= cr[29] + 1;
            end
        end
    endcase
end

// FETCH
always @(posedge CLK)
begin
    if (RST == 0) begin
        act_fe <= 0;
        addr_fe <= 0;
        ex_pc <= 0;
        i_pfault <= 0;
        fe_st <= FE_INIT;
        fe_init_cnt <= 0;
        fe_inv_done <= 0;
        do_int <= 0;
    end else if (FE_EN == 1) begin
        case (fe_st)
            FE_INIT: begin
                if (fe_init_cnt == 256) begin
                    fe_init_cnt <= 0;
                    IC0_WE <= 0;
                    IC1_WE <= 0;
                    IT0_WE <= 0;
                    IT1_WE <= 0;
                    IT2_WE <= 0;
                    IT3_WE <= 0;
                    IT4_WE <= 0;
                    IT5_WE <= 0;
                    iCACHE_INIT <= 0;
                    fe_st <= FE_START;
                end else begin
                    fe_init_cnt <= fe_init_cnt + 1;
                    iCACHE_INIT_ADDR <= fe_init_cnt;
                    if (fe_init_cnt == 0) begin
                        iCACHE_INIT <= 1;
                        IC0_IN <= 0;
                        IC0_WE <= 'h1fffffffff;
                        IC1_IN <= 0;
                        IC1_WE <= 'h1fffffffff;
                        IT0_IN <= 0;
                        IT0_WE <= 'h7ff;
                        IT1_IN <= 0;
                        IT1_WE <= 'h7ff;
                        IT2_IN <= 0;
                        IT2_WE <= 'h7ff;
                        IT3_IN <= 0;
                        IT3_WE <= 'h7ff;
                        IT4_IN <= 0;
                        IT4_WE <= 'h7ff;
                        IT5_IN <= 0;
                        IT5_WE <= 'h7ff;
                    end
                end
            end
            FE_START: begin
                do_int <= 0;
                if (wfi == 1) begin
                    fe_st <= FE_WFI;
                end else begin
                    fe_st <= FE_CACHE0;
                    fe_inv_done <= 1;
                    if (fe_inv_done == 0) begin
                        priv_lvl_ex1 <= priv_lvl_fe;
                        i_pfault <= 0;
                        prev_pc <= ex_pc;
                        ex_pc <= pc;
                        iacc <= (priv_lvl_fe == 1) ? 6'b001000 : 6'b000001;
                        if (inv_asid != 0) begin
                            fe_st <= FE_INVASID;
                            iCACHE_INIT <= 1;
                            iCACHE_INIT_ADDR <= 0;
                            iasid_flush_cnt <= 0;
                        end else if (inv_addr != 1) begin
                            fe_st <= FE_INVADDR;
                            iCACHE_INIT <= 1;
                            iCACHE_INIT_ADDR <= inv_addr[12:5];
                            iaddr_flush_cnt <= 0;
                        end else if ((INT == 1) && (cr[2][0] == 1)) begin
                            fe_st <= FE_INT0;
                            act_fe <= 3;
                            do_int <= 1;
                            ex_pc <= ex_pc;
                            prev_pc <= prev_pc;
                        end else if ((ex_pc[63:5] == pc[63:5]) && (IC_MATCH != 3)) begin
                            // if we're still in the same cache line, and the previous insn was in cache,
                            // just read from the same cache line
                            fe_st <= FE_END;
                            iaddr[4:0] <= pc[4:0];
                        end
                    end
                end
            end
            FE_WFI: begin
                if ((INT == 1) && (cr[2][0] == 1)) begin
                    fe_st <= FE_INT0;
                    act_fe <= 3;
                    do_int <= 1;
                end
            end
            FE_CACHE0: begin
                fe_st <= FE_CACHE1;
            end
            FE_CACHE1: begin
                fe_st <= FE_CACHE2;
                if ((pc >= 'ha000000000000000) && (pc < 'ha000000100000000)) begin
                    // direct map range
                    // only for kernel-mode when not disabled
                    if ((cr[2][1] == 0) && (priv_lvl_fe == 1)) begin
                        iaddr <= pc[31:0];
                    end else begin
                        fe_st <= FE_END;
                        i_pfault <= 1;
                    end
                end else if (pc < 'h8000000000000000) begin
                    // search user TLB
                    if ((IT0_OUT[76] == 1) && (pc[63:13] == IT0_OUT[69:19]) && (IT0_OUT[75:70] & iacc) && (asid == IT0_OUT[85:77])) begin
                        iaddr <= {IT0_OUT[18:0], pc[12:0]};
                    end else if ((IT1_OUT[76] == 1) && (pc[63:13] == IT1_OUT[69:19]) && (IT1_OUT[75:70] & iacc) && (asid == IT1_OUT[85:77])) begin
                        iaddr <= {IT1_OUT[18:0], pc[12:0]};
                    end else if ((IT2_OUT[76] == 1) && (pc[63:13] == IT2_OUT[69:19]) && (IT2_OUT[75:70] & iacc) && (asid == IT2_OUT[85:77])) begin
                        iaddr <= {IT2_OUT[18:0], pc[12:0]};
                    end else if ((IT3_OUT[76] == 1) && (pc[63:13] == IT3_OUT[69:19]) && (IT3_OUT[75:70] & iacc) && (asid == IT3_OUT[85:77])) begin
                        iaddr <= {IT3_OUT[18:0], pc[12:0]};
                    end else begin
                        fe_st <= FE_PW0;
                    end
                end else begin
                    if ((IT4_OUT[76] == 1) && (pc[63:13] == IT4_OUT[69:19]) && (IT4_OUT[75:70] & iacc)) begin
                        iaddr <= {IT4_OUT[18:0], pc[12:0]};
                    end else if ((IT5_OUT[76] == 1) && (pc[63:13] == IT5_OUT[69:19]) && (IT5_OUT[75:70] & iacc)) begin
                        iaddr <= {IT5_OUT[18:0], pc[12:0]};
                    end else begin
                        fe_st <= FE_PW0;
                    end
                end
            end
            FE_CACHE2: begin
                IT0_WE <= 0;
                IT1_WE <= 0;
                IT2_WE <= 0;
                IT3_WE <= 0;
                IT4_WE <= 0;
                IT5_WE <= 0;
                fe_st <= FE_END;
                if ((IC0_OUT[283] == 1) && (iaddr[31:5] == IC0_OUT[282:256]))
                    IC_MATCH <= 0;
                else if ((IC1_OUT[283] == 1) && (iaddr[31:5] == IC1_OUT[282:256]))
                    IC_MATCH <= 1;
                else begin
                    IC_MATCH <= 3;
                    addr_fe <= iaddr[31:5];
                    act_fe <= 1;
                    fe_st <= FE_MEM0;
                end
            end
            FE_INT0: begin
                act_fe <= 0;
                if (BUS == 1) begin
                    fe_st <= FE_END;
                    irq <= DIN[3:0];
                end
            end
            FE_PW0: begin
                fe_st <= FE_PW1;            
                if (pc >= 'hfffffff800000000) begin
                    addr_fe <= {2'b0, pt1[29:13], pc[34:27]};
                    act_fe <= 1;
                end else if ((pc >= {1'b0, pt0_off}) && (pc < ({1'b0, pt0_off} + 'h800000000))) begin
                    addr_fe <= {2'b0, pt0[29:13], pc[34:27]};
                    act_fe <= 1;
                end else begin
                    i_pfault <= 1;
                    fe_st <= FE_END;
                end
            end
            FE_PW1: begin
                act_fe <= 0;
                if (BUS == 1) begin
                    if (iPT0_ENT[12] == 1) begin
                        addr_fe <= {2'b0, iPT0_ENT[29:13], pc[23:16]};
                        act_fe <= 1;
                        fe_st <= FE_PW2;
                    end else begin
                        i_pfault <= 1;
                        fe_st <= FE_END;
                    end
                end
            end
            FE_PW2: begin
                act_fe <= 0;
                if (BUS == 1) begin
                    if ((iPT1_ENT[12] == 1) && (iPT1_ENT[5:0] & iacc)) begin
                        iaddr <= {iPT1_ENT[31:13], pc[12:0]};
                        fe_st <= FE_CACHE2;
                        if (pc < 'h8000000000000000) begin
                            case (IT0_OUT[87:86])
                                0: begin IT0_WE <= 'h7ff; IT0_IN <= {2'b01, asid, 1'b1, iPT1_ENT[5:0], pc[63:13], iPT1_ENT[31:13]}; end
                                1: begin IT1_WE <= 'h7ff; IT1_IN <= {2'b00, asid, 1'b1, iPT1_ENT[5:0], pc[63:13], iPT1_ENT[31:13]};
                                         IT0_WE <= 'h400; IT0_IN[87:80] <= {2'b10, IT0_OUT[85:80]};
                                end
                                2: begin IT2_WE <= 'h7ff; IT2_IN <= {2'b00, asid, 1'b1, iPT1_ENT[5:0], pc[63:13], iPT1_ENT[31:13]};
                                         IT0_WE <= 'h400; IT0_IN[87:80] <= {2'b11, IT0_OUT[85:80]};
                                end
                                3: begin IT3_WE <= 'h7ff; IT3_IN <= {2'b00, asid, 1'b1, iPT1_ENT[5:0], pc[63:13], iPT1_ENT[31:13]};
                                         IT0_WE <= 'h400; IT0_IN[87:80] <= {2'b00, IT0_OUT[85:80]};
                                end
                            endcase
                        end else begin
                            case (IT4_OUT[87:86])
                                0: begin IT4_WE <= 'h7ff; IT4_IN <= {2'b01, 9'b00, 1'b1, iPT1_ENT[5:0], pc[63:13], iPT1_ENT[31:13]}; end
                                default: begin IT5_WE <= 'h7ff; IT5_IN <= {2'b00, 9'b00, 1'b1, iPT1_ENT[5:0], pc[63:13], iPT1_ENT[31:13]};
                                         IT4_WE <= 'h400; IT4_IN[87:80] <= {2'b00, IT4_OUT[85:80]};
                                end
                            endcase
                        end
                    end else begin
                        i_pfault <= 1;
                        fe_st <= FE_END;
                    end
                end
            end
            FE_MEM0: begin
                act_fe <= 0;
                if (BUS == 1) begin
                    fe_st <= FE_END;
                    if (iaddr < 'h40000000) // cache only if from RAM, need a way to not hardcode this
                        case (IC0_OUT[289:288])
                            0: begin IC0_WE <= 'h1fffffffff; IC0_IN <= {2'b01, 4'b0, 1'b1, iaddr[31:5], DIN}; end
                            1: begin IC1_WE <= 'h1fffffffff; IC1_IN <= {2'b00, 4'b0, 1'b1, iaddr[31:5], DIN};
                                     IC0_WE <= 'h1000000000; IC0_IN[289:288] <= 2'b00;
                            end
                        endcase
                end
            end
            FE_END: begin
                fe_inv_done <= 0;
                IC0_WE <= 0;
                IC1_WE <= 0;
                if (IC_MATCH != 3)  // found in cache
                    case (iaddr[4:0])
                        0: insn <= IC_DAT[31:0];
                        4: insn <= IC_DAT[63:32];
                        8: insn <= IC_DAT[95:64];
                        12: insn <= IC_DAT[127:96];
                        16: insn <= IC_DAT[159:128];
                        20: insn <= IC_DAT[191:160];
                        24: insn <= IC_DAT[223:192];
                        28: insn <= IC_DAT[255:224];
                        default: i_pfault <= 1;
                    endcase
                else
                    case (iaddr[4:0])
                        0: insn <= DIN[31:0];
                        4: insn <= DIN[63:32];
                        8: insn <= DIN[95:64];
                        12: insn <= DIN[127:96];
                        16: insn <= DIN[159:128];
                        20: insn <= DIN[191:160];
                        24: insn <= DIN[223:192];
                        28: insn <= DIN[255:224];
                        default: i_pfault <= 1;
                    endcase
                fe_st <= FE_START;
            end
            FE_INVASID: begin
                IT0_WE <= 0;
                IT1_WE <= 0;
                IT2_WE <= 0;
                IT3_WE <= 0;
                if (iCACHE_INIT_ADDR == 256) begin
                    iCACHE_INIT <= 0;
                    fe_st <= FE_START;
                end else begin
                    if (iasid_flush_cnt == 4) begin
                        iasid_flush_cnt <= 0;
                        iCACHE_INIT_ADDR <= iCACHE_INIT_ADDR + 1;
                    end else if (iasid_flush_cnt == 3) begin
                        if (IT0_OUT[85:77] == inv_asid) begin IT0_WE <= 'h7ff; IT0_IN <= 0; end
                        if (IT1_OUT[85:77] == inv_asid) begin IT1_WE <= 'h7ff; IT1_IN <= 0; end
                        if (IT2_OUT[85:77] == inv_asid) begin IT2_WE <= 'h7ff; IT2_IN <= 0; end
                        if (IT3_OUT[85:77] == inv_asid) begin IT3_WE <= 'h7ff; IT3_IN <= 0; end
                        iasid_flush_cnt <= 4;
                    end else
                        iasid_flush_cnt <= iasid_flush_cnt + 1;
                end
            end
            FE_INVADDR: begin
                iaddr_flush_cnt <= iaddr_flush_cnt + 1;
                if (iaddr_flush_cnt == 4) begin
                    IT0_WE <= 0;
                    IT1_WE <= 0;
                    IT2_WE <= 0;
                    IT3_WE <= 0;
                    IT4_WE <= 0;
                    IT5_WE <= 0;
                    iCACHE_INIT <= 0;
                    fe_st <= FE_START;
                end else if (iaddr_flush_cnt == 3) begin
                    if (IT0_OUT[69:19] == inv_addr[63:13]) begin IT0_WE <= 'h7ff; IT0_IN <= 0; end
                    if (IT1_OUT[69:19] == inv_addr[63:13]) begin IT1_WE <= 'h7ff; IT1_IN <= 0; end
                    if (IT2_OUT[69:19] == inv_addr[63:13]) begin IT2_WE <= 'h7ff; IT2_IN <= 0; end
                    if (IT3_OUT[69:19] == inv_addr[63:13]) begin IT3_WE <= 'h7ff; IT3_IN <= 0; end
                    if (IT4_OUT[69:19] == inv_addr[63:13]) begin IT4_WE <= 'h7ff; IT4_IN <= 0; end
                    if (IT5_OUT[69:19] == inv_addr[63:13]) begin IT5_WE <= 'h7ff; IT5_IN <= 0; end
                end
            end
        endcase
    end else begin
        act_fe <= 0;
        addr_fe <= 0;
    end
end

// DECODE
always @(posedge CLK)
begin
    if (RST == 0) begin
        rs1 <= 0;
        rs2 <= 0;
        rcs1 <= 0;
        rcs2 <= 0;
        rd2 <= 0;
        op <= OP_ILLINSTR;
        rs1i <= 0;
    end else if (DE_EN == 1) begin
        insn3 <= insn;
        rs1 <= r[insn[19:15]];
        rs2 <= r[insn[24:20]];
        rcs1 <= rc[insn[19:15]];
        rcs2 <= rc[insn[19:15] + 1];
        rd2 <= insn[11:7];
        rs1i <= insn[19:15];
        irq2 <= irq;
        if (i_pfault == 1)
            op <= OP_PAGEFAULT;
        else if (do_int == 1)
            op <= OP_INTR;
        else casez (insn)
            // 0110111
            'b?????????????????????????0110111: begin op <= OP_LUI; `IMM20; end
            // 0010111
            'b?????????????????????????0010111: begin op <= OP_AUIPC; `IMM20; end
            // 1101111
            'b?????????????????????????1101111: begin op <= OP_JAL; `CALC4; end
            // 1100111
            'b?????????????????000?????1100111: begin op <= OP_JALR; `CALC1; end
            // 1100011
            'b?????????????????000?????1100011: begin op <= OP_BEQ; `CALC5; end
            'b?????????????????001?????1100011: begin op <= OP_BNE; `CALC5; end
            'b?????????????????100?????1100011: begin op <= OP_BLT; `CALC5; end
            'b?????????????????101?????1100011: begin op <= OP_BGE; `CALC5; end
            'b?????????????????110?????1100011: begin op <= OP_BLTU; `CALC5; end
            'b?????????????????111?????1100011: begin op <= OP_BGEU; `CALC5; end
            // 0000011
            'b?????????????????000?????0000011: begin op <= OP_LB; `IMM12; end
            'b?????????????????001?????0000011: begin op <= OP_LH; `IMM12; end
            'b?????????????????010?????0000011: begin op <= OP_LW; `IMM12; end
            'b?????????????????011?????0000011: begin op <= OP_LD; `IMM12; end
            'b?????????????????100?????0000011: begin op <= OP_LBU; `IMM12; end
            'b?????????????????101?????0000011: begin op <= OP_LHU; `IMM12; end
            'b?????????????????110?????0000011: begin op <= OP_LWU; `IMM12; end
            // 0100011
            'b?????????????????000?????0100011: begin op <= OP_SB; `SIMM12; end
            'b?????????????????001?????0100011: begin op <= OP_SH; `SIMM12; end
            'b?????????????????010?????0100011: begin op <= OP_SW; `SIMM12; end
            'b?????????????????011?????0100011: begin op <= OP_SD; `SIMM12; end
            // 0010011
            'b?????????????????000?????0010011: begin op <= OP_ADDI; `IMM12; end
            'b000000???????????001?????0010011: begin op <= OP_SLLI; `SHAMTD; end
            'b?????????????????010?????0010011: begin op <= OP_SLTI; `IMM12; end
            'b?????????????????011?????0010011: begin op <= OP_SLTIU; `IMM12; end
            'b?????????????????100?????0010011: begin op <= OP_XORI; `IMM12; end
            'b000000???????????101?????0010011: begin op <= OP_SRLI; `SHAMTD; end
            'b010000???????????101?????0010011: begin op <= OP_SRAI; `SHAMTD; end
            'b?????????????????110?????0010011: begin op <= OP_ORI; `IMM12; end
            'b?????????????????111?????0010011: begin op <= OP_ANDI; `IMM12; end
            // 0110011
            'b0000000??????????000?????0110011: op <= OP_ADD;
            'b0100000??????????000?????0110011: op <= OP_SUB;
            'b0000000??????????001?????0110011: op <= OP_SLL;
            'b0000000??????????010?????0110011: op <= OP_SLT;
            'b0000000??????????011?????0110011: op <= OP_SLTU;
            'b0000000??????????100?????0110011: op <= OP_XOR;
            'b0000000??????????101?????0110011: op <= OP_SRL;
            'b0100000??????????101?????0110011: op <= OP_SRA;
            'b0000000??????????110?????0110011: op <= OP_OR;
            'b0000000??????????111?????0110011: op <= OP_AND;
            'b0000001??????????000?????0110011: op <= OP_MUL;
            'b0000001??????????001?????0110011: op <= OP_MULH;
            'b0000001??????????010?????0110011: op <= OP_MULHSU;
            'b0000001??????????011?????0110011: op <= OP_MULHU;
            'b0000001??????????100?????0110011: op <= OP_DIV;
            'b0000001??????????101?????0110011: op <= OP_DIVU;
            'b0000001??????????110?????0110011: op <= OP_REM;
            'b0000001??????????111?????0110011: op <= OP_REMU;
            // 0011011
            'b?????????????????000?????0011011: begin op <= OP_ADDIW; `CALC2; end
            'b0000000??????????001?????0011011: begin op <= OP_SLLIW; `CALC3; end
            'b0000000??????????101?????0011011: begin op <= OP_SRLIW; `CALC3; end
            'b0100000??????????101?????0011011: begin op <= OP_SRAIW; `CALC3; end
            // 0111011
            'b0000000??????????000?????0111011: op <= OP_ADDW;
            'b0100000??????????000?????0111011: op <= OP_SUBW;
            'b0000000??????????001?????0111011: op <= OP_SLLW;
            'b0000000??????????101?????0111011: op <= OP_SRLW;
            'b0100000??????????101?????0111011: op <= OP_SRAW;
            'b0000001??????????000?????0111011: op <= OP_MULW;
            'b0000001??????????100?????0111011: op <= OP_DIVW;
            'b0000001??????????101?????0111011: op <= OP_DIVUW;
            'b0000001??????????110?????0111011: op <= OP_REMW;
            'b0000001??????????111?????0111011: op <= OP_REMUW;
            // 0001111
            'b0000????????00000000000000001111: op <= OP_FENCE;
            'b00000000000000000010000000001111: op <= OP_FENCE_I;
            // 1110011
            'b00000000000000000000000001110011: op <= OP_ECALL;
//          'b00000000000100000000000001110011: op <= OP_EBREAK;
            'b00010000001000000000000001110011: op <= OP_SRET;
            'b00010000010100000000000001110011: op <= OP_WFI;
            'b?????????????????001?????1110011: begin op <= OP_CSRRW; `IMM12; end
            'b?????????????????010?????1110011: begin op <= OP_CSRRS; `IMM12; end
            'b?????????????????011?????1110011: begin op <= OP_CSRRC; `IMM12; end
            'b?????????????????101?????1110011: begin op <= OP_CSRRWI; `IMM12; end
            'b?????????????????110?????1110011: begin op <= OP_CSRRSI; `IMM12; end
            'b?????????????????111?????1110011: begin op <= OP_CSRRCI; `IMM12; end
            // CUSTOM-0 0001011
            'b000000000000?????000000000001011: op <= OP_INVASID;
            'b100000000000?????000000000001011: op <= OP_INVADDR;
            'b000000000000?????100?????0001011: op <= OP_SWAP16;
            'b100000000000?????100?????0001011: op <= OP_SWAP32;
            'b010000000000?????100?????0001011: op <= OP_SWAP64;
            'b110000000000?????100?????0001011: op <= OP_ILSB;
            'b001000000000?????100?????0001011: op <= OP_TLSB;
            'b101000000000?????100?????0001011: op <= OP_ILCB;
            'b011000000000?????100?????0001011: op <= OP_TLCB;
            'b111000000000?????100?????0001011: op <= OP_BCNT;
            'b000100000000?????100?????0001011: op <= OP_BSET;
            'b100100000000?????100?????0001011: op <= OP_BCLR;
            'b000000000000?????101?????0001011: op <= OP_RC_WR;
            'b100000000000?????101?????0001011: op <= OP_RC_RD;
            'b01000000000000000101000000001011: op <= OP_CHACHA;
            'b11000000000000000101000000001011: op <= OP_CHACHA_INIT;
            'b00100000000000000101000000001011: op <= OP_CHACHA_DONE;
            'b10100000000000000101000000001011: op <= OP_CUBEH;
            'b11100000000000000101000000001011: op <= OP_POLY1305MUL;
            'b00010000000000000101000000001011: op <= OP_POLY1305INIT;
            'b10010000000000000101000000001011: op <= OP_POLY1305DONE;
            default: op <= OP_ILLINSTR;
        endcase
    end
end

// EXEC1
always @(posedge CLK)
begin
    if (RST == 0) begin
        op2 <= OP_ILLINSTR;
        calc2 <= 0;
        rs1_2 <= 0;
        rs2_2 <= 0;
        rcs1_2 <= 0;
        rcs2_2 <= 0;
        ex_pc2 <= 0;
        res1 <= 0;
        rs1i_2 <= 0;
        ME_DO <= 0;
    end else if (EX1_EN)
    begin
        ME_DO <= 0;
        MULSS_EN <= 0;
        MULSU_EN <= 0;
        MULUU_EN <= 0;
        DIVS_EN <= 0;
        DIV_START <= 0;
        DIVS_DD_VALID <= 0;
        DIVS_DS_VALID <= 0;
        DIVU_DD_VALID <= 0;
        DIVU_DS_VALID <= 0;
        CHACHA_EN <= 0;
        CUBEH_EN <= 0;
        POLY1305MUL_EN <= 0;
        POLY1305INIT_EN <= 0;
        POLY1305DONE_EN <= 0;
        op2 <= op;
        calc2 <= calc;
        rs1_2 <= rs1;
        rs2_2 <= rs2;
        rcs1_2 <= rcs1;
        rcs2_2 <= rcs2;
        rd <= rd2;
        ex_pc2 <= ex_pc;
        rs1i_2 <= rs1i;
        irq3 <= irq2;
        case (op)
            OP_LUI:         res1 <= {{32{calc[19]}}, calc[19:0], 12'b0};
            OP_AUIPC:       res1 <= {{32{calc[19]}}, calc[19:0], 12'b0};
            OP_JAL:         res1 <= ex_pc + 4;
            OP_JALR:        res1 <= ex_pc + 4;
            OP_BEQ:         res1 <= rs1 == rs2 ? 1 : 0;
            OP_BNE:         res1 <= rs1 == rs2 ? 0 : 1;
            OP_BLT:         res1 <= $signed(rs1) < $signed(rs2) ? 1 : 0;
            OP_BGE:         res1 <= $signed(rs1) >= $signed(rs2) ? 1 : 0;
            OP_BLTU:        res1 <= rs1 < rs2 ? 1 : 0;
            OP_BGEU:        res1 <= rs1 >= rs2 ? 1 : 0;
            OP_LB:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_LH:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_LW:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_LD:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_LBU:         begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_LHU:         begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_LWU:         begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 1; end
            OP_SB:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 2; end
            OP_SH:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 2; end
            OP_SW:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 2; end
            OP_SD:          begin res1 <= {{52{calc[11]}}, calc[11:0]}; ME_DO <= 2; end
            OP_ADDI:        res1 <= {{52{calc[11]}}, calc[11:0]};
            OP_SLTI:        res1 <= {{52{calc[11]}}, calc[11:0]};
            OP_SLTIU:       res1 <= {{52{calc[11]}}, calc[11:0]};
            OP_XORI:        res1 <= {{52{calc[11]}}, calc[11:0]};
            OP_ORI:         res1 <= {{52{calc[11]}}, calc[11:0]};
            OP_ANDI:        res1 <= {{52{calc[11]}}, calc[11:0]};
            OP_SLLI:        res1 <= rs1 << calc[5:0];
            OP_SRLI:        res1 <= rs1 >> calc[5:0];
            OP_SRAI:        res1 <= $signed(rs1) >>> calc[5:0];
            OP_ADD:         res1 <= rs1 + rs2;
            OP_SUB:         res1 <= rs1 - rs2;
            OP_SLL:         res1 <= rs1 << rs2[5:0];
            OP_SLT:         res1 <= $signed(rs1) < $signed(rs2) ? 1 : 0;
            OP_SLTU:        res1 <= rs1 < rs2 ? 1 : 0;
            OP_XOR:         res1 <= rs1 ^ rs2;
            OP_SRL:         res1 <= rs1 >> rs2[5:0];
            OP_SRA:         res1 <= $signed(rs1) >>> rs2[5:0];
            OP_OR:          res1 <= rs1 | rs2;
            OP_AND:         res1 <= rs1 & rs2;
            OP_MUL:         begin MULUU_A <= rs1; MULUU_B <= rs2; MULUU_EN <= 1; end
            OP_MULH:        begin MULSS_A <= rs1; MULSS_B <= rs2; MULSS_EN <= 1; end
            OP_MULHU:       begin MULUU_A <= rs1; MULUU_B <= rs2; MULUU_EN <= 1; end
            OP_MULHSU:      begin MULSU_A <= rs1; MULSU_B <= rs2; MULSU_EN <= 1; end
            OP_DIV:         begin DIVS_DD <= rs1; DIVS_DS <= rs2; DIVS_DD_VALID <= 1; DIVS_DS_VALID <= 1; DIVS_EN <= 1; end
            OP_DIVU:        begin DIV_A <= rs1; DIV_B <= rs2; DIV_START <= 1; end
            OP_REM:         begin DIVS_DD <= rs1; DIVS_DS <= rs2; DIVS_DD_VALID <= 1; DIVS_DS_VALID <= 1; DIVS_EN <= 1; end
            OP_REMU:        begin DIV_A <= rs1; DIV_B <= rs2; DIV_START <= 1; end
            OP_ADDIW:       res1 <= rs1[31:0] + calc;
            OP_SLLIW:       res1 <= rs1[31:0] << calc[4:0];
            OP_SRLIW:       res1 <= {32'b0, rs1[31:0]} >> calc[4:0];
            OP_SRAIW:       res1 <= $signed(rs1[31:0]) >>> calc[4:0];
            OP_ADDW:        res1 <= rs1[31:0] + rs2[31:0];
            OP_SUBW:        res1 <= rs1[31:0] - rs2[31:0];
            OP_SLLW:        res1 <= rs1[31:0] << rs2[4:0];
            OP_SRLW:        res1 <= {32'b0, rs1[31:0]} >> rs2[4:0];
            OP_SRAW:        res1 <= $signed(rs1[31:0]) >>> rs2[4:0];
            OP_MULW:        begin MULUU_A <= rs1[31:0]; MULUU_B <= rs2[31:0]; MULUU_EN <= 1; end
            OP_DIVW:        begin DIVS_DD <= rs1[31:0]; DIVS_DS <= rs2[31:0]; DIVS_DD_VALID <= 1; DIVS_DS_VALID <= 1; DIVS_EN <= 1; end
            //OP_DIVUW:       begin DIVU_DD <= rs1[31:0]; DIVU_DS <= rs2[31:0]; DIVU_DD_VALID <= 1; DIVU_DS_VALID <= 1; DIVU_EN <= 1; end
            OP_DIVUW:       begin DIV_A <= { 32'b0, rs1[31:0] }; DIV_B <= { 32'b0, rs2[31:0] }; DIV_START <= 1; end
            OP_REMW:        begin DIVS_DD <= rs1[31:0]; DIVS_DS <= rs2[31:0]; DIVS_DD_VALID <= 1; DIVS_DS_VALID <= 1; DIVS_EN <= 1; end
            //OP_REMUW:       begin DIVU_DD <= rs1[31:0]; DIVU_DS <= rs2[31:0]; DIVU_DD_VALID <= 1; DIVU_DS_VALID <= 1; DIVU_EN <= 1; end
            OP_REMUW:       begin DIV_A <= { 32'b0, rs1[31:0] }; DIV_B <= {32'b0, rs2[31:0] }; DIV_START <= 1; end
            OP_WFI:         if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT;
            OP_SRET:        if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT;
            OP_CSRRW:       if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else res1 <= cr[calc[4:0]];
            OP_CSRRS:       if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else res1 <= cr[calc[4:0]];
            OP_CSRRC:       if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else res1 <= cr[calc[4:0]];
            OP_CSRRWI:      if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else res1 <= cr[calc[4:0]];
            OP_CSRRSI:      if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else res1 <= cr[calc[4:0]];
            OP_CSRRCI:      if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else res1 <= cr[calc[4:0]];
            OP_INVASID:     if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else begin res1 <= rs1; ME_DO <= 3; end
            OP_INVADDR:     if (priv_lvl_ex1 != 1) op2 <= OP_PROTFAULT; else begin res1 <= rs1; ME_DO <= 4; end
            OP_SWAP16:      res1 <= { 48'b0, rs1[7:0], rs1[15:8] };
            OP_SWAP32:      res1 <= { 32'b0, rs1[7:0], rs1[15:8], rs1[23:16], rs1[31:24] };
            OP_SWAP64:      res1 <= { rs1[7:0], rs1[15:8], rs1[23:16], rs1[31:24], rs1[39:32], rs1[47:40], rs1[55:48], rs1[63:56] };
            OP_ILSB:        res1 <= -rs1;
            OP_TLSB:        res1 <= rs1 - 1;
            OP_ILCB:        res1 <= ~(rs1 + 1);
            OP_TLCB:        res1 <= rs1 + 1;
            OP_BCNT:        begin
                for (int i = 0; i < 64; i++) begin
                    if (rs1 == (1 << i))
                        res1 <= i;
                end
            end
            OP_BSET:        res1 <= rs1[5:0];
            OP_BCLR:        res1 <= rs1[5:0];
            OP_CHACHA:      CHACHA_EN <= 1;
            OP_CUBEH:       CUBEH_EN <= 1;
            OP_POLY1305MUL: POLY1305MUL_EN <= 1;
            OP_POLY1305INIT:POLY1305INIT_EN <= 1;
            OP_POLY1305DONE:POLY1305DONE_EN <= 1;
            default:        res1 <= 0;
        endcase
    end
end

// EXEC2
always @(posedge CLK)
begin
    if (RST == 0) begin
        rs1_3 <= 0;
        rs2_3 <= 0;
        rcs1_3 <= 0;
        calc3 <= 0;
        op3 <= OP_ILLINSTR;
        new_pc <= 0;
        FAULT <= 0;
        res2 <= 0;
        wfi <= 0;
        rs1i_3 <= 0;
    end else if (EX2_EN)
    begin
        wfi <= 0;
        sret <= 0;
        FAULT <= 0;
        rs1_3 <= rs1_2;
        rs2_3 <= rs2_2;
        rcs1_3 <= rcs1_2;
        rcs2_3 <= rcs2_2;
        calc3 <= calc2;
        op3 <= op2;
        new_pc <= ex_pc2 + 4;
        rs1i_3 <= rs1i_2;
        irq4 <= irq3;
        case (op2)
            OP_LUI:         res2 <= res1;
            OP_AUIPC:       res2 <= ex_pc2 + res1;
            OP_JAL:         begin res2 <= res1; new_pc <= ex_pc2 + calc2; end
            OP_JALR:        begin res2 <= res1; new_pc <= {rs1_2[63:1] + calc2[63:1], 1'b0}; end
            OP_BEQ:         if (res1 == 1) new_pc <= ex_pc2 + calc2;
            OP_BNE:         if (res1 == 1) new_pc <= ex_pc2 + calc2;
            OP_BLT:         if (res1 == 1) new_pc <= ex_pc2 + calc2;
            OP_BGE:         if (res1 == 1) new_pc <= ex_pc2 + calc2;
            OP_BLTU:        if (res1 == 1) new_pc <= ex_pc2 + calc2;
            OP_BGEU:        if (res1 == 1) new_pc <= ex_pc2 + calc2;
            OP_LB:          begin res2 <= rs1_2 + $signed(res1); end
            OP_LH:          begin res2 <= rs1_2 + $signed(res1); end
            OP_LW:          begin res2 <= rs1_2 + $signed(res1); end
            OP_LD:          begin res2 <= rs1_2 + $signed(res1); end
            OP_LBU:         begin res2 <= rs1_2 + $signed(res1); end
            OP_LHU:         begin res2 <= rs1_2 + $signed(res1); end
            OP_LWU:         begin res2 <= rs1_2 + $signed(res1); end
            OP_SB:          begin res2 <= rs1_2 + $signed(res1); end
            OP_SH:          begin res2 <= rs1_2 + $signed(res1); end
            OP_SW:          begin res2 <= rs1_2 + $signed(res1); end
            OP_SD:          begin res2 <= rs1_2 + $signed(res1); end
            OP_ADDI:        res2 <= rs1_2 + res1;
            OP_SLTI:        res2 <= $signed(rs1_2) < $signed(res1) ? 1 : 0;
            OP_SLTIU:       res2 <= rs1_2 < res1 ? 1 : 0;
            OP_XORI:        res2 <= rs1_2 ^ res1;
            OP_ORI:         res2 <= rs1_2 | res1;
            OP_ANDI:        res2 <= rs1_2 & res1;
            OP_SLLI:        res2 <= res1;
            OP_SRLI:        res2 <= res1;
            OP_SRAI:        res2 <= res1;
            OP_ADD:         res2 <= res1;
            OP_SUB:         res2 <= res1;
            OP_SLL:         res2 <= res1;
            OP_SLT:         res2 <= res1;
            OP_SLTU:        res2 <= res1;
            OP_XOR:         res2 <= res1;
            OP_SRL:         res2 <= res1;
            OP_SRA:         res2 <= res1;
            OP_OR:          res2 <= res1;
            OP_AND:         res2 <= res1;
            OP_ADDIW:       res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SLLIW:       res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SRLIW:       res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SRAIW:       res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_ADDW:        res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SUBW:        res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SLLW:        res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SRLW:        res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_SRAW:        res2 <= {{32{res1[31]}}, res1[31:0]};
            OP_ECALL:       FAULT <= 1;
            OP_SRET:        sret <= 1;
            OP_WFI:         wfi <= 1;
            OP_INTR:        FAULT <= 1;
            OP_CSRRW:       res2 <= res1;
            OP_CSRRS:       res2 <= res1;
            OP_CSRRC:       res2 <= res1;
            OP_CSRRWI:      res2 <= res1;
            OP_CSRRSI:      res2 <= res1;
            OP_CSRRCI:      res2 <= res1;
            OP_INVASID:     res2 <= res1;
            OP_INVADDR:     res2 <= res1;
            OP_SWAP16:      res2 <= res1;
            OP_SWAP32:      res2 <= res1;
            OP_SWAP64:      res2 <= res1;
            OP_ILSB:        res2 <= rs1_2 & res1;
            OP_TLSB:        res2 <= rs1_2 & res1;
            OP_ILCB:        res2 <= ~(rs1_2 | res1);
            OP_TLCB:        res2 <= rs1_2 | res1;
            OP_BCNT:        res2 <= res1;
            OP_BSET:        res2 <= res1;
            OP_BCLR:        res2 <= res1;
            OP_PAGEFAULT:   FAULT <= 1;
            OP_PROTFAULT:   FAULT <= 1;
            OP_ILLINSTR:    FAULT <= 1;
            default:        res2 <= 0;
        endcase
    end
end

// MEM
always @(posedge CLK)
begin
    if (RST == 0) begin
        cr[30] <= 0;
        cr[31] <= 0;
        time_inc <= 0;
    end else begin
        cr[31] <= cr[31] + 1;
        if (time_inc == 199) begin
            time_inc <= 0;
            cr[30] <= cr[30] + 1;
        end else
            time_inc <= time_inc + 1;
    end
    if (RST == 0) begin
        r[0] <= 0;
        r[2] <= {40'ha00000003f, CORE_ID[7:0], 16'b0000};
        r[10] <= {56'b0, CORE_ID[7:0]};
        pc <= 'ha000000080000000;
        cr[29] <= 0;
        cr[5] <= 0;
        cr[4] <= 0;
        cr[3] <= 0;
        cr[2] <= 64'b100;   // supervisor bit
        cr[1] <= 0;
        cr[0] <= 0;
        priv_lvl_fe <= 1;
        FAULT2 <= 0;
        FAULT_DONE <= 0;
        exc_cnt <= 0;
        sret_cnt <= 0;
        DATA_ADDR <= 0;
        DATA_ACT <= 0;
        CHACHA_CNT <= 23;
        CUBEH_CNT <= 9;
        POLY1305MUL_CNT <= 19;
        POLY1305INIT_CNT <= 3;
        POLY1305DONE_CNT <= 17;
        inv_asid <= 0;
        inv_addr <= 1;
    end else if (ME_EN == 1) begin
        priv_lvl_fe <= cr[2][2];
        priv_lvl_dc <= cr[2][2];
        inv_asid <= 0;
        inv_addr <= 1;
        cr[29] <= insn_cnt;
        if (sret == 1) begin
            case (sret_cnt)
                0: begin
                    DATA_ADDR <= r[2];
                    dst1 <= cr[3];
                    fault <= 4;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 1;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        r[2] <= DATA_DIN;
                        sret_cnt <= 1;
                        stack_addr <= r[2] + 8;
                        DATA_ACT <= 0;
                    end else if (DATA_ST == 2) begin
                        sret_cnt <= 4;
                        DATA_ACT <= 0;
                    end
                end
                1: begin
                    DATA_ADDR <= stack_addr;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 1;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        pc <= DATA_DIN;
                        sret_cnt <= 2;
                        stack_addr <= stack_addr + 8;
                        DATA_ACT <= 0;
                    end else if (DATA_ST == 2) begin
                        sret_cnt <= 4;
                        DATA_ACT <= 0;
                    end
                end
                2: begin
                    DATA_ADDR <= stack_addr;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 1;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        cr[2] <= DATA_DIN;
                        cr[1] <= stack_addr + 8;
                        sret_cnt <= 5;
                        DATA_ACT <= 0;
                        FAULT_DONE <= 1;
                    end else if (DATA_ST == 2) begin
                        sret_cnt <= 4;
                        DATA_ACT <= 0;
                    end
                end
                3: begin
                end
                4: begin
                    // stackfault handler
                    DATA_ADDR <= cr[0] + 32;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 1;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        sret_cnt <= 5;
                        DATA_ACT <= 0;
                        pc <= DATA_DIN;
                        r[2] <= dst1;
                        FAULT_DONE <= 1;
                    end else if (DATA_ST == 2) begin
                        DATA_ACT <= 0;
                        sret_cnt <= 3;
                        //SYS_FAULT <= 1;
                    end
                end
                5: begin
                    sret_cnt <= 0;
                    FAULT_DONE <= 0;
                end
            endcase
        end else if ((FAULT == 1) || (FAULT2 == 1)) begin
            case (exc_cnt)
                0: begin
                    if (cr[2][2] == 0)
                        stack_addr <= cr[1];
                    else
                        stack_addr <= r[2];
                    exc_cnt <= 1;
                    cr[2][2] <= 1;
                    DATA_DOUT <= cr[2];
                    dst1 <= cr[3];
                    if (FAULT == 1) begin
                        case (op3)
                            OP_ECALL:       begin fault <= 0; pc <= new_pc; end
                            OP_PAGEFAULT:   begin fault <= 1; cr[8][2:0] <= {cr[2][2], 1'b0, 1'b0};  cr[7] <= ex_pc; cr[6] <= prev_pc; pc <= ex_pc; end
                            OP_PROTFAULT:   begin fault <= 2; cr[8][63:32] <= insn3; cr[7] <= ex_pc; pc <= ex_pc; end
                            OP_INTR:        begin fault <= irq4 + 16; pc <= new_pc; end
                            default:        begin fault <= 3; cr[8][63:32] <= insn3; cr[7] <= ex_pc; pc <= ex_pc; end
                        endcase
                    end else if (FAULT2 == 1) begin
                        pc <= ex_pc;
                        cr[7] <= ex_pc;
                        cr[8][2:0] <= {cr[2][2], 1'b1, ME_DO == 1 ? 1'b0 : 1'b1};
                        cr[6] <= res2;
                        fault <= 1;
                    end
                end
                1: begin
                    DATA_ADDR <= stack_addr - 8;
                    DMASK <= 'b11111111;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 2;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        exc_cnt <= 2;
                        DATA_ACT <= 0;
                        stack_addr <= stack_addr - 8;
                    end else if (DATA_ST == 2) begin
                        DATA_ACT <= 0;
                        exc_cnt <= 4;
                    end
                end
                2: begin
                    DATA_ADDR <= stack_addr - 8;
                    DATA_DOUT <= pc;
                    DMASK <= 'b11111111;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 2;
                    else 
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        exc_cnt <= 3;
                        DATA_ACT <= 0;
                        stack_addr <= stack_addr - 8;
                    end else if (DATA_ST == 2) begin
                        DATA_ACT <= 0;
                        exc_cnt <= 4;
                    end
                end
                3: begin
                    DATA_ADDR <= stack_addr - 8;
                    DATA_DOUT <= r[2];
                    DMASK <= 'b11111111;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 2;
                    else 
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        exc_cnt <= 5;
                        DATA_ACT <= 0;
                        stack_addr <= stack_addr - 8;
                    end else if (DATA_ST == 2) begin
                        DATA_ACT <= 0;
                        exc_cnt <= 4;
                    end
                end
                4: begin
                    // stackfault handler
                    DATA_ADDR <= cr[0] + 32;
                    if (DATA_ACT == 0)
                        DATA_ACT <= 1;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        exc_cnt <= 6;
                        DATA_ACT <= 0;
                        pc <= DATA_DIN;
                        r[2] <= dst1;
                        FAULT_DONE <= 1;
                    end else if (DATA_ST == 2) begin
                        DATA_ACT <= 0;
                        exc_cnt <= 7;
                        //SYS_FAULT <= 1;
                    end
                end
                5: begin
                    DATA_ADDR <= cr[0] + (fault << 3);
                    if (DATA_ACT == 0)
                        DATA_ACT <= 1;
                    else
                        DATA_ACT <= 7;
                    if (DATA_ST == 1) begin
                        exc_cnt <= 6;
                        DATA_ACT <= 0;
                        pc <= DATA_DIN;
                        r[2] <= stack_addr;
                        FAULT_DONE <= 1;
                    end else if (DATA_ST == 2) begin
                        DATA_ACT <= 0;
                        exc_cnt <= 4;
                    end
                end
                6: begin
                    exc_cnt <= 0;
                    cr[2][0] <= 0;
                    FAULT2 <= 0;
                    FAULT_DONE <= 0;
                end
                7: begin
                end
            endcase
        end else begin
            pc <= new_pc;
            if (ME_DO != 0) begin
                if (DATA_ACT == 0)
                    DATA_ACT <= ME_DO;
                else
                    DATA_ACT <= 7;
                DATA_ADDR <= res2;
                if (ME_DO == 2) DATA_DOUT <= rs2_3;
                if (DATA_ST == 1) begin
                    DATA_ACT <= 0;
                end else if (DATA_ST == 2) begin
                    DATA_ACT <= 0;
                    FAULT2 <= 1;
                end
            end
            case (op3)
                OP_LUI:     if (rd != 0) r[rd] <= res2;
                OP_AUIPC:   if (rd != 0) r[rd] <= res2;
                OP_JAL:     if (rd != 0) r[rd] <= res2;
                OP_JALR:    if (rd != 0) r[rd] <= res2;
                OP_LB:      begin DMASK <= 'b00000001; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= {{56{DATA_DIN[7]}}, DATA_DIN[7:0]}; end end
                OP_LH:      begin DMASK <= 'b00000011; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= {{48{DATA_DIN[15]}}, DATA_DIN[15:0]}; end end
                OP_LW:      begin DMASK <= 'b00001111; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= {{32{DATA_DIN[31]}}, DATA_DIN[31:0]}; end end
                OP_LD:      begin DMASK <= 'b11111111; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= DATA_DIN; end end
                OP_LBU:     begin DMASK <= 'b00000001; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= {56'b0, DATA_DIN[7:0]}; end end
                OP_LHU:     begin DMASK <= 'b00000011; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= {48'b0, DATA_DIN[15:0]}; end end
                OP_LWU:     begin DMASK <= 'b00001111; if ((DATA_ST == 1) && (rd != 0)) begin r[rd] <= {32'b0, DATA_DIN[31:0]}; end end
                OP_SB:      DMASK <= 'b00000001;
                OP_SH:      DMASK <= 'b00000011;
                OP_SW:      DMASK <= 'b00001111;
                OP_SD:      DMASK <= 'b11111111;
                OP_ADDI:    if (rd != 0) r[rd] <= res2;
                OP_SLTI:    if (rd != 0) r[rd] <= res2;
                OP_SLTIU:   if (rd != 0) r[rd] <= res2;
                OP_XORI:    if (rd != 0) r[rd] <= res2;
                OP_ORI:     if (rd != 0) r[rd] <= res2;
                OP_ANDI:    if (rd != 0) r[rd] <= res2;
                OP_SLLI:    if (rd != 0) r[rd] <= res2;
                OP_SRLI:    if (rd != 0) r[rd] <= res2;
                OP_SRAI:    if (rd != 0) r[rd] <= res2;
                OP_ADD:     if (rd != 0) r[rd] <= res2;
                OP_SUB:     if (rd != 0) r[rd] <= res2;
                OP_SLL:     if (rd != 0) r[rd] <= res2;
                OP_SLT:     if (rd != 0) r[rd] <= res2;
                OP_SLTU:    if (rd != 0) r[rd] <= res2;
                OP_XOR:     if (rd != 0) r[rd] <= res2;
                OP_SRL:     if (rd != 0) r[rd] <= res2;
                OP_SRA:     if (rd != 0) r[rd] <= res2;
                OP_OR:      if (rd != 0) r[rd] <= res2;
                OP_AND:     if (rd != 0) r[rd] <= res2;
                OP_MUL:     if (rd != 0) r[rd] <= MULUU_P[63:0];
                OP_MULH:    if (rd != 0) r[rd] <= MULSS_P[127:64];
                OP_MULHU:   if (rd != 0) r[rd] <= MULUU_P[127:64];
                OP_MULHSU:  if (rd != 0) r[rd] <= MULSU_P[127:64];
                OP_DIV:     if (rd != 0) r[rd] <= DIVS_OUT_DATA[127:64];
                OP_DIVU:    if (rd != 0) r[rd] <= DIV_Q;
                OP_REM:     if (rd != 0) r[rd] <= DIVS_OUT_DATA[63:0];
                OP_REMU:    if (rd != 0) r[rd] <= DIV_R;
                OP_ADDIW:   if (rd != 0) r[rd] <= res2;
                OP_SLLIW:   if (rd != 0) r[rd] <= res2;
                OP_SRLIW:   if (rd != 0) r[rd] <= res2;
                OP_SRAIW:   if (rd != 0) r[rd] <= res2;
                OP_ADDW:    if (rd != 0) r[rd] <= res2;
                OP_SUBW:    if (rd != 0) r[rd] <= res2;
                OP_SLLW:    if (rd != 0) r[rd] <= res2;
                OP_SRLW:    if (rd != 0) r[rd] <= res2;
                OP_SRAW:    if (rd != 0) r[rd] <= res2;
                OP_MULW:    if (rd != 0) r[rd] <= {{32{MULUU_P[31]}}, MULUU_P[31:0]};
                OP_DIVW:    if (rd != 0) r[rd] <= {{32{DIVS_OUT_DATA[95]}}, DIVS_OUT_DATA[95:64]};
                OP_DIVUW:   if (rd != 0) r[rd] <= {{32{DIV_Q[31]}}, DIV_Q[31:0]};
                OP_REMW:    if (rd != 0) r[rd] <= {{32{DIVS_OUT_DATA[31]}}, DIVS_OUT_DATA[31:0]};
                OP_REMUW:   if (rd != 0) r[rd] <= {{32{DIV_R[31]}}, DIV_R[31:0]};
                OP_CSRRW:   begin if (rd != 0) r[rd] <= res2; cr[calc3[4:0]] <= rs1_3; end
                OP_CSRRS:   begin if (rd != 0) r[rd] <= res2; cr[calc3[4:0]] <= res2 | rs1_3; end
                OP_CSRRC:   begin if (rd != 0) r[rd] <= res2; cr[calc3[4:0]] <= res2 & ~rs1_3; end
                OP_CSRRWI:  begin if (rd != 0) r[rd] <= res2; cr[calc3[4:0]] <= {59'b0, rs1i_3}; end
                OP_CSRRSI:  begin if (rd != 0) r[rd] <= res2; cr[calc3[4:0]] <= res2 | {59'b0, rs1i_3}; end
                OP_CSRRCI:  begin if (rd != 0) r[rd] <= res2; cr[calc3[4:0]] <= res2 & ~{59'b0, rs1i_3}; end
                OP_INVASID: inv_asid <= res2[8:0];
                OP_INVADDR: inv_addr <= res2;
                OP_SWAP16:  if (rd != 0) r[rd] <= res2;
                OP_SWAP32:  if (rd != 0) r[rd] <= res2;
                OP_SWAP64:  if (rd != 0) r[rd] <= res2;
                OP_ILSB:    if (rd != 0) r[rd] <= res2;
                OP_TLSB:    if (rd != 0) r[rd] <= res2;
                OP_ILCB:    if (rd != 0) r[rd] <= res2;
                OP_TLCB:    if (rd != 0) r[rd] <= res2;
                OP_BCNT:    if (rd != 0) r[rd] <= res2;
                OP_BSET:    if (rd != 0) r[rd][res2] <= 1;
                OP_BCLR:    if (rd != 0) r[rd][res2] <= 0;
                OP_RC_RD:   if (rd != 0) r[rd] <= { rcs2_3, rcs1_3 };
                OP_RC_WR:   { rc[rd + 1], rc[rd] } <= rs1_2;
                OP_CHACHA_INIT: begin
                    rc[16] <= 'h61707865; rc[0] <= 'h61707865;
                    rc[17] <= 'h3320646e; rc[1] <= 'h3320646e;
                    rc[18] <= 'h79622d32; rc[2] <= 'h79622d32;
                    rc[19] <= 'h6b206574; rc[3] <= 'h6b206574;
                    rc[4] <= rc[20]; rc[5] <= rc[21]; rc[6] <= rc[22]; rc[7] <= rc[23];
                    rc[8] <= rc[24]; rc[9] <= rc[25]; rc[10] <= rc[26]; rc[11] <= rc[27];
                    rc[12] <= rc[28]; rc[13] <= rc[29]; rc[14] <= rc[30]; rc[15] <= rc[31];
                end
                OP_CHACHA_DONE: begin
                    rc[0] <= rc[0] + rc[16]; rc[1] <= rc[1] + rc[17]; rc[2] <= rc[2] + rc[18]; rc[3] <= rc[3] + rc[19];
                    rc[4] <= rc[4] + rc[20]; rc[5] <= rc[5] + rc[21]; rc[6] <= rc[6] + rc[22]; rc[7] <= rc[7] + rc[23];
                    rc[8] <= rc[8] + rc[24]; rc[9] <= rc[9] + rc[25]; rc[10] <= rc[10] + rc[26]; rc[11] <= rc[11] + rc[27];
                    rc[12] <= rc[12] + rc[28]; rc[13] <= rc[13] + rc[29]; rc[14] <= rc[14] + rc[30]; rc[15] <= rc[15] + rc[31];
                end
                OP_CHACHA: begin
                    `define QRND00(x0, x1, x2, x3) x0 <= x0 + x1
                    `define QRND10(x0, x1, x2, x3) x2 <= x2 + x3
                    `define QRND20(x0, x1, x2, x3) x0 <= x0 + x1
                    `define QRND30(x0, x1, x2, x3) x2 <= x2 + x3
                    `define QRND01(x0, x1, x2, x3) x3 <= x3 ^ x0
                    `define QRND11(x0, x1, x2, x3) x1 <= x1 ^ x2
                    `define QRND21(x0, x1, x2, x3) x3 <= x3 ^ x0
                    `define QRND31(x0, x1, x2, x3) x1 <= x1 ^ x2
                    `define QRND02(x0, x1, x2, x3) x3 <= { x3[15:0], x3[31:16] }
                    `define QRND12(x0, x1, x2, x3) x1 <= { x1[19:0], x1[31:20] }
                    `define QRND22(x0, x1, x2, x3) x3 <= { x3[23:0], x3[31:24] }
                    `define QRND32(x0, x1, x2, x3) x1 <= { x1[24:0], x1[31:25] }
                    CHACHA_CNT <= CHACHA_CNT - 1;
                    case (CHACHA_CNT)
                        23: begin
                            `QRND00(rc[0], rc[4], rc[8], rc[12]);
                            `QRND00(rc[1], rc[5], rc[9], rc[13]);
                            `QRND00(rc[2], rc[6], rc[10], rc[14]);
                            `QRND00(rc[3], rc[7], rc[11], rc[15]);
                        end
                        22: begin
                            `QRND01(rc[0], rc[4], rc[8], rc[12]);
                            `QRND01(rc[1], rc[5], rc[9], rc[13]);
                            `QRND01(rc[2], rc[6], rc[10], rc[14]);
                            `QRND01(rc[3], rc[7], rc[11], rc[15]);
                        end
                        21: begin
                            `QRND02(rc[0], rc[4], rc[8], rc[12]);
                            `QRND02(rc[1], rc[5], rc[9], rc[13]);
                            `QRND02(rc[2], rc[6], rc[10], rc[14]);
                            `QRND02(rc[3], rc[7], rc[11], rc[15]);
                        end
                        20: begin
                            `QRND10(rc[0], rc[4], rc[8], rc[12]);
                            `QRND10(rc[1], rc[5], rc[9], rc[13]);
                            `QRND10(rc[2], rc[6], rc[10], rc[14]);
                            `QRND10(rc[3], rc[7], rc[11], rc[15]);                        
                        end
                        19: begin
                            `QRND11(rc[0], rc[4], rc[8], rc[12]);
                            `QRND11(rc[1], rc[5], rc[9], rc[13]);
                            `QRND11(rc[2], rc[6], rc[10], rc[14]);
                            `QRND11(rc[3], rc[7], rc[11], rc[15]);                        
                        end
                        18: begin
                            `QRND12(rc[0], rc[4], rc[8], rc[12]);
                            `QRND12(rc[1], rc[5], rc[9], rc[13]);
                            `QRND12(rc[2], rc[6], rc[10], rc[14]);
                            `QRND12(rc[3], rc[7], rc[11], rc[15]);                        
                        end
                        17: begin
                            `QRND20(rc[0], rc[4], rc[8], rc[12]);
                            `QRND20(rc[1], rc[5], rc[9], rc[13]);
                            `QRND20(rc[2], rc[6], rc[10], rc[14]);
                            `QRND20(rc[3], rc[7], rc[11], rc[15]);
                        end
                        16: begin
                            `QRND21(rc[0], rc[4], rc[8], rc[12]);
                            `QRND21(rc[1], rc[5], rc[9], rc[13]);
                            `QRND21(rc[2], rc[6], rc[10], rc[14]);
                            `QRND21(rc[3], rc[7], rc[11], rc[15]);
                        end
                        15: begin
                            `QRND22(rc[0], rc[4], rc[8], rc[12]);
                            `QRND22(rc[1], rc[5], rc[9], rc[13]);
                            `QRND22(rc[2], rc[6], rc[10], rc[14]);
                            `QRND22(rc[3], rc[7], rc[11], rc[15]);
                        end
                        14: begin
                            `QRND30(rc[0], rc[4], rc[8], rc[12]);
                            `QRND30(rc[1], rc[5], rc[9], rc[13]);
                            `QRND30(rc[2], rc[6], rc[10], rc[14]);
                            `QRND30(rc[3], rc[7], rc[11], rc[15]);
                        end
                        13: begin
                            `QRND31(rc[0], rc[4], rc[8], rc[12]);
                            `QRND31(rc[1], rc[5], rc[9], rc[13]);
                            `QRND31(rc[2], rc[6], rc[10], rc[14]);
                            `QRND31(rc[3], rc[7], rc[11], rc[15]);
                        end
                        12: begin
                            `QRND32(rc[0], rc[4], rc[8], rc[12]);
                            `QRND32(rc[1], rc[5], rc[9], rc[13]);
                            `QRND32(rc[2], rc[6], rc[10], rc[14]);
                            `QRND32(rc[3], rc[7], rc[11], rc[15]);
                        end
                        11: begin
                            `QRND00(rc[0], rc[5], rc[10], rc[15]);
                            `QRND00(rc[1], rc[6], rc[11], rc[12]);
                            `QRND00(rc[2], rc[7], rc[8], rc[13]);
                            `QRND00(rc[3], rc[4], rc[9], rc[14]);
                        end
                        10: begin
                            `QRND01(rc[0], rc[5], rc[10], rc[15]);
                            `QRND01(rc[1], rc[6], rc[11], rc[12]);
                            `QRND01(rc[2], rc[7], rc[8], rc[13]);
                            `QRND01(rc[3], rc[4], rc[9], rc[14]);
                        end
                        9: begin
                            `QRND02(rc[0], rc[5], rc[10], rc[15]);
                            `QRND02(rc[1], rc[6], rc[11], rc[12]);
                            `QRND02(rc[2], rc[7], rc[8], rc[13]);
                            `QRND02(rc[3], rc[4], rc[9], rc[14]);
                        end
                        8: begin
                            `QRND10(rc[0], rc[5], rc[10], rc[15]);
                            `QRND10(rc[1], rc[6], rc[11], rc[12]);
                            `QRND10(rc[2], rc[7], rc[8], rc[13]);
                            `QRND10(rc[3], rc[4], rc[9], rc[14]);
                        end
                        7: begin
                            `QRND11(rc[0], rc[5], rc[10], rc[15]);
                            `QRND11(rc[1], rc[6], rc[11], rc[12]);
                            `QRND11(rc[2], rc[7], rc[8], rc[13]);
                            `QRND11(rc[3], rc[4], rc[9], rc[14]);
                        end
                        6: begin
                            `QRND12(rc[0], rc[5], rc[10], rc[15]);
                            `QRND12(rc[1], rc[6], rc[11], rc[12]);
                            `QRND12(rc[2], rc[7], rc[8], rc[13]);
                            `QRND12(rc[3], rc[4], rc[9], rc[14]);
                        end
                        5: begin
                            `QRND20(rc[0], rc[5], rc[10], rc[15]);
                            `QRND20(rc[1], rc[6], rc[11], rc[12]);
                            `QRND20(rc[2], rc[7], rc[8], rc[13]);
                            `QRND20(rc[3], rc[4], rc[9], rc[14]);
                        end
                        4: begin
                            `QRND21(rc[0], rc[5], rc[10], rc[15]);
                            `QRND21(rc[1], rc[6], rc[11], rc[12]);
                            `QRND21(rc[2], rc[7], rc[8], rc[13]);
                            `QRND21(rc[3], rc[4], rc[9], rc[14]);
                        end
                        3: begin
                            `QRND22(rc[0], rc[5], rc[10], rc[15]);
                            `QRND22(rc[1], rc[6], rc[11], rc[12]);
                            `QRND22(rc[2], rc[7], rc[8], rc[13]);
                            `QRND22(rc[3], rc[4], rc[9], rc[14]);
                        end
                        2: begin
                            `QRND30(rc[0], rc[5], rc[10], rc[15]);
                            `QRND30(rc[1], rc[6], rc[11], rc[12]);
                            `QRND30(rc[2], rc[7], rc[8], rc[13]);
                            `QRND30(rc[3], rc[4], rc[9], rc[14]);
                        end
                        1: begin
                            `QRND31(rc[0], rc[5], rc[10], rc[15]);
                            `QRND31(rc[1], rc[6], rc[11], rc[12]);
                            `QRND31(rc[2], rc[7], rc[8], rc[13]);
                            `QRND31(rc[3], rc[4], rc[9], rc[14]);
                        end
                        0: begin
                            `QRND32(rc[0], rc[5], rc[10], rc[15]);
                            `QRND32(rc[1], rc[6], rc[11], rc[12]);
                            `QRND32(rc[2], rc[7], rc[8], rc[13]);
                            `QRND32(rc[3], rc[4], rc[9], rc[14]);
                            CHACHA_CNT <= 23;
                        end
                    endcase
                end
                OP_CUBEH: begin
                    CUBEH_CNT <= CUBEH_CNT - 1;
                    case (CUBEH_CNT)
                        9: begin
                            rc[16] <= rc[16] + rc[0]; rc[17] <= rc[17] + rc[1]; rc[18] <= rc[18] + rc[2]; rc[19] <= rc[19] + rc[3];
                            rc[20] <= rc[20] + rc[4]; rc[21] <= rc[21] + rc[5]; rc[22] <= rc[22] + rc[6]; rc[23] <= rc[23] + rc[7];
                            rc[24] <= rc[24] + rc[8]; rc[25] <= rc[25] + rc[9]; rc[26] <= rc[26] + rc[10]; rc[27] <= rc[27] + rc[11];
                            rc[28] <= rc[28] + rc[12]; rc[29] <= rc[29] + rc[13]; rc[30] <= rc[30] + rc[14]; rc[31] <= rc[31] + rc[15];
                        end
                        8: begin
                            rc[0] <= { rc[0][24:0], rc[0][31:25] }; rc[1] <= { rc[1][24:0], rc[1][31:25] };
                            rc[2] <= { rc[2][24:0], rc[2][31:25] }; rc[3] <= { rc[3][24:0], rc[3][31:25] };
                            rc[4] <= { rc[4][24:0], rc[4][31:25] }; rc[5] <= { rc[5][24:0], rc[5][31:25] };
                            rc[6] <= { rc[6][24:0], rc[6][31:25] }; rc[7] <= { rc[7][24:0], rc[7][31:25] };
                            rc[8] <= { rc[8][24:0], rc[8][31:25] }; rc[9] <= { rc[9][24:0], rc[9][31:25] };
                            rc[10] <= { rc[10][24:0], rc[10][31:25] }; rc[11] <= { rc[11][24:0], rc[11][31:25] };
                            rc[12] <= { rc[12][24:0], rc[12][31:25] }; rc[13] <= { rc[13][24:0], rc[13][31:25] };
                            rc[14] <= { rc[14][24:0], rc[14][31:25] }; rc[15] <= { rc[15][24:0], rc[15][31:25] };
                        end
                        7: begin
                            rc[0] <= rc[8]; rc[8] <= rc[0]; rc[1] <= rc[9]; rc[9] <= rc[1];
                            rc[2] <= rc[10]; rc[10] <= rc[2]; rc[3] <= rc[11]; rc[11] <= rc[3];
                            rc[4] <= rc[12]; rc[12] <= rc[4]; rc[5] <= rc[13]; rc[13] <= rc[5];
                            rc[6] <= rc[14]; rc[14] <= rc[6]; rc[7] <= rc[15]; rc[15] <= rc[7];
                        end
                        6: begin
                            rc[0] <= rc[0] ^ rc[16]; rc[1] <= rc[1] ^ rc[17]; rc[2] <= rc[2] ^ rc[18]; rc[3] <= rc[3] ^ rc[19];
                            rc[4] <= rc[4] ^ rc[20]; rc[5] <= rc[5] ^ rc[21]; rc[6] <= rc[6] ^ rc[22]; rc[7] <= rc[7] ^ rc[23];
                            rc[8] <= rc[8] ^ rc[24]; rc[9] <= rc[9] ^ rc[25]; rc[10] <= rc[10] ^ rc[26]; rc[11] <= rc[11] ^ rc[27];
                            rc[12] <= rc[12] ^ rc[28]; rc[13] <= rc[13] ^ rc[29]; rc[14] <= rc[14] ^ rc[30]; rc[15] <= rc[15] ^ rc[31];
                        end
                        5: begin
                            rc[16] <= rc[18]; rc[18] <= rc[16]; rc[17] <= rc[19]; rc[19] <= rc[17];
                            rc[20] <= rc[22]; rc[22] <= rc[20]; rc[21] <= rc[23]; rc[23] <= rc[21];
                            rc[24] <= rc[26]; rc[26] <= rc[24]; rc[25] <= rc[27]; rc[27] <= rc[25];
                            rc[28] <= rc[30]; rc[30] <= rc[28]; rc[29] <= rc[31]; rc[31] <= rc[29];
                        end
                        4: begin
                            rc[16] <= rc[16] + rc[0]; rc[17] <= rc[17] + rc[1]; rc[18] <= rc[18] + rc[2]; rc[19] <= rc[19] + rc[3];
                            rc[20] <= rc[20] + rc[4]; rc[21] <= rc[21] + rc[5]; rc[22] <= rc[22] + rc[6]; rc[23] <= rc[23] + rc[7];
                            rc[24] <= rc[24] + rc[8]; rc[25] <= rc[25] + rc[9]; rc[26] <= rc[26] + rc[10]; rc[27] <= rc[27] + rc[11];
                            rc[28] <= rc[28] + rc[12]; rc[29] <= rc[29] + rc[13]; rc[30] <= rc[30] + rc[14]; rc[31] <= rc[31] + rc[15];
                        end
                        3: begin
                            rc[0] <= { rc[0][20:0], rc[0][31:21] }; rc[1] <= { rc[1][20:0], rc[1][31:21] };
                            rc[2] <= { rc[2][20:0], rc[2][31:21] }; rc[3] <= { rc[3][20:0], rc[3][31:21] };
                            rc[4] <= { rc[4][20:0], rc[4][31:21] }; rc[5] <= { rc[5][20:0], rc[5][31:21] };
                            rc[6] <= { rc[6][20:0], rc[6][31:21] }; rc[7] <= { rc[7][20:0], rc[7][31:21] };
                            rc[8] <= { rc[8][20:0], rc[8][31:21] }; rc[9] <= { rc[9][20:0], rc[9][31:21] };
                            rc[10] <= { rc[10][20:0], rc[10][31:21] }; rc[11] <= { rc[11][20:0], rc[11][31:21] };
                            rc[12] <= { rc[12][20:0], rc[12][31:21] }; rc[13] <= { rc[13][20:0], rc[13][31:21] };
                            rc[14] <= { rc[14][20:0], rc[14][31:21] }; rc[15] <= { rc[15][20:0], rc[15][31:21] };
                        end
                        2: begin
                            rc[0] <= rc[4]; rc[4] <= rc[0]; rc[1] <= rc[5]; rc[5] <= rc[1];
                            rc[2] <= rc[6]; rc[6] <= rc[2]; rc[3] <= rc[7]; rc[7] <= rc[3];
                            rc[8] <= rc[12]; rc[12] <= rc[8]; rc[9] <= rc[13]; rc[13] <= rc[9];
                            rc[10] <= rc[14]; rc[14] <= rc[10]; rc[11] <= rc[15]; rc[15] <= rc[11];
                        end
                        1: begin
                            rc[0] <= rc[0] ^ rc[16]; rc[1] <= rc[1] ^ rc[17]; rc[2] <= rc[2] ^ rc[18]; rc[3] <= rc[3] ^ rc[19];
                            rc[4] <= rc[4] ^ rc[20]; rc[5] <= rc[5] ^ rc[21]; rc[6] <= rc[6] ^ rc[22]; rc[7] <= rc[7] ^ rc[23];
                            rc[8] <= rc[8] ^ rc[24]; rc[9] <= rc[9] ^ rc[25]; rc[10] <= rc[10] ^ rc[26]; rc[11] <= rc[11] ^ rc[27];
                            rc[12] <= rc[12] ^ rc[28]; rc[13] <= rc[13] ^ rc[29]; rc[14] <= rc[14] ^ rc[30]; rc[15] <= rc[15] ^ rc[31];
                        end
                        0: begin
                            rc[16] <= rc[17]; rc[17] <= rc[16]; rc[18] <= rc[19]; rc[19] <= rc[18];
                            rc[20] <= rc[21]; rc[21] <= rc[20]; rc[22] <= rc[23]; rc[23] <= rc[22];
                            rc[24] <= rc[25]; rc[25] <= rc[24]; rc[26] <= rc[27]; rc[27] <= rc[26];
                            rc[28] <= rc[29]; rc[29] <= rc[28]; rc[30] <= rc[31]; rc[31] <= rc[30];
                            CUBEH_CNT <= 9;
                        end
                    endcase
                end
                OP_POLY1305INIT: begin
                    POLY1305INIT_CNT <= POLY1305INIT_CNT - 1;
                    case (POLY1305INIT_CNT)
                        3: begin
                            t0 <= { 32'b0, rc[22] };
                            t1 <= { 32'b0, rc[23] };
                            t2 <= { 32'b0, rc[24] };
                            t3 <= { 32'b0, rc[25] };
                        end
                        2: begin
                            rc[5] <= t0 & 'h3ffffff;
                            t0 <= (t0 >> 26) | (t1 << 6);
                            t1 <= (t1 >> 20) | (t2 << 12);
                            t2 <= (t2 >> 14) | (t3 << 18);
                            t3 <= t3 >> 8;
                        end
                        1: begin
                            rc[6] <= t0 & 'h3ffff03;
                            rc[7] <= t1 & 'h3ffc0ff;
                            rc[8] <= t2 & 'h3f03fff;
                            rc[9] <= t3 & 'h00fffff;
                        end
                        0: begin
                            rc[13] <= rc[6] + rc[6] + rc[6] + rc[6] + rc[6];
                            rc[12] <= rc[7] + rc[7] + rc[7] + rc[7] + rc[7];
                            rc[11] <= rc[8] + rc[8] + rc[8] + rc[8] + rc[8];
                            rc[10] <= rc[9] + rc[9] + rc[9] + rc[9] + rc[9];
                            rc[0] <= 0;
                            rc[1] <= 0;
                            rc[2] <= 0;
                            rc[3] <= 0;
                            rc[4] <= 0;
                            POLY1305INIT_CNT <= 3;
                        end
                    endcase
                end
                OP_POLY1305DONE: begin
                    POLY1305DONE_CNT <= POLY1305DONE_CNT - 1;
                    case (POLY1305DONE_CNT)
                        17: begin
                            rc[0] <= rc[0] & 'h3ffffff;
                            rc[1] <= rc[1] + (rc[0] >> 26);
                        end
                        16: begin
                            rc[1] <= rc[1] & 'h3ffffff;
                            rc[2] <= rc[2] + (rc[1] >> 26);
                        end
                        15: begin
                            rc[2] <= rc[2] & 'h3ffffff;
                            rc[3] <= rc[3] + (rc[2] >> 26);
                        end
                        14: begin
                            rc[3] <= rc[3] & 'h3ffffff;
                            rc[4] <= rc[4] + (rc[3] >> 26);
                        end
                        13: t00 <= rc[4] >> 26;
                        12: begin
                            rc[4] <= rc[4] & 'h3ffffff;
                            rc[0] <= rc[0] + t00 + t00 + t00 + t00 + t00;
                        end
                        11: begin
                            rc[0] <= rc[0] & 'h3ffffff;
                            rc[1] <= rc[1] + (rc[0] >> 26);
                        end
                        10: t00 <= rc[0] + 5;
                        9: begin
                            t00 <= t00 & 'h3ffffff;
                            t01 <= rc[1] + (t00 >> 26);
                        end
                        8: begin
                            t01 <= t01 & 'h3ffffff;
                            t02 <= rc[2] + (t01 >> 26);
                        end
                        7: begin
                            t02 <= t02 & 'h3ffffff;
                            t03 <= rc[3] + (t02 >> 26);
                        end
                        6: begin
                            t03 <= t03 & 'h3ffffff;
                            t04 <= rc[4] + (t03 >> 26) - (1 << 26);
                        end
                        5: tt <= (t04 >> 31) - 1;
                        4: begin
                            rc[0] <= (rc[0] & ~tt) | (t00 & tt);
                            rc[1] <= (rc[1] & ~tt) | (t01 & tt);
                            rc[2] <= (rc[2] & ~tt) | (t02 & tt);
                            rc[3] <= (rc[3] & ~tt) | (t03 & tt);
                            rc[4] <= (rc[4] & ~tt) | (t04 & tt);
                        end
                        3: begin
                            et00 <= {1'b0, (rc[0] | (rc[1] << 26))};
                            et10 <= {1'b0, ((rc[1] >> 6) | (rc[2] << 20))};
                            et20 <= {1'b0, ((rc[2] >> 12) | (rc[3] << 14))};
                            et30 <= {1'b0, ((rc[3] >> 18) | (rc[4] << 8))};
                        end
                        2: begin
                            et0 <= et00 + {1'b0, rc[26]};
                            et1 <= et10 + {1'b0, rc[27]};
                            et2 <= et20 + {1'b0, rc[28]};
                            et3 <= et30 + {1'b0, rc[29]};
                        end
                        1: begin
                            et10 <= et1 + et0[32];
                            et20 <= et2 + et1[32];
                            et30 <= et3 + et2[32];
                        end
                        0: begin
                            rc[0] <= et0[31:0];
                            rc[1] <= et10[31:0];
                            rc[2] <= et20[31:0];
                            rc[3] <= et30[31:0];
                            POLY1305DONE_CNT <= 17;
                        end
                    endcase
                end
                OP_POLY1305MUL: begin
                    POLY1305MUL_CNT <= POLY1305MUL_CNT - 1;
                    case (POLY1305MUL_CNT)
                        19: begin
                            t0 <= { 32'b0, rc[16] };
                            t1 <= { 32'b0, rc[17] };
                            t2 <= { 32'b0, rc[18] };
                            t3 <= { 32'b0, rc[19] };
                            poly_last <= rc[20];
                        end
                        18: begin
                            rc[0] <= rc[0] + { 6'b0, t0[25:0] };
                            rc[1] <= rc[1] + ((((t1 << 32) | t0) >> 26) & 'h3ffffff);
                            rc[2] <= rc[2] + ((((t2 << 32) | t1) >> 20) & 'h3ffffff);
                            rc[3] <= rc[3] + ((((t3 << 32) | t2) >> 14) & 'h3ffffff);
                            rc[4] <= rc[4] + ((t3 >> 8) | (poly_last << 24));
                        end
                        17: begin
                            mul0CE <= 1;
                            mul0A <= rc[0];
                            mul0B <= rc[5];
                            mul1CE <= 1;
                            mul1A <= rc[1];
                            mul1B <= rc[10];
                            mul2CE <= 1;
                            mul2A <= rc[2];
                            mul2B <= rc[11];
                            mul3CE <= 1;
                            mul3A <= rc[3];
                            mul3B <= rc[12];
                            mul4CE <= 1;
                            mul4A <= rc[4];
                            mul4B <= rc[13];
                        end
                        16: begin
                            mul0A <= rc[0];
                            mul0B <= rc[6];
                            mul1A <= rc[1];
                            mul1B <= rc[5];
                            mul2A <= rc[2];
                            mul2B <= rc[10];
                            mul3A <= rc[3];
                            mul3B <= rc[11];
                            mul4A <= rc[4];
                            mul4B <= rc[12];
                        end
                        15: begin
                            mul0A <= rc[0];
                            mul0B <= rc[7];
                            mul1A <= rc[1];
                            mul1B <= rc[6];
                            mul2A <= rc[2];
                            mul2B <= rc[5];
                            mul3A <= rc[3];
                            mul3B <= rc[10];
                            mul4A <= rc[4];
                            mul4B <= rc[11];
                        end
                        14: begin
                            mul0A <= rc[0];
                            mul0B <= rc[8];
                            mul1A <= rc[1];
                            mul1B <= rc[7];
                            mul2A <= rc[2];
                            mul2B <= rc[6];
                            mul3A <= rc[3];
                            mul3B <= rc[5];
                            mul4A <= rc[4];
                            mul4B <= rc[10];
                        end
                        13: begin
                            mul0A <= rc[0];
                            mul0B <= rc[9];
                            mul1A <= rc[1];
                            mul1B <= rc[8];
                            mul2A <= rc[2];
                            mul2B <= rc[7];
                            mul3A <= rc[3];
                            mul3B <= rc[6];
                            mul4A <= rc[4];
                            mul4B <= rc[5];
                        end
                        10: t0 <= mul0P + mul1P + mul2P + mul3P + mul4P;
                        9: t1 <= mul0P + mul1P + mul2P + mul3P + mul4P;
                        8: t2 <= mul0P + mul1P + mul2P + mul3P + mul4P;
                        7: t3 <= mul0P + mul1P + mul2P + mul3P + mul4P;
                        6: begin
                            t4 <= mul0P + mul1P + mul2P + mul3P + mul4P;
                            mul0CE <= 0;
                            mul1CE <= 0;
                            mul2CE <= 0;
                            mul3CE <= 0;
                            mul4CE <= 0;
                        end
                        5: begin
                            rc[0] <= t0 & 'h3ffffff;
                            t1 <= t1 + (t0 >> 26);
                        end
                        4: begin
                            rc[1] <= t1 & 'h3ffffff;
                            t2 <= t2 + { 32'b0, t1[57:26] };
                        end
                        3: begin
                            rc[2] <= t2 & 'h3ffffff;
                            t3 <= t3 + { 32'b0, t2[57:26] };
                        end
                        2: begin
                            rc[3] <= t3 & 'h3ffffff;
                            t4 <= t4 + { 32'b0, t3[57:26] };
                        end
                        1: begin
                            rc[4] <= t4 & 'h3ffffff;
                            t0 <= { 32'b0, t4[57:26] };
                        end
                        0: begin 
                            rc[0] <= rc[0] + t0 + t0 + t0 + t0 + t0;
                            POLY1305MUL_CNT <= 19;
                        end
                    endcase
                end
                default: begin end
            endcase
        end
    end
end

typedef enum {
    DC_INIT, DC_IDLE, DC_CACHE0, DC_CACHE1, DC_CACHE2, DC_PW0, DC_PW1, DC_PW2, DC_MEM0, DC_MEM1, DC_END, DC_INVASID, DC_INVADDR
} dcache_state;
dcache_state dc_st;
reg [8:0]       dcache_cnt;
reg [5:0]       dacc;
wire [31:0]     dPT0_ENT;
wire [31:0]     dPT1_ENT;
reg             d_pfault;
reg             rw;
assign dPT0_ENT = (DATA_ADDR[26:24] == 0) ? DIN[31:0] :
                 (DATA_ADDR[26:24] == 1) ? DIN[63:32] :
                 (DATA_ADDR[26:24] == 2) ? DIN[95:64] :
                 (DATA_ADDR[26:24] == 3) ? DIN[127:96] :
                 (DATA_ADDR[26:24] == 4) ? DIN[159:128] :
                 (DATA_ADDR[26:24] == 5) ? DIN[191:160] :
                 (DATA_ADDR[26:24] == 6) ? DIN[223:192] :
                 (DATA_ADDR[26:24] == 7) ? DIN[255:224] :
                 0;
assign dPT1_ENT = (DATA_ADDR[15:13] == 0) ? DIN[31:0] :
                 (DATA_ADDR[15:13] == 1) ? DIN[63:32] :
                 (DATA_ADDR[15:13] == 2) ? DIN[95:64] :
                 (DATA_ADDR[15:13] == 3) ? DIN[127:96] :
                 (DATA_ADDR[15:13] == 4) ? DIN[159:128] :
                 (DATA_ADDR[15:13] == 5) ? DIN[191:160] :
                 (DATA_ADDR[15:13] == 6) ? DIN[223:192] :
                 (DATA_ADDR[15:13] == 7) ? DIN[255:224] :
                 0;

always @(posedge CLK)
begin
    if (RST == 0) begin
        dcache_cnt <= 0;
        dc_st <= DC_INIT;
        DATA_ST <= 0;
    end else
        case (dc_st)
            DC_INIT: begin
                if (dcache_cnt == 256) begin
                    dcache_cnt <= 0;
                    DC0_WE <= 0;
                    DC1_WE <= 0;
                    DT0_WE <= 0;
                    DT1_WE <= 0;
                    DT2_WE <= 0;
                    DT3_WE <= 0;
                    DT4_WE <= 0;
                    DT5_WE <= 0;
                    dCACHE_INIT <= 0;
                    dc_st <= DC_IDLE;
                end else begin
                    dcache_cnt <= dcache_cnt + 1;
                    dCACHE_INIT_ADDR <= dcache_cnt;
                    if (dcache_cnt == 0) begin
                        dCACHE_INIT <= 1;
                        DC0_IN <= 0;
                        DC0_WE <= 'h1fffffffff;
                        DC1_IN <= 0;
                        DC1_WE <= 'h1fffffffff;
                        DT0_IN <= 0;
                        DT0_WE <= 'h7ff;
                        DT1_IN <= 0;
                        DT1_WE <= 'h7ff;
                        DT2_IN <= 0;
                        DT2_WE <= 'h7ff;
                        DT3_IN <= 0;
                        DT3_WE <= 'h7ff;
                        DT4_IN <= 0;
                        DT4_WE <= 'h7ff;
                        DT5_IN <= 0;
                        DT5_WE <= 'h7ff;
                    end
                end
            end
            DC_IDLE: begin
                DATA_ST <= 0;
                d_pfault <= 0;
                if (DATA_ACT == 1) begin
                    dacc <= (priv_lvl_dc == 1) ? 6'b100000 : 6'b000100;
                    rw <= 0;
                    dc_st <= DC_CACHE0;
                end else if (DATA_ACT == 2) begin
                    dacc <= (priv_lvl_dc == 1) ? 6'b010000 : 6'b000010;
                    rw <= 1;
                    dc_st <= DC_CACHE0;
                end else if (DATA_ACT == 3) begin
                    dc_st <= DC_INVASID;
                    dCACHE_INIT <= 1;
                    dCACHE_INIT_ADDR <= 0;
                    dasid_flush_cnt <= 0;
                end else if (DATA_ACT == 4) begin
                    dc_st <= DC_INVADDR;
                    dCACHE_INIT <= 1;
                    dCACHE_INIT_ADDR <= DATA_ADDR[12:5];
                    daddr_flush_cnt <= 0;
                end
            end
            DC_CACHE0: begin
                dc_st <= DC_CACHE1;
            end
            DC_CACHE1: begin
                dc_st <= DC_CACHE2;
                if ((DATA_ADDR >= 'ha000000000000000) && (DATA_ADDR < 'ha000000100000000)) begin
                    // direct map range
                    // only for kernel-mode when not disabled
                    if ((cr[2][1] == 0) && (priv_lvl_dc == 1)) begin
                        daddr <= DATA_ADDR[31:0];
                    end else begin
                        dc_st <= DC_END;
                        d_pfault <= 1;
                    end
                end else if (DATA_ADDR < 'h8000000000000000) begin
                    // search user TLB
                    if ((DT0_OUT[76] == 1) && (DATA_ADDR[63:13] == DT0_OUT[69:19]) && (DT0_OUT[75:70] & dacc) && (asid == DT0_OUT[85:77])) begin
                        daddr <= {DT0_OUT[18:0], DATA_ADDR[12:0]};
                    end else if ((DT1_OUT[76] == 1) && (DATA_ADDR[63:13] == DT1_OUT[69:19]) && (DT1_OUT[75:70] & dacc) && (asid == DT1_OUT[85:77])) begin
                        daddr <= {DT1_OUT[18:0], DATA_ADDR[12:0]};
                    end else if ((DT2_OUT[76] == 1) && (DATA_ADDR[63:13] == DT2_OUT[69:19]) && (DT2_OUT[75:70] & dacc) && (asid == DT2_OUT[85:77])) begin
                        daddr <= {DT2_OUT[18:0], DATA_ADDR[12:0]};
                    end else if ((DT3_OUT[76] == 1) && (DATA_ADDR[63:13] == DT3_OUT[69:19]) && (DT3_OUT[75:70] & dacc) && (asid == DT3_OUT[85:77])) begin
                        daddr <= {DT3_OUT[18:0], DATA_ADDR[12:0]};
                    end else begin
                        dc_st <= DC_PW0;
                    end
                end else begin
                    if ((DT4_OUT[76] == 1) && (DATA_ADDR[63:13] == DT4_OUT[69:19]) && (DT4_OUT[75:70] & dacc)) begin
                        daddr <= {DT4_OUT[18:0], DATA_ADDR[12:0]};
                    end else if ((DT5_OUT[76] == 1) && (DATA_ADDR[63:13] == DT5_OUT[69:19]) && (DT5_OUT[75:70] & dacc)) begin
                        daddr <= {DT5_OUT[18:0], DATA_ADDR[12:0]};
                    end else begin
                        dc_st <= DC_PW0;
                    end
                end
            end
            DC_CACHE2: begin
                DT0_WE <= 0;
                DT1_WE <= 0;
                DT2_WE <= 0;
                DT3_WE <= 0;
                DT4_WE <= 0;
                DT5_WE <= 0;
                addr_dc <= daddr[31:5];
                if (rw == 0)
                    dc_st <= DC_END;
                else begin
                    dc_st <= DC_MEM1;
                    act_dc <= 2;
                    case (daddr[4:0])
                        0: begin DOUT[63:0] <= DATA_DOUT; MASK <= DMASK; end
                        1: begin DOUT[71:8] <= DATA_DOUT; MASK <= (DMASK << 1); end
                        2: begin DOUT[79:16] <= DATA_DOUT; MASK <= (DMASK << 2); end
                        3: begin DOUT[87:24] <= DATA_DOUT; MASK <= (DMASK << 3); end
                        4: begin DOUT[95:32] <= DATA_DOUT; MASK <= (DMASK << 4); end
                        5: begin DOUT[103:40] <= DATA_DOUT; MASK <= (DMASK << 5); end
                        6: begin DOUT[111:48] <= DATA_DOUT; MASK <= (DMASK << 6); end
                        7: begin DOUT[119:56] <= DATA_DOUT; MASK <= (DMASK << 7); end
                        8: begin DOUT[127:64] <= DATA_DOUT; MASK <= (DMASK << 8); end
                        9: begin DOUT[135:72] <= DATA_DOUT; MASK <= (DMASK << 9); end
                        10: begin DOUT[143:80] <= DATA_DOUT; MASK <= (DMASK << 10); end
                        11: begin DOUT[151:88] <= DATA_DOUT; MASK <= (DMASK << 11); end
                        12: begin DOUT[159:96] <= DATA_DOUT; MASK <= (DMASK << 12); end
                        13: begin DOUT[167:104] <= DATA_DOUT; MASK <= (DMASK << 13); end
                        14: begin DOUT[175:112] <= DATA_DOUT; MASK <= (DMASK << 14); end
                        15: begin DOUT[183:120] <= DATA_DOUT; MASK <= (DMASK << 15); end
                        16: begin DOUT[191:128] <= DATA_DOUT; MASK <= (DMASK << 16); end
                        17: begin DOUT[199:136] <= DATA_DOUT; MASK <= (DMASK << 17); end
                        18: begin DOUT[207:144] <= DATA_DOUT; MASK <= (DMASK << 18); end
                        19: begin DOUT[215:152] <= DATA_DOUT; MASK <= (DMASK << 19); end
                        20: begin DOUT[223:160] <= DATA_DOUT; MASK <= (DMASK << 20); end
                        21: begin DOUT[231:168] <= DATA_DOUT; MASK <= (DMASK << 21); end
                        22: begin DOUT[239:176] <= DATA_DOUT; MASK <= (DMASK << 22); end
                        23: begin DOUT[247:184] <= DATA_DOUT; MASK <= (DMASK << 23); end
                        24: begin DOUT[255:192] <= DATA_DOUT; MASK <= (DMASK << 24); end
                        25: begin DOUT[255:200] <= DATA_DOUT; MASK <= (DMASK << 25); end
                        26: begin DOUT[255:208] <= DATA_DOUT; MASK <= (DMASK << 26); end
                        27: begin DOUT[255:216] <= DATA_DOUT; MASK <= (DMASK << 27); end
                        28: begin DOUT[255:224] <= DATA_DOUT; MASK <= (DMASK << 28); end
                        29: begin DOUT[255:232] <= DATA_DOUT; MASK <= (DMASK << 29); end
                        30: begin DOUT[255:240] <= DATA_DOUT; MASK <= (DMASK << 30); end
                        31: begin DOUT[255:248] <= DATA_DOUT; MASK <= (DMASK << 31); end
                    endcase
                end
                if ((DC0_OUT[283] == 1) && (daddr[31:5] == DC0_OUT[282:256]))
                    DC_MATCH <= 0;
                else if ((DC1_OUT[283] == 1) && (daddr[31:5] == DC1_OUT[282:256]))
                    DC_MATCH <= 1;
                else begin
                    DC_MATCH <= 3;
                    if (rw == 0) begin
                        dc_st <= DC_MEM0;
                        act_dc <= 1;
                    end
                end
            end
            DC_PW0: begin
                dc_st <= DC_PW1;
                if (DATA_ADDR >= 'hfffffff800000000) begin
                    addr_dc <= {2'b0, pt1[29:13], DATA_ADDR[34:27]};
                    act_dc <= 1;
                end else if ((DATA_ADDR >= {1'b0, pt0_off}) && (DATA_ADDR < ({1'b0, pt0_off} + 'h800000000))) begin
                    addr_dc <= {2'b0, pt0[29:13], DATA_ADDR[34:27]};
                    act_dc <= 1;
                end else begin
                    d_pfault <= 1;
                    dc_st <= DC_END;
                end
            end
            DC_PW1: begin
                act_dc <= 0;
                if (BUS == 1) begin
                    if (dPT0_ENT[12] == 1) begin
                        addr_dc <= {2'b0, dPT0_ENT[29:13], DATA_ADDR[23:16]};
                        act_dc <= 1;
                        dc_st <= DC_PW2;
                    end else begin
                        d_pfault <= 1;
                        dc_st <= DC_END;
                    end
                end
            end
            DC_PW2: begin
                act_dc <= 0;
                if (BUS == 1) begin
                    if ((dPT1_ENT[12] == 1) && (dPT1_ENT[5:0] & dacc)) begin
                        daddr <= {dPT1_ENT[31:13], DATA_ADDR[12:0]};
                        dc_st <= DC_CACHE2;
                        if (DATA_ADDR < 'h8000000000000000) begin
                            case (DT0_OUT[87:86])
                                0: begin DT0_WE <= 'h7ff; DT0_IN <= {2'b01, asid, 1'b1, dPT1_ENT[5:0], DATA_ADDR[63:13], dPT1_ENT[31:13]}; end
                                1: begin DT1_WE <= 'h7ff; DT1_IN <= {2'b00, asid, 1'b1, dPT1_ENT[5:0], DATA_ADDR[63:13], dPT1_ENT[31:13]};
                                         DT0_WE <= 'h400; DT0_IN[87:80] <= {2'b10, DT0_OUT[85:80]};
                                end
                                2: begin DT2_WE <= 'h7ff; DT2_IN <= {2'b00, asid, 1'b1, dPT1_ENT[5:0], DATA_ADDR[63:13], dPT1_ENT[31:13]};
                                         DT0_WE <= 'h400; DT0_IN[87:80] <= {2'b11, DT0_OUT[85:80]};
                                end
                                3: begin DT3_WE <= 'h7ff; DT3_IN <= {2'b00, asid, 1'b1, dPT1_ENT[5:0], DATA_ADDR[63:13], dPT1_ENT[31:13]};
                                         DT0_WE <= 'h400; DT0_IN[87:80] <= {2'b00, DT0_OUT[85:80]};
                                end
                            endcase
                        end else begin
                            case (DT4_OUT[87:86])
                                0: begin DT4_WE <= 'h7ff; DT4_IN <= {2'b01, 9'b00, 1'b1, dPT1_ENT[5:0], DATA_ADDR[63:13], dPT1_ENT[31:13]}; end
                                default: begin DT5_WE <= 'h7ff; DT5_IN <= {2'b00, 9'b00, 1'b1, dPT1_ENT[5:0], DATA_ADDR[63:13], dPT1_ENT[31:13]};
                                         DT4_WE <= 'h400; DT4_IN[87:80] <= {2'b00, DT4_OUT[85:80]};
                                end
                            endcase
                        end
                    end else begin
                        d_pfault <= 1;
                        dc_st <= DC_END;
                    end
                end
            end
            DC_MEM0: begin
                act_dc <= 0;
                if (BUS == 1) begin
                    dc_st <= DC_END;
                    if (daddr < 'h40000000) // cache only if from RAM, need a way to not hardcode this
                        case (DC0_OUT[289:288])
                            0: begin DC0_WE <= 'h1fffffffff; DC0_IN <= {2'b01, 4'b0, 1'b1, daddr[31:5], DIN}; end
                            1: begin DC1_WE <= 'h1fffffffff; DC1_IN <= {2'b00, 4'b0, 1'b1, daddr[31:5], DIN};
                                     DC0_WE <= 'h1000000000; DC0_IN[289:288] <= 2'b00;
                            end
                        endcase
                end
            end
            DC_MEM1: begin
                act_dc <= 0;
                if (BUS == 1) begin
                    dc_st <= DC_END;
                    case (DC_MATCH)
                        0: begin DC0_WE[31:0] <= MASK; DC0_IN[255:0] <= DOUT; end
                        1: begin DC1_WE[31:0] <= MASK; DC1_IN[255:0] <= DOUT; end
                        default: begin end
                    endcase
                end
            end
            DC_END: begin
                DC0_WE <= 0;
                DC1_WE <= 0;
                if (d_pfault == 1)
                    DATA_ST <= 2;
                else
                    DATA_ST <= 1;
                if (rw == 0) begin
                    if (DC_MATCH != 3)  // found in cache
                        case (daddr[4:0])
                            0: DATA_DIN <= DC_DAT[63:0];
                            1: DATA_DIN <= DC_DAT[71:8];
                            2: DATA_DIN <= DC_DAT[79:16];
                            3: DATA_DIN <= DC_DAT[87:24];
                            4: DATA_DIN <= DC_DAT[95:32];
                            5: DATA_DIN <= DC_DAT[103:40];
                            6: DATA_DIN <= DC_DAT[111:48];
                            7: DATA_DIN <= DC_DAT[119:56];
                            8: DATA_DIN <= DC_DAT[127:64];
                            9: DATA_DIN <= DC_DAT[135:72];
                            10: DATA_DIN <= DC_DAT[143:80];
                            11: DATA_DIN <= DC_DAT[151:88];
                            12: DATA_DIN <= DC_DAT[159:96];
                            13: DATA_DIN <= DC_DAT[167:104];
                            14: DATA_DIN <= DC_DAT[175:112];
                            15: DATA_DIN <= DC_DAT[183:120];
                            16: DATA_DIN <= DC_DAT[191:128];
                            17: DATA_DIN <= DC_DAT[199:136];
                            18: DATA_DIN <= DC_DAT[207:144];
                            19: DATA_DIN <= DC_DAT[215:152];
                            20: DATA_DIN <= DC_DAT[223:160];
                            21: DATA_DIN <= DC_DAT[231:168];
                            22: DATA_DIN <= DC_DAT[239:176];
                            23: DATA_DIN <= DC_DAT[247:184];
                            24: DATA_DIN <= DC_DAT[255:192];
                            25: DATA_DIN <= DC_DAT[255:200];
                            26: DATA_DIN <= DC_DAT[255:208];
                            27: DATA_DIN <= DC_DAT[255:216];
                            28: DATA_DIN <= DC_DAT[255:224];
                            29: DATA_DIN <= DC_DAT[255:232];
                            30: DATA_DIN <= DC_DAT[255:240];
                            31: DATA_DIN <= DC_DAT[255:248];
                        endcase
                    else
                        case (daddr[4:0])
                            0: DATA_DIN <= DIN[63:0];
                            1: DATA_DIN <= DIN[71:8];
                            2: DATA_DIN <= DIN[79:16];
                            3: DATA_DIN <= DIN[87:24];
                            4: DATA_DIN <= DIN[95:32];
                            5: DATA_DIN <= DIN[103:40];
                            6: DATA_DIN <= DIN[111:48];
                            7: DATA_DIN <= DIN[119:56];
                            8: DATA_DIN <= DIN[127:64];
                            9: DATA_DIN <= DIN[135:72];
                            10: DATA_DIN <= DIN[143:80];
                            11: DATA_DIN <= DIN[151:88];
                            12: DATA_DIN <= DIN[159:96];
                            13: DATA_DIN <= DIN[167:104];
                            14: DATA_DIN <= DIN[175:112];
                            15: DATA_DIN <= DIN[183:120];
                            16: DATA_DIN <= DIN[191:128];
                            17: DATA_DIN <= DIN[199:136];
                            18: DATA_DIN <= DIN[207:144];
                            19: DATA_DIN <= DIN[215:152];
                            20: DATA_DIN <= DIN[223:160];
                            21: DATA_DIN <= DIN[231:168];
                            22: DATA_DIN <= DIN[239:176];
                            23: DATA_DIN <= DIN[247:184];
                            24: DATA_DIN <= DIN[255:192];
                            25: DATA_DIN <= DIN[255:200];
                            26: DATA_DIN <= DIN[255:208];
                            27: DATA_DIN <= DIN[255:216];
                            28: DATA_DIN <= DIN[255:224];
                            29: DATA_DIN <= DIN[255:232];
                            30: DATA_DIN <= DIN[255:240];
                            31: DATA_DIN <= DIN[255:248];
                        endcase
                end
                dc_st <= DC_IDLE;
            end
            DC_INVASID: begin
                DT0_WE <= 0;
                DT1_WE <= 0;
                DT2_WE <= 0;
                DT3_WE <= 0;
                if (dCACHE_INIT_ADDR == 256) begin
                    dCACHE_INIT <= 0;
                    dc_st <= DC_IDLE;
                    DATA_ST <= 1;
                end else begin
                    if (dasid_flush_cnt == 4) begin
                        dasid_flush_cnt <= 0;
                        dCACHE_INIT_ADDR <= dCACHE_INIT_ADDR + 1;
                    end else if (dasid_flush_cnt == 3) begin
                        if (DT0_OUT[85:77] == DATA_ADDR[8:0]) begin DT0_WE <= 'h7ff; DT0_IN <= 0; end
                        if (DT1_OUT[85:77] == DATA_ADDR[8:0]) begin DT1_WE <= 'h7ff; DT1_IN <= 0; end
                        if (DT2_OUT[85:77] == DATA_ADDR[8:0]) begin DT2_WE <= 'h7ff; DT2_IN <= 0; end
                        if (DT3_OUT[85:77] == DATA_ADDR[8:0]) begin DT3_WE <= 'h7ff; DT3_IN <= 0; end
                        dasid_flush_cnt <= 4;
                    end else
                        dasid_flush_cnt <= dasid_flush_cnt + 1;
                end
            end
            DC_INVADDR: begin
                daddr_flush_cnt <= daddr_flush_cnt + 1;
                if (daddr_flush_cnt == 4) begin
                    DT0_WE <= 0;
                    DT1_WE <= 0;
                    DT2_WE <= 0;
                    DT3_WE <= 0;
                    DT4_WE <= 0;
                    DT5_WE <= 0;
                    dCACHE_INIT <= 0;
                    dc_st <= DC_IDLE;
                    DATA_ST <= 1;
                end else if (daddr_flush_cnt == 3) begin
                    if (DT0_OUT[69:19] == DATA_ADDR[63:13]) begin DT0_WE <= 'h7ff; DT0_IN <= 0; end
                    if (DT1_OUT[69:19] == DATA_ADDR[63:13]) begin DT1_WE <= 'h7ff; DT1_IN <= 0; end
                    if (DT2_OUT[69:19] == DATA_ADDR[63:13]) begin DT2_WE <= 'h7ff; DT2_IN <= 0; end
                    if (DT3_OUT[69:19] == DATA_ADDR[63:13]) begin DT3_WE <= 'h7ff; DT3_IN <= 0; end
                    if (DT4_OUT[69:19] == DATA_ADDR[63:13]) begin DT4_WE <= 'h7ff; DT4_IN <= 0; end
                    if (DT5_OUT[69:19] == DATA_ADDR[63:13]) begin DT5_WE <= 'h7ff; DT5_IN <= 0; end
                end
            end
        endcase
end

endmodule
