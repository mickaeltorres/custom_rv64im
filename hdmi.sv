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

module HDMI(
    input               CLK200,
    input               CLK108,
    input               RST,
    output reg          CLK,
    output reg          SPDIF_OUT,
    output reg          VSYNC,
    output reg          HSYNC,
    output reg          DE,
    output reg [15:0]   DATA,
    input               INT,
    input               SPDIF_IN,
    output reg          SCL,
    output reg          SDA_OUT,
    input               SDA_IN,
    output reg          FB_WE,
    output reg [13:0]   FB_WADDR,
    output reg [15:0]   FB_WDAT,
    output reg [13:0]   FB_RADDR,
    input [15:0]        FB_RDAT,
    output reg [10:0]   FONT_ADDR,
    input [7:0]         FONT_DATA,
    output reg          RD,
    input               RD_EMPTY,
    input [7:0]         RD_DAT
);

assign CLK = CLK108;

(* KEEP="TRUE" *)reg         rst;
(* KEEP="TRUE" *)reg [1:0]   rst_sync;
(* KEEP="TRUE" *)reg         rst_in;

always @(posedge CLK200)
begin
    rst_sync[1] <= RST;
    rst_in <= RST;
end

always @(posedge CLK108)
begin
    rst_sync[0] <= rst_sync[1];
    rst <= rst_sync[0];
end

reg [7:0] hdmi_init_dev = 'h72; // 0x39 + 0 (WR)
`define HDMI_REG_MAX 17
reg [7:0] hdmi_init_reg[`HDMI_REG_MAX] = '{ 'h41, 'h98, 'h9a, 'h9c, 'h9d, 'ha2, 'ha3, 'he0, 'hf9, 'h15, 'h16, 'h17, 'h18, 'haf, 'h40, 'h48, 'h55 };
reg [7:0] hdmi_init_dat[`HDMI_REG_MAX] = '{ 'h10, 'h03, 'he0, 'h30, 'h01, 'ha4, 'ha4, 'hd0, 'h00, 'h01, 'h38, 'h00, 'hC6, 'h04, 'h00, 'h08, 'h00 };
reg [7:0] hdmi_reg_cnt;
reg [7:0] hdmi_bit_cnt;

reg [31:0]  init_cnt;
reg         init_done;

reg [23:0]  pal[18];

always @(posedge CLK108)
begin
    if (!rst) begin
        SPDIF_OUT <= 0;
        SCL <= 1;
        SDA_OUT <= 1;
        init_cnt <= 0;
        init_done <= 0;
    end else begin
        if (!init_done)
            case (init_cnt)
                0: begin SDA_OUT <= 1; SCL <= 1; hdmi_reg_cnt <= 0; init_cnt <= init_cnt + 1; end
                // START BIT
                12201000: begin SDA_OUT <= 0; init_cnt <= init_cnt + 1; end
                12201500: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                // ADDRESS
                12201999: begin hdmi_bit_cnt <= 8; init_cnt <= init_cnt + 1; end
                12202000: begin SDA_OUT <= hdmi_init_dev[hdmi_bit_cnt-1]; hdmi_bit_cnt <= hdmi_bit_cnt - 1; init_cnt <= init_cnt + 1; end
                12202500: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12203000: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                12203499: begin if (hdmi_bit_cnt == 0) init_cnt <= init_cnt + 1; else init_cnt <= 12202000; end
                // ACK
                12203500: begin SDA_OUT <= 1; init_cnt <= init_cnt + 1; end
                12204000: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12204500: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                // REG
                12204999: begin hdmi_bit_cnt <= 8; init_cnt <= init_cnt + 1; end
                12205000: begin SDA_OUT <= hdmi_init_reg[hdmi_reg_cnt][hdmi_bit_cnt-1]; hdmi_bit_cnt <= hdmi_bit_cnt - 1; init_cnt <= init_cnt + 1; end
                12205500: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12206000: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                12206499: begin if (hdmi_bit_cnt == 0) init_cnt <= init_cnt + 1; else init_cnt <= 12205000; end
                // ACK
                12206500: begin SDA_OUT <= 1; init_cnt <= init_cnt + 1; end
                12207000: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12207500: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                // DATA
                12207999: begin hdmi_bit_cnt <= 8; init_cnt <= init_cnt + 1; end
                12208000: begin SDA_OUT <= hdmi_init_dat[hdmi_reg_cnt][hdmi_bit_cnt-1]; hdmi_bit_cnt <= hdmi_bit_cnt - 1; init_cnt <= init_cnt + 1; end
                12208500: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12209000: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                12209499: begin if (hdmi_bit_cnt == 0) init_cnt <= init_cnt + 1; else init_cnt <= 12208000; end
                // ACK
                12209500: begin SDA_OUT <= 1; init_cnt <= init_cnt + 1; end
                12210000: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12210500: begin SCL <= 0; init_cnt <= init_cnt + 1; end
                // STOP
                12211000: begin SDA_OUT <= 0; init_cnt <= init_cnt + 1; end
                12211500: begin SCL <= 1; init_cnt <= init_cnt + 1; end
                12212000: begin SDA_OUT <= 1; init_cnt <= init_cnt + 1; end
                // LOOP
                12300000: begin
                    hdmi_reg_cnt <= hdmi_reg_cnt + 1;
                    if (hdmi_reg_cnt == (`HDMI_REG_MAX - 1)) begin
                        init_done <= 1;
                        init_cnt <= 0;
                    end else
                        init_cnt <= 12201000;
                end
                default: init_cnt <= init_cnt + 1;
            endcase
    end
end

// quick RGB to YCbCr conversion
reg [15:0]  hsync_cnt;
reg [15:0]  vsync_cnt;
reg [7:0]   R;
reg [7:0]   G;
reg [7:0]   B;
wire [7:0]  Y;
wire [7:0]  Cb;
wire [7:0]  Cr;
assign Y = (((R << 6) + (R << 1) + (G << 7) + G + (B << 4) + (B << 3) + B) >> 8) + 16;
assign Cb = ((-((R << 5) + (R << 2) + (R << 1)) - ((G << 6) + (G << 3) + (G << 1)) + (B << 7) - (B << 4)) >> 8) + 128;
assign Cr = (((R << 7) - (R << 4) - ((G << 6) + (G << 5) - (G << 1)) - ((B << 4) + (B << 1))) >> 8) + 128;

always @(posedge CLK108)
begin
    if (!init_done) begin
        hsync_cnt <= 0;
        vsync_cnt <= 0;
        HSYNC <= 0;
        VSYNC <= 0;
        DE <= 0;
        DATA <= 0;
    end else begin
        if (hsync_cnt < 1687)
            hsync_cnt <= hsync_cnt + 1;
        else begin
            hsync_cnt <= 0;
            if (vsync_cnt < 1065)
                vsync_cnt <= vsync_cnt + 1;
            else
                vsync_cnt <= 0;
        end
        // HSync
        if (hsync_cnt == 1687)
            HSYNC <= 1;
        else if (hsync_cnt == 110)
            HSYNC <= 0;
        // VSync
        if (vsync_cnt == 1065)
            VSYNC <= 1;
        else if (vsync_cnt == 2)
            VSYNC <= 0;
        // Vactive + Hactive
        if ((vsync_cnt > 40) && (vsync_cnt < 1065) && (hsync_cnt > 359) && (hsync_cnt < 1640)) begin
            if (!hsync_cnt[0])
                DATA <= { Cb[7:0], Y[7:0] };
            else
                DATA <= { Cr[7:0], Y[7:0] }; 
            DE <= 1;
        end else begin
            DATA <= 0;
            DE <= 0;
        end
    end
end

reg [15:0]  cnt_x;
reg [15:0]  cnt_y;
reg [15:0]  calc_y;
reg [15:0]  calc_x;
reg [23:0]  col_fg;
reg [23:0]  col_bg;
reg [7:0]   cur_char;
reg [7:0]   cur_x;
reg [5:0]   cur_y;
reg [7:0]   cur_x1;
reg [5:0]   cur_y1;
reg [5:0]   sc_st;
reg [5:0]   sc_st1;
reg         cur_on;
reg [31:0]  cur_cnt;
reg [2:0]   cnt_x8;

always @(posedge CLK108)
begin
    if (cur_cnt < 53999999)
        cur_cnt <= cur_cnt + 1;
    else begin
        cur_cnt <= 0;
        cur_on <= !cur_on;
    end
end

always @(posedge CLK108)
begin
    if ((vsync_cnt > 40) && (vsync_cnt < 1065)) begin
        if ((hsync_cnt > 358) && (hsync_cnt < 1639)) begin
            R <= (cur_char[cnt_x8] == 1) ? col_fg[23:16] : col_bg[23:16];
            G <= (cur_char[cnt_x8] == 1) ? col_fg[15:8] : col_bg[15:8];
            B <= (cur_char[cnt_x8] == 1) ? col_fg[7:0] : col_bg[7:0];
            cnt_x8 <= cnt_x8 + 1;
        end
        if ((hsync_cnt > 350) && (hsync_cnt < 1631)) begin
            if (cnt_x == 1279) begin
                cnt_y <= cnt_y + 1;
                cnt_x <= 0;
            end else
                cnt_x <= cnt_x + 1;
            if (cnt_x[2:0] == 0) begin
                if (((cnt_y >> 4) + sc_st) > 63)
                    calc_y <= ((cnt_y >> 4) + sc_st) - 64;
                else
                    calc_y <= (cnt_y >> 4) + sc_st;
                calc_x <= (cnt_x >> 3);
            end else if (cnt_x[2:0] == 1) begin
                FB_RADDR <= (calc_y << 7) + (calc_y << 5) + calc_x;
            end else if (cnt_x[2:0] == 4) begin
                FONT_ADDR <= { FB_RDAT[6:0], cnt_y[3:0] };
            end else if (cnt_x[2:0] == 7) begin
                cur_char <= FONT_DATA;
                if ((calc_x == cur_x) && (calc_y == cur_y) && (cur_on == 1)) begin
                    col_bg <= pal[16];
                    col_fg <= pal[17];
                end else begin
                    col_bg <= pal[FB_RDAT[15:12]];
                    col_fg <= pal[FB_RDAT[11:8]];
                end
            end
        end
    end else begin
        cnt_x <= 0;
        cnt_y <= 0;
        cnt_x8 <= 0;
    end
end

reg [7:0]   chr;
reg [15:0]  in_cnt;
reg [15:0]  in_addr;
reg [15:0]  cl_addr;
(* KEEP="TRUE" *)reg [7:0]   col;
reg [5:0]   sc_start;
reg [7:0]   x;
reg [5:0]   y;
reg [5:0]   yp1;
reg [7:0]   cl_n;
reg [7:0]   esc_buf1;
reg [7:0]   esc_buf2;
reg [4:0]   esc_wait;

typedef enum
{
    ST_INIT, ST_CLEAR, ST_IDLE, ST_RD1, ST_RD2, ST_RUN, ST_CLLN, ST_CLS, ST_SGR
} state_t;
state_t state;

initial
begin
    in_cnt <= 0;
    state <= ST_INIT;
end

always @(posedge CLK200)
begin
    case (state)
        ST_INIT: begin
            col <= 'h4f;
            in_cnt <= 0;
            state <= ST_CLEAR;
            pal[0] <= 0;
            pal[1] <= 'h330000;
            pal[2] <= 'h003300;
            pal[3] <= 'h333300;
            pal[4] <= 'h000033;
            pal[5] <= 'h330033;
            pal[6] <= 'h003333;
            pal[7] <= 'he7e7e7;
            pal[8] <= 'h7e7e7e;
            pal[9] <= 'hff0000;
            pal[10] <= 'h00ff00;
            pal[11] <= 'hffff00;
            pal[12] <= 'h5c5cff;
            pal[13] <= 'hff00ff;
            pal[14] <= 'h00ffff;
            pal[15] <= 'hffffff;
            pal[16] <= 'hff8000;
            pal[17] <= 0;
        end
        ST_CLEAR: begin
            esc_wait <= 0;
            x <= 0;
            y <= 0;
            yp1 <= 1;
            sc_start <= 0;
            if (in_cnt == 10240) begin
                FB_WE <= 0;
                state <= ST_IDLE;
            end else begin
                FB_WE <= 1;
                FB_WADDR <= in_cnt;
                FB_WDAT <= { col, 8'h00 };
                in_cnt <= in_cnt + 1;
            end
        end
        ST_IDLE: begin
            if (!rst_in) begin
                state <= ST_INIT;
            end else begin
                FB_WE <= 0;
                if (!RD_EMPTY)
                begin
                    RD <= 1;
                    state <= ST_RD1;
                end
            end
        end
        ST_RD1: begin
            RD <= 0;
            state <= ST_RD2;
        end
        ST_RD2: begin
            chr <= RD_DAT;
            state <= ST_RUN;
        end
        ST_RUN: begin
            if (!rst_in) begin
                state <= ST_INIT;
            end else begin
                state <= ST_IDLE;
                if (esc_wait == 1) begin
                    if (chr == 91) // [
                        esc_wait <= 2;
                    else
                        esc_wait <= 0;
                end else if (esc_wait == 2) begin
                    esc_buf1 <= chr;
                    esc_wait <= 3;
                end else if (esc_wait == 3) begin
                    esc_wait <= 4;
                    esc_buf2 <= chr;
                    if (chr == 74) begin // J
                        esc_wait <= 0;
                        state <= ST_CLS;
                    end else if (chr == 109) begin // m
                        esc_wait <= 0;
                        state <= ST_SGR;
                    end else if ((chr == 48) && (esc_buf1 == 49))
                        esc_wait <= 3;
                end else if (esc_wait == 4) begin
                    esc_wait <= 0;
                    if (chr == 74) // J
                        state <= ST_CLS;
                    else if (chr == 109) // m
                        state <= ST_SGR;
                end else begin  
                    if (chr == 27) begin // esc
                        esc_wait <= 1;
                    end else if (chr == 13) // \r
                        x <= 0;
                    else if (chr == 10) begin // \n
                        if (yp1 == sc_start) begin
                            sc_start <= sc_start + 1;
                            cl_n <= 0;
                            state <= ST_CLLN;
                        end
                        y <= y + 1;
                        yp1 <= yp1 + 1;
                    end else if ((chr > 31) && (chr < 128)) begin
                        FB_WE <= 1;
                        FB_WADDR <= (y << 7) + (y << 5) + x;
                        FB_WDAT <= { col, chr };
                        if (x == 159) begin
                            x <= 0;
                            if (yp1 == sc_start) begin
                                sc_start <= sc_start + 1;
                                cl_n <= 0;
                                state <= ST_CLLN;
                            end
                            y <= y + 1;
                            yp1 <= yp1 + 1;
                        end else
                            x <= x + 1;
                    end
                end
            end
        end
        ST_CLLN: begin
            if (cl_n == 160) begin
                state <= ST_IDLE;
                FB_WE <= 0;
            end else begin
                FB_WE <= 1;
                FB_WADDR <= (y << 7) + (y << 5) + cl_n;
                FB_WDAT <= { col, 8'h00 };
                cl_n <= cl_n + 1;
            end
        end
        ST_CLS: begin
            in_cnt <= 0;
            state <= ST_CLEAR;
        end
        ST_SGR: begin
            if (esc_buf1 == 51) // 3
                col[3:0] <= esc_buf2 - 48;
            else if (esc_buf1 == 52) // 4
                col[7:4] <= esc_buf2 - 48;
            else if (esc_buf1 == 57) // 9
                col[3:0] <= esc_buf2 - 40;
            else if (esc_buf1 == 49) // 1
                col[7:4] <= esc_buf2 - 40;
            state <= ST_IDLE;
        end
    endcase
end

always @(posedge CLK108)
begin
    cur_x1 <= x;
    cur_y1 <= y;
    cur_x <= cur_x1;
    cur_y <= cur_y1;
    sc_st1 <= sc_start;
    sc_st <= sc_st1;
end

endmodule
