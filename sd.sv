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

module SDCTRL(
    input               CLK,
    input               RST,
    output reg          SD_CS,
    output reg          SD_MOSI,
    output reg          SD_SCK,
    input               SD_MISO,
    output reg          SDCMDRE,
    input [39:0]        SDCMDOUT,
    input               SDCMDEMPTY,
    output reg [7:0]    SDDATIN,
    output reg          SDDATWE,
    input               SDDATFULL
);

reg [15:0]  step;
reg [39:0]  cmd;
reg [47:0]  cmdrd;
reg [7:0]   dat;
reg [15:0]  cnt;
reg [15:0]  wait_init;
reg [24:0]  retry;
reg [7:0]   err;
reg [47:0]  CMD0 =  'h400000000095;
reg [47:0]  CMD8 =  'h48000001aa87;
reg [47:0]  CMD9 =  'h49;
reg [47:0]  CMD10 = 'h4a;
reg [47:0]  CMD55 = 'h7700000000aa;
reg [47:0]  CMD41 = 'h6940000000aa;
reg [47:0]  CMD58 = 'h7a00000000aa;
reg [7:0]   CMD17 = 'h51;
reg [7:0]   CMD24 = 'h58;

always @(posedge CLK)
begin
    if (RST == 0) begin
        SD_CS <= 1;
        SD_SCK <= 0;
        SD_MOSI <= 1;
        SDCMDRE <= 0;
        SDDATWE <= 0;
        step <= 0;
        err <= 0;
    end else case (step)
        0: begin
            SD_SCK <= 0;
            SD_CS <= 1;
            SD_MOSI <= 1;
            err <= 0;
            if (SDCMDEMPTY != 1) begin
                SDCMDRE <= 1;
                step <= 1;
            end
        end
        1: begin
            SDCMDRE <= 0;
            step <= 2;
        end
        2: begin
            cmd <= SDCMDOUT;
            step <= 3;
        end
        3: begin
            case (cmd[7:0])
                0:          step <= 100;    // INIT
                1:          begin           // READ SECTOR
                    step <= 10000;
                    SD_CS <= 0;
                    cmdrd[47:40] <= CMD17;
                    cmdrd[39:8] <= cmd[39:8];
                    cmdrd[7:0] <= 'hff;
                end
                2:          begin           // WRITE SECTOR
                    step <= 20000;
                    SD_CS <= 0;
                    cmdrd[47:40] <= CMD24;
                    cmdrd[39:8] <= cmd[39:8];
                    cmdrd[7:0] <= 'hff;
                end
                3:          begin           // READ CSD
                    step <= 10000;
                    SD_CS <= 0;
                    cmdrd[47:40] <= CMD9;
                    cmdrd[39:8] <= 0;
                    cmdrd[7:0] <= 'hff;
                end
                4:          begin           // READ CID
                    step <= 10000;
                    SD_CS <= 0;
                    cmdrd[47:40] <= CMD10;
                    cmdrd[39:8] <= 0;
                    cmdrd[7:0] <= 'hff;
                end
                default:    step <= 0;
            endcase
        end
        
        // INIT
        // first some un-selected clocks
        100: begin
            cnt <= 159;
            step <= 101;
            SD_SCK <= 1;
            err <= 'hff;
        end
        150: begin
            cnt <= cnt - 1;
            if (cnt == 0) begin
                step <= 151;
                SD_CS <= 0;
            end else begin
                step <= 101;
                SD_SCK <= !SD_SCK;
            end
        end
        
        // then some selected clocks
        200: begin
            cnt <= 639;
            step <= 201;
            SD_SCK <= 1;
            SD_MOSI <= 1;
        end
        250: begin
            cnt <= cnt - 1;
            if (cnt == 0)
                step <= 300;
            else begin
                step <= 201;
                SD_SCK <= !SD_SCK;
            end
        end
        
        // send CMD0
        300: begin
            cnt <= 47;
            step <= 301;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= CMD0[47];
        end
        350: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 400;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= CMD0[cnt - 1];
                end
                step <= 301;
                SD_SCK <= !SD_SCK;
            end
        end

        // expect 0x01
        400: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 401;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        401: begin
            step <= 402;
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
        end
        450: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 1) begin
                    step <= 500;
                end else begin
                    err <= 1;
                    if (retry != 0) begin
                        step <= 400;
                    end else
                        step <= 65000;
                end
            end else begin
                step <= 401;
                SD_SCK <= !SD_SCK;
            end
        end
        
        // send CMD8
        500: begin
            cnt <= 47;
            step <= 501;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= CMD8[47];
        end
        550: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 600;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= CMD8[cnt - 1];
                end
                step <= 501;
                SD_SCK <= !SD_SCK;
            end
        end
            
        // expect 0x01
        600: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 601;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        601: begin
            step <= 602;
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
        end
        650: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 1) begin
                    step <= 651;
                    SD_SCK <= 1;
                    cnt <= 95;
                end else begin
                    err <= 2;
                    if (retry != 0) begin
                        step <= 600;
                    end else
                        step <= 65000;
                end
            end else begin
                step <= 601;
                SD_SCK <= !SD_SCK;
            end
        end

        // read 8 bytes
        699: begin
            cnt <= cnt - 1;
            if (cnt == 0) begin
                step <= 700;
                wait_init <= 2048;
            end else begin
                step <= 651;
                SD_SCK <= !SD_SCK;
            end
        end

        // send CMD55
        700: begin
            wait_init <= wait_init - 1;
            cnt <= 47;
            step <= 701;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= CMD55[47];
        end
        750: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 800;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= CMD55[cnt - 1];
                end
                step <= 701;
                SD_SCK <= !SD_SCK;
            end
        end

        // expect 0x01
        800: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 801;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        801: begin
            step <= 802;
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
        end
        850: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 1) begin
                    step <= 900;
                end else begin
                    err <= 3;
                    if (retry != 0) begin
                        step <= 800;
                    end else
                        step <= 65000;
                end
            end else begin
                step <= 801;
                SD_SCK <= !SD_SCK;
            end
        end

        // send CMD41
        900: begin
            cnt <= 47;
            step <= 901;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= CMD41[47];
        end
        950: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 1000;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= CMD41[cnt - 1];
                end
                step <= 901;
                SD_SCK <= !SD_SCK;
            end
        end
            
        // expect 0x00
        1000: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 1001;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        1001: begin
            step <= 1002;
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
        end
        1050: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 0) begin
                    step <= 1051;
                    SD_SCK <= 1;
                    cnt <= 95;
                end else begin
                    err <= 4;
                    if (retry != 0) begin
                        step <= 1000;
                    end else begin
                        if (wait_init == 0)
                            step <= 65000;
                        else
                            step <= 700;
                    end
                end
            end else begin
                step <= 1001;
                SD_SCK <= !SD_SCK;
            end
        end

        // read 8 bytes
        1099: begin
            cnt <= cnt - 1;
            if (cnt == 0)
                step <= 1100;
            else begin
                step <= 1051;
                SD_SCK <= !SD_SCK;
            end
        end

        // send CMD58
        1100: begin
            cnt <= 47;
            step <= 1101;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= CMD58[47];
        end
        1150: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 1200;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= CMD58[cnt - 1];
                end
                step <= 1101;
                SD_SCK <= !SD_SCK;
            end
        end
            
        // expect 0x00
        1200: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 1201;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        1201: begin
            step <= 1202;
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
        end
        1250: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 0) begin
                    step <= 1251;
                    SD_SCK <= 1;
                    cnt <= 95;
                end else begin
                    err <= 5;
                    if (retry != 0) begin
                        step <= 1200;
                    end else
                        step <= 65000;
                end
            end else begin
                step <= 1201;
                SD_SCK <= !SD_SCK;
            end
        end

        // read 8 bytes
        1299: begin
            cnt <= cnt - 1;
            if (cnt == 0)
                step <= 64000;
            else begin
                step <= 1251;
                SD_SCK <= !SD_SCK;
            end
        end

        // READ sector
        // send CMD17
        10000: begin
            cnt <= 47;
            step <= 10001;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= cmdrd[47];
        end
        10001: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 10010;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= cmdrd[cnt - 1];
                end
                SD_SCK <= !SD_SCK;
            end
        end

        // expect 0x00
        10010: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 10011;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        10011: begin
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 0) begin
                    step <= 10020;
                    retry <= 'hffff;
                end else begin
                    err <= 1;
                    if (retry != 0) begin
                        step <= 10010;
                    end else
                        step <= 65000;
                end
            end else begin
                SD_SCK <= !SD_SCK;
            end
        end

        // expect 0xfe
        10020: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 10021;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        10021: begin
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 'hfe) begin
                    step <= 10030;
                    retry <= 512;
                    SDDATWE <= 1;
                    SDDATIN <= 0;
                end else begin
                    err <= 2;
                    if (retry != 0) begin
                        step <= 10020;
                    end else
                        step <= 65000;
                end
            end else begin
                SD_SCK <= !SD_SCK;
            end
        end
        
        // read data
        10030: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 10031;
            SD_SCK <= 1;
            SD_CS <= 0;
            SDDATWE <= 0;
        end
        10031: begin
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
            if ((cnt == 0) && (SD_SCK == 0)) begin
                SDDATWE <= 1;
                SDDATIN <= dat;
                if (retry != 0) begin
                    step <= 10030;
                end else
                    step <= 10040;
            end else begin
                SD_SCK <= !SD_SCK;
            end
        end

        // SEND some selected clocks
        10040: begin
            SDDATWE <= 0;
            cnt <= 95;
            step <= 10041;
            SD_SCK <= 1;
            SD_MOSI <= 1;
        end
        10041: begin
            cnt <= cnt - 1;
            if (cnt == 0)
                step <= 64000;
            else begin
                SD_SCK <= !SD_SCK;
            end
        end

        // WRITE sector
        // send CMD24
        20000: begin
            cnt <= 47;
            step <= 20001;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= cmdrd[47];
        end
        20001: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                step <= 20010;
                retry <= 16;
                SD_MOSI <= 1;
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= cmdrd[cnt - 1];
                end
                SD_SCK <= !SD_SCK;
            end
        end

        // expect 0x00
        20010: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 20011;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        20011: begin
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 0) begin
                    step <= 20025;
                    retry <= 512;
                    cmdrd[7:0] <= 'hfe;
                    SDDATIN <= 'h0;
                    SDDATWE <= 1;
                end else begin
                    err <= 1;
                    if (retry != 0) begin
                        step <= 20010;
                    end else
                        step <= 65000;
                end
            end else begin
                SD_SCK <= !SD_SCK;
            end
        end

        // send 0xfe then data
        20020: begin
            if (SDCMDEMPTY != 1) begin
                SDCMDRE <= 1;
                step <= 20021;
            end
        end
        20021: begin
            SDCMDRE <= 0;
            step <= 20022;
        end
        20022: begin
            cmdrd[7:0] <= SDCMDOUT;
            step <= 20025;
        end
        20025: begin
            cnt <= 7;
            step <= 20026;
            SD_SCK <= 1;
            SD_CS <= 0;
            SD_MOSI <= cmdrd[7];
            SDDATWE <= 0;
        end
        20026: begin
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (retry == 0) begin
                    retry <= 16;
                    step <= 20030;
                    SD_MOSI <= 1;
                end else begin
                    step <= 20020;
                    retry <= retry - 1;
                end
            end else begin
                if (SD_SCK == 0) begin
                    cnt <= cnt - 1;
                    SD_MOSI <= cmdrd[cnt - 1];
                end
                SD_SCK <= !SD_SCK;
            end
        end

        // expect 0xe5
        20030: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 20031;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        20031: begin
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat == 'he5) begin
                    retry <= 1310720;
                    step <= 20040;
                end else begin
                    err <= 2;
                    if (retry != 0) begin
                        step <= 20030;
                    end else
                        step <= 65000;
                end
            end else begin
                SD_SCK <= !SD_SCK;
            end
        end
        
        // expect !0x00
        20040: begin
            retry <= retry - 1;
            cnt <= 8;
            step <= 20041;
            SD_SCK <= 1;
            SD_CS <= 0;
        end
        20041: begin
            if (SD_SCK == 1) begin
                dat[cnt - 1] <= SD_MISO;
                cnt <= cnt - 1;
            end
            if ((cnt == 0) && (SD_SCK == 0)) begin
                if (dat != 0) begin
                    step <= 64000;
                end else begin
                    err <= 3;
                    if (retry != 0) begin
                        step <= 20040;
                    end else
                        step <= 65000;
                end
            end else begin
                SD_SCK <= !SD_SCK;
            end
        end

        // send OK
        64000: begin
            SDDATIN <= 'h0;
            SDDATWE <= 1;
            step <= 64001;
            SD_SCK <= 1;
            SD_MOSI <= 1;
            cnt <= 64;
        end
        64001: begin
            SDDATWE <= 0;
            SD_SCK <= !SD_SCK;
            cnt <= cnt - 1;
            if (cnt == 1) begin
                SD_SCK <= 0;
                SD_CS <= 1;
                step <= 0;
            end
        end
        
        // send ERR
        65000: begin
            SDDATIN <= err;
            SDDATWE <= 1;
            step <= 64001;
            SD_SCK <= 1;
            SD_MOSI <= 1;
            cnt <= 64;
        end

        default: step <= step + 1;
    endcase
end

endmodule
