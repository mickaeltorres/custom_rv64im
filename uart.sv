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

module UART(
    input               CLK,
    input               RST,
    input               rx,
    output reg          tx,
    output reg          rd,
    input               rdempty,
    input [7:0]         idat,
    output reg [7:0]    odat,
    output reg          odat_en
);

reg [7:0]   snd;
reg [7:0]   rcv;
reg [3:0]   cnts;
reg [3:0]   cntr;
reg [7:0]   rcvd;

initial
begin
    snd <= 0;
    rcv <= 0;
    tx <= 1;
    rcvd <= 0;
end

always @(posedge CLK)
begin
    if (RST == 0)
    begin
        rcv <= 0;
        rcvd <= 0;
    end else case (rcv)
        0: begin
            odat_en <= 0;
            if (rx == 0)
            begin
                cntr <= 15;
                rcv <= 1;
            end
        end
        100: begin
            rcv <= rcv + 1;
            if (cntr == 15)
                cntr <= 0;
            else if (cntr == 8)
            begin
                rcv <= 0;
                odat <= rcvd;
                odat_en <= 1;
            end
            else
                begin
                    rcvd[cntr] <= rx;
                    cntr <= cntr + 1;
                end
        end
        200: begin
            rcv <= 1;
        end
        default: begin
            rcv <= rcv + 1;
        end
    endcase
end

always @(posedge CLK)
begin
    if (RST == 0)
    begin
        snd <= 0;
        tx <= 1;
    end else case (snd)
        0: begin
            if (rdempty == 0)
            begin
                rd <= 1;
                cnts <= 15;
                tx <= 0;
                snd <= 1; 
            end
        end
        1: begin
            rd <= 0;
            snd <= snd + 1;
            if (cnts == 15)
                cnts <= 0;
            else if (cnts == 8)
            begin
                tx <= 1;
                cnts <= cnts + 1;
            end
            else if (cnts == 9)
                snd <= 0;
            else
            begin
                cnts <= cnts + 1;
                tx <= idat[cnts];
            end
        end
        200: begin
            snd <= 1;
        end
        default: begin
            snd <= snd + 1;
        end
    endcase
end

endmodule
