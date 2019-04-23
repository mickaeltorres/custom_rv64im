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

module RNG(
    input               CLK,
    input               RST,
    input               RND_IN,
    output reg [7:0]    OUT,
    output reg          WE,
    input               FULL
);

reg [7:0]           RND_DAT;
reg [2:0]           RND_SHIFT;
reg                 RND_CNT;
reg [15:0]          start;
reg                 tog;
reg                 rd;
reg                 rd_ok;
reg                 init;
reg [63:0]          l0_0;
reg [64:0]          l0_1;
reg [65:0]          l0_2;
reg [63:0]          l1_0;
reg [64:0]          l1_1;
reg [65:0]          l1_2;
reg [63:0]          l2_0;
reg [64:0]          l2_1;
reg [65:0]          l2_2;
reg [63:0]          l3_0;
reg [64:0]          l3_1;
reg [65:0]          l3_2;
reg [63:0]          l4_0;
reg [64:0]          l4_1;
reg [65:0]          l4_2;
reg [63:0]          l5_0;
reg [64:0]          l5_1;
reg [65:0]          l5_2;
reg [63:0]          l6_0;
reg [64:0]          l6_1;
reg [65:0]          l6_2;
reg [63:0]          l7_0;
reg [64:0]          l7_1;
reg [65:0]          l7_2;
reg [15:0]          i_cnt;
reg [3:0]           upd;

initial
begin
    RND_SHIFT = 0;
    start = 0;
    tog = 0;
    rd = 0;
    rd_ok = 0;
    init = 0;
    i_cnt = 0;
    upd = 0;
end

always @(posedge CLK)
begin
    if (!RST) begin
        rd_ok <= 0;
        i_cnt <= 0;
        init <= 0;
        upd <= 0;
    end else begin
        if (init == 0) begin
            rd_ok <= 0;
            if (i_cnt == 208)
                init = 1;
            else if (rd == 1) begin
                rd_ok <= 1;
                i_cnt <= i_cnt + 1;
                case (i_cnt)
                    // L0
                    0:   l0_0[7:0]   <= RND_DAT;
                    1:   l0_0[15:8]  <= RND_DAT;
                    2:   l0_0[23:16] <= RND_DAT;
                    3:   l0_0[31:24] <= RND_DAT;
                    4:   l0_0[39:32] <= RND_DAT;
                    5:   l0_0[47:40] <= RND_DAT;
                    6:   l0_0[55:48] <= RND_DAT;
                    7:   l0_0[63:56] <= RND_DAT;
                    8:   l0_1[7:0]   <= RND_DAT;
                    9:   l0_1[15:8]  <= RND_DAT;
                    10:  l0_1[23:16] <= RND_DAT;
                    11:  l0_1[31:24] <= RND_DAT;
                    12:  l0_1[39:32] <= RND_DAT;
                    13:  l0_1[47:40] <= RND_DAT;
                    14:  l0_1[55:48] <= RND_DAT;
                    15:  l0_1[63:56] <= RND_DAT;
                    16:  l0_1[64]    <= RND_DAT[3];
                    17:  l0_2[7:0]   <= RND_DAT;
                    18:  l0_2[15:8]  <= RND_DAT;
                    19:  l0_2[23:16] <= RND_DAT;
                    20:  l0_2[31:24] <= RND_DAT;
                    21:  l0_2[39:32] <= RND_DAT;
                    22:  l0_2[47:40] <= RND_DAT;
                    23:  l0_2[55:48] <= RND_DAT;
                    24:  l0_2[63:56] <= RND_DAT;
                    25:  l0_2[65:64] <= RND_DAT[5:4];
                    // L1
                    26:  l1_0[7:0]   <= RND_DAT;
                    27:  l1_0[15:8]  <= RND_DAT;
                    28:  l1_0[23:16] <= RND_DAT;
                    29:  l1_0[31:24] <= RND_DAT;
                    30:  l1_0[39:32] <= RND_DAT;
                    31:  l1_0[47:40] <= RND_DAT;
                    32:  l1_0[55:48] <= RND_DAT;
                    33:  l1_0[63:56] <= RND_DAT;
                    34:  l1_1[7:0]   <= RND_DAT;
                    35:  l1_1[15:8]  <= RND_DAT;
                    36:  l1_1[23:16] <= RND_DAT;
                    37:  l1_1[31:24] <= RND_DAT;
                    38:  l1_1[39:32] <= RND_DAT;
                    39:  l1_1[47:40] <= RND_DAT;
                    40:  l1_1[55:48] <= RND_DAT;
                    41:  l1_1[63:56] <= RND_DAT;
                    42:  l1_1[64]    <= RND_DAT[1];
                    43:  l1_2[7:0]   <= RND_DAT;
                    44:  l1_2[15:8]  <= RND_DAT;
                    45:  l1_2[23:16] <= RND_DAT;
                    46:  l1_2[31:24] <= RND_DAT;
                    47:  l1_2[39:32] <= RND_DAT;
                    48:  l1_2[47:40] <= RND_DAT;
                    49:  l1_2[55:48] <= RND_DAT;
                    50:  l1_2[63:56] <= RND_DAT;
                    51:  l1_2[65:64] <= RND_DAT[7:6];
                    // L2
                    52:  l2_0[7:0]   <= RND_DAT;
                    53:  l2_0[15:8]  <= RND_DAT;
                    54:  l2_0[23:16] <= RND_DAT;
                    55:  l2_0[31:24] <= RND_DAT;
                    56:  l2_0[39:32] <= RND_DAT;
                    57:  l2_0[47:40] <= RND_DAT;
                    58:  l2_0[55:48] <= RND_DAT;
                    59:  l2_0[63:56] <= RND_DAT;
                    60:  l2_1[7:0]   <= RND_DAT;
                    61:  l2_1[15:8]  <= RND_DAT;
                    62:  l2_1[23:16] <= RND_DAT;
                    63:  l2_1[31:24] <= RND_DAT;
                    64:  l2_1[39:32] <= RND_DAT;
                    65:  l2_1[47:40] <= RND_DAT;
                    66:  l2_1[55:48] <= RND_DAT;
                    67:  l2_1[63:56] <= RND_DAT;
                    68:  l2_1[64]    <= RND_DAT[0];
                    69:  l2_2[7:0]   <= RND_DAT;
                    70:  l2_2[15:8]  <= RND_DAT;
                    71:  l2_2[23:16] <= RND_DAT;
                    72:  l2_2[31:24] <= RND_DAT;
                    73:  l2_2[39:32] <= RND_DAT;
                    74:  l2_2[47:40] <= RND_DAT;
                    75:  l2_2[55:48] <= RND_DAT;
                    76:  l2_2[63:56] <= RND_DAT;
                    77:  l2_2[65:64] <= RND_DAT[3:2];
                    // L3
                    78:  l3_0[7:0]   <= RND_DAT;
                    79:  l3_0[15:8]  <= RND_DAT;
                    80:  l3_0[23:16] <= RND_DAT;
                    81:  l3_0[31:24] <= RND_DAT;
                    82:  l3_0[39:32] <= RND_DAT;
                    83:  l3_0[47:40] <= RND_DAT;
                    84:  l3_0[55:48] <= RND_DAT;
                    85:  l3_0[63:56] <= RND_DAT;
                    86:  l3_1[7:0]   <= RND_DAT;
                    87:  l3_1[15:8]  <= RND_DAT;
                    88:  l3_1[23:16] <= RND_DAT;
                    89:  l3_1[31:24] <= RND_DAT;
                    90:  l3_1[39:32] <= RND_DAT;
                    91:  l3_1[47:40] <= RND_DAT;
                    92:  l3_1[55:48] <= RND_DAT;
                    93:  l3_1[63:56] <= RND_DAT;
                    94:  l3_1[64]    <= RND_DAT[7];
                    95:  l3_2[7:0]   <= RND_DAT;
                    96:  l3_2[15:8]  <= RND_DAT;
                    97:  l3_2[23:16] <= RND_DAT;
                    98:  l3_2[31:24] <= RND_DAT;
                    99:  l3_2[39:32] <= RND_DAT;
                    100: l3_2[47:40] <= RND_DAT;
                    101: l3_2[55:48] <= RND_DAT;
                    102: l3_2[63:56] <= RND_DAT;
                    103: l3_2[65:64] <= RND_DAT[1:0];
                    // L4
                    104:  l4_0[7:0]   <= RND_DAT;
                    105: l4_0[15:8]  <= RND_DAT;
                    106: l4_0[23:16] <= RND_DAT;
                    107: l4_0[31:24] <= RND_DAT;
                    108: l4_0[39:32] <= RND_DAT;
                    109: l4_0[47:40] <= RND_DAT;
                    110: l4_0[55:48] <= RND_DAT;
                    111: l4_0[63:56] <= RND_DAT;
                    112: l4_1[7:0]   <= RND_DAT;
                    113: l4_1[15:8]  <= RND_DAT;
                    114: l4_1[23:16] <= RND_DAT;
                    115: l4_1[31:24] <= RND_DAT;
                    116: l4_1[39:32] <= RND_DAT;
                    117: l4_1[47:40] <= RND_DAT;
                    118: l4_1[55:48] <= RND_DAT;
                    119: l4_1[63:56] <= RND_DAT;
                    120: l4_1[64]    <= RND_DAT[3];
                    121: l4_2[7:0]   <= RND_DAT;
                    122: l4_2[15:8]  <= RND_DAT;
                    123: l4_2[23:16] <= RND_DAT;
                    124: l4_2[31:24] <= RND_DAT;
                    125: l4_2[39:32] <= RND_DAT;
                    126: l4_2[47:40] <= RND_DAT;
                    127: l4_2[55:48] <= RND_DAT;
                    128: l4_2[63:56] <= RND_DAT;
                    129: l4_2[65:64] <= RND_DAT[5:4];
                    // L5
                    130: l5_0[7:0]   <= RND_DAT;
                    131: l5_0[15:8]  <= RND_DAT;
                    132: l5_0[23:16] <= RND_DAT;
                    133: l5_0[31:24] <= RND_DAT;
                    134: l5_0[39:32] <= RND_DAT;
                    135: l5_0[47:40] <= RND_DAT;
                    136: l5_0[55:48] <= RND_DAT;
                    137: l5_0[63:56] <= RND_DAT;
                    138: l5_1[7:0]   <= RND_DAT;
                    139: l5_1[15:8]  <= RND_DAT;
                    140: l5_1[23:16] <= RND_DAT;
                    141: l5_1[31:24] <= RND_DAT;
                    142: l5_1[39:32] <= RND_DAT;
                    143: l5_1[47:40] <= RND_DAT;
                    144: l5_1[55:48] <= RND_DAT;
                    145: l5_1[63:56] <= RND_DAT;
                    146: l5_1[64]    <= RND_DAT[1];
                    147: l5_2[7:0]   <= RND_DAT;
                    148: l5_2[15:8]  <= RND_DAT;
                    149: l5_2[23:16] <= RND_DAT;
                    150: l5_2[31:24] <= RND_DAT;
                    151: l5_2[39:32] <= RND_DAT;
                    152: l5_2[47:40] <= RND_DAT;
                    153: l5_2[55:48] <= RND_DAT;
                    154: l5_2[63:56] <= RND_DAT;
                    155: l5_2[65:64] <= RND_DAT[7:6];
                    // L6
                    156: l6_0[7:0]   <= RND_DAT;
                    157: l6_0[15:8]  <= RND_DAT;
                    158: l6_0[23:16] <= RND_DAT;
                    159: l6_0[31:24] <= RND_DAT;
                    160: l6_0[39:32] <= RND_DAT;
                    161: l6_0[47:40] <= RND_DAT;
                    162: l6_0[55:48] <= RND_DAT;
                    163: l6_0[63:56] <= RND_DAT;
                    164: l6_1[7:0]   <= RND_DAT;
                    165: l6_1[15:8]  <= RND_DAT;
                    166: l6_1[23:16] <= RND_DAT;
                    167: l6_1[31:24] <= RND_DAT;
                    168: l6_1[39:32] <= RND_DAT;
                    169: l6_1[47:40] <= RND_DAT;
                    170: l6_1[55:48] <= RND_DAT;
                    171: l6_1[63:56] <= RND_DAT;
                    172: l6_1[64]    <= RND_DAT[0];
                    173: l6_2[7:0]   <= RND_DAT;
                    174: l6_2[15:8]  <= RND_DAT;
                    175: l6_2[23:16] <= RND_DAT;
                    176: l6_2[31:24] <= RND_DAT;
                    177: l6_2[39:32] <= RND_DAT;
                    178: l6_2[47:40] <= RND_DAT;
                    179: l6_2[55:48] <= RND_DAT;
                    180: l6_2[63:56] <= RND_DAT;
                    181: l6_2[65:64] <= RND_DAT[3:2];
                    // L7
                    182: l7_0[7:0]   <= RND_DAT;
                    183: l7_0[15:8]  <= RND_DAT;
                    184: l7_0[23:16] <= RND_DAT;
                    185: l7_0[31:24] <= RND_DAT;
                    186: l7_0[39:32] <= RND_DAT;
                    187: l7_0[47:40] <= RND_DAT;
                    188: l7_0[55:48] <= RND_DAT;
                    189: l7_0[63:56] <= RND_DAT;
                    190: l7_1[7:0]   <= RND_DAT;
                    191: l7_1[15:8]  <= RND_DAT;
                    192: l7_1[23:16] <= RND_DAT;
                    193: l7_1[31:24] <= RND_DAT;
                    194: l7_1[39:32] <= RND_DAT;
                    195: l7_1[47:40] <= RND_DAT;
                    196: l7_1[55:48] <= RND_DAT;
                    197: l7_1[63:56] <= RND_DAT;
                    198: l7_1[64]    <= RND_DAT[7];
                    199: l7_2[7:0]   <= RND_DAT;
                    200: l7_2[15:8]  <= RND_DAT;
                    201: l7_2[23:16] <= RND_DAT;
                    202: l7_2[31:24] <= RND_DAT;
                    203: l7_2[39:32] <= RND_DAT;
                    204: l7_2[47:40] <= RND_DAT;
                    205: l7_2[55:48] <= RND_DAT;
                    206: l7_2[63:56] <= RND_DAT;
                    207: l7_2[65:64] <= RND_DAT[1:0];
                endcase
            end
        end else begin
            rd_ok <= 0;
            WE <= 0;
            if (FULL == 0) begin
                WE <= 1;
                OUT <= { l7_0[0] ^ l7_1[0], l6_0[0] ^ l6_1[0], l5_0[0] ^ l5_1[0], l4_0[0] ^ l4_1[0], l3_0[0] ^ l3_1[0], l2_0[0] ^ l2_1[0], l1_0[0] ^ l1_1[0], l0_0[0] ^ l0_1[0]};
                l7_2 <= (l7_2 >> 1) | ((l7_2[0] ^ l7_2[1] ^ l7_2[9] ^ l7_2[10]) << 65);
                l6_2 <= (l6_2 >> 1) | ((l6_2[0] ^ l6_2[1] ^ l6_2[9] ^ l6_2[10]) << 65);
                l5_2 <= (l5_2 >> 1) | ((l5_2[0] ^ l5_2[1] ^ l5_2[9] ^ l5_2[10]) << 65);
                l4_2 <= (l4_2 >> 1) | ((l4_2[0] ^ l4_2[1] ^ l4_2[9] ^ l4_2[10]) << 65);
                l3_2 <= (l3_2 >> 1) | ((l3_2[0] ^ l3_2[1] ^ l3_2[9] ^ l3_2[10]) << 65);
                l2_2 <= (l2_2 >> 1) | ((l2_2[0] ^ l2_2[1] ^ l2_2[9] ^ l2_2[10]) << 65);
                l1_2 <= (l1_2 >> 1) | ((l1_2[0] ^ l1_2[1] ^ l1_2[9] ^ l1_2[10]) << 65);
                l0_2 <= (l0_2 >> 1) | ((l0_2[0] ^ l0_2[1] ^ l0_2[9] ^ l0_2[10]) << 65);
                if (l7_2[0] == 0)
                    l7_0 <= (l7_0 >> 1) | ((l7_0[0] ^ l7_0[1] ^ l7_0[3] ^ l7_0[4]) << 63);
                else
                    l7_1 <= (l7_1 >> 1) | ((l7_1[0] ^ l7_1[18]) << 64);
                if (l6_2[0] == 0)
                    l6_0 <= (l6_0 >> 1) | ((l6_0[0] ^ l6_0[1] ^ l6_0[3] ^ l6_0[4]) << 63);
                else
                    l6_1 <= (l6_1 >> 1) | ((l6_1[0] ^ l6_1[18]) << 64);
                if (l5_2[0] == 0)
                    l5_0 <= (l5_0 >> 1) | ((l5_0[0] ^ l5_0[1] ^ l5_0[3] ^ l5_0[4]) << 63);
                else
                    l5_1 <= (l5_1 >> 1) | ((l5_1[0] ^ l5_1[18]) << 64);
                if (l4_2[0] == 0)
                    l4_0 <= (l4_0 >> 1) | ((l4_0[0] ^ l4_0[1] ^ l4_0[3] ^ l4_0[4]) << 63);
                else
                    l4_1 <= (l4_1 >> 1) | ((l4_1[0] ^ l4_1[18]) << 64);
                if (l3_2[0] == 0)
                    l3_0 <= (l3_0 >> 1) | ((l3_0[0] ^ l3_0[1] ^ l3_0[3] ^ l3_0[4]) << 63);
                else
                    l3_1 <= (l3_1 >> 1) | ((l3_1[0] ^ l3_1[18]) << 64);
                if (l2_2[0] == 0)
                    l2_0 <= (l2_0 >> 1) | ((l2_0[0] ^ l2_0[1] ^ l2_0[3] ^ l2_0[4]) << 63);
                else
                    l2_1 <= (l2_1 >> 1) | ((l2_1[0] ^ l2_1[18]) << 64);
                if (l1_2[0] == 0)
                    l1_0 <= (l1_0 >> 1) | ((l1_0[0] ^ l1_0[1] ^ l1_0[3] ^ l1_0[4]) << 63);
                else
                    l1_1 <= (l1_1 >> 1) | ((l1_1[0] ^ l1_1[18]) << 64);
                if (l0_2[0] == 0)
                    l0_0 <= (l0_0 >> 1) | ((l0_0[0] ^ l0_0[1] ^ l0_0[3] ^ l0_0[4]) << 63);
                else
                    l0_1 <= (l0_1 >> 1) | ((l0_1[0] ^ l0_1[18]) << 64);
            end else begin
                if (rd == 1) begin
                    rd_ok <= 1;
                    upd <= upd + 1;
                    case (upd)
                        0:  l0_0[9:2] <= RND_DAT;
                        1:  l1_0[9:2] <= RND_DAT;
                        2:  l2_0[9:2] <= RND_DAT;
                        3:  l3_0[9:2] <= RND_DAT;
                        4:  l4_0[9:2] <= RND_DAT;
                        5:  l5_0[9:2] <= RND_DAT;
                        6:  l6_0[9:2] <= RND_DAT;
                        7:  l7_0[9:2] <= RND_DAT;
                        8:  l0_1[9:2] <= RND_DAT;
                        9:  l1_1[9:2] <= RND_DAT;
                        10: l2_1[9:2] <= RND_DAT;
                        11: l3_1[9:2] <= RND_DAT;
                        12: l4_1[9:2] <= RND_DAT;
                        13: l5_1[9:2] <= RND_DAT;
                        14: l6_1[9:2] <= RND_DAT;
                        15: l7_1[9:2] <= RND_DAT;
                    endcase
                end
            end
        end
    end
end

always @(posedge CLK)
begin
    if (!RST) begin
        RND_SHIFT <= 0;
        start <= 0;
        tog <= 0;
        rd <= 0;
    end else begin
        RND_CNT <= !RND_CNT;
        if (rd_ok)
            rd <= 0;
        if (RND_IN == 0) begin
            tog <= 0;
        end else if (tog == 0) begin
            tog <= 1;
            RND_SHIFT <= RND_SHIFT + 1;
            RND_DAT[RND_SHIFT] <= RND_DAT[RND_SHIFT] ^ RND_CNT;
            if (RND_SHIFT == 0) begin
                if (start < 1024)
                    start <= start + 1;
                else begin
                    rd <= 1;
                end
            end
        end
    end
end

endmodule
