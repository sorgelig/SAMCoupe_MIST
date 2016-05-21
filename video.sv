//
//
// Sam Coupe Video Controller implementation
// 
// Copyright (c) 2016 Sorgelig
//
// 
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 
//

`timescale 1ns / 1ps

module video
(
	input         reset,

	input         clk_sys,	// master clock
	input         ce_6mp,
	input         ce_6mn,
	input         ce_24m,

	// CPU interfacing
	input  [15:0] addr,
	input   [7:0] din,
	output  [7:0] dout,
	output        dout_en,
	input         port_we,

	output        mem_contention,
	output        io_contention,

	output reg    INT_line,
	output reg    INT_frame,

	// VRAM interfacing
	output [18:0] vram_addr1,
	output [18:0] vram_addr2,
	input  [15:0] vram_dout1,
	input  [15:0] vram_dout2,

	// Misc. signals
	input   [3:0] border_color,
	input         scandoubler_disable,
	input         soff,
	output  [1:0] video_mode,
	input   [1:0] mode3_hi,

	// OSD IO interface
	input         SPI_SCK,
	input         SPI_SS3,
	input         SPI_DI,

	// Video outputs
	output  [5:0] VGA_R,
	output  [5:0] VGA_G,
	output  [5:0] VGA_B,
	output        VGA_VS,
	output        VGA_HS
);

wire  [2:0] cpu_slot  = 5;
assign io_contention  = hc[2:0] != cpu_slot;
assign mem_contention = (fetch | (!mode & hc[6])) ? hc[2:0] != cpu_slot : hc[1:0] != cpu_slot[1:0];

assign vram_addr1 = vaddr1;
assign vram_addr2 = vaddr2;

wire  [5:0] VGA_Rs, VGA_Rd;
wire  [5:0] VGA_Gs, VGA_Gd;
wire  [5:0] VGA_Bs, VGA_Bd;
wire        hsyncd, vsyncd;

osd #(10'd10, 10'd0, 3'd4) osd
(
	.*,
	.ce_pix(ce_6mp | ce_6mn),
	.VGA_Rx({2{R, I}}),
	.VGA_Gx({2{G, I}}),
	.VGA_Bx({2{B, I}}),
	.VGA_R(VGA_Rs),
	.VGA_G(VGA_Gs),
	.VGA_B(VGA_Bs),
	.OSD_HS(HSync),
	.OSD_VS(VSync)
);

scandoubler scandoubler
(
	.clk_sys(clk_sys),
	.ce_x2(ce_24m),
	.ce_x1(ce_6mp | ce_6mn),

	.scanlines(0),

	.hs_in(HSync),
	.vs_in(VSync),
	.r_in(VGA_Rs),
	.g_in(VGA_Gs),
	.b_in(VGA_Bs),

	.hs_out(hsyncd),
	.vs_out(vsyncd),
	.r_out(VGA_Rd),
	.g_out(VGA_Gd),
	.b_out(VGA_Bd)
);

assign {VGA_R,  VGA_G,  VGA_B,  VGA_VS,  VGA_HS          } = scandoubler_disable ?
       {VGA_Rs, VGA_Gs, VGA_Bs, 1'b1,    ~(HSync ^ VSync)} :
       {VGA_Rd, VGA_Gd, VGA_Bd, ~vsyncd, ~hsyncd         };

reg        HBlank;
reg        HSync;
reg        VBlank;
reg        VSync;

reg  [7:0] attr;
reg [31:0] shift;
reg [18:0] vaddr1;
reg [18:0] vaddr2;

reg  [4:0] flashcnt;
reg        paper, fetch;

reg  [8:0] hc  = 0;
reg  [8:0] vc  = 0;
wire [4:0] col = {~hc[7], hc[6:3]};

wire [7:0] lpen;
reg  [7:0] hpen;
reg  [3:0] border;

always @(posedge clk_sys) begin

	if(ce_6mp) begin
		if (hc==383) begin
			hc <= 0;
			if (vc == 311) begin 
				vc <= 0;
				flashcnt <= flashcnt + 1'd1;
			end else begin
				vc <= vc + 1'd1;
			end
		end else begin
			hc <= hc + 1'd1;
		end
		if(mode == 2) shift <= shift << 2;
	end
	if(ce_6mn) begin
		if(hc == 28)  HBlank <= 1;
		if(hc == 44)  HSync  <= 1;
		if(hc == 76)  HSync  <= 0;
		if(hc == 108) HBlank <= 0;

		if((vc == 236) & (hc == 28))  VBlank <= 1;
		if( vc == 240) VSync <= 1;
		if( vc == 244) VSync <= 0;
		if((vc == 260) & (hc == 108)) VBlank <= 0;

		INT_line  <= (INT_line_no < 192) & (INT_line_no == vc) & (hc<128);
		INT_frame <= (vc == 244) & (hc<128);

		case(mode)
			0,1: shift <= shift << 1;
			  2: shift <= shift << 2;
			  3: shift <= shift << 4;
		endcase

		if(!hc) fetch <= 0;
		if((hc>=128) & (vc<192) & !hc[2:0]) begin
			fetch <= ~soff;
			if(~soff) begin
				case(mode)
					0: {vaddr1,vaddr2} <= { {page, 1'b0, vc[7:6],vc[2:0],vc[5:3],col}, {page, 4'b0110,vc[7:3],col}     };
					1: {vaddr1,vaddr2} <= { {page, 1'b0, vc[7:0],col},                 {page, 1'b1, vc[7:0],col}       };
				 2,3: {vaddr1,vaddr2} <= { {page[4:1],  vc[7:0],col, 2'b00},          {page[4:1],  vc[7:0],col, 2'b10}};
				endcase
			end
		end

		if(hc[2:0] == 4) begin
			paper <= fetch;
			shift <= {vram_dout1[7:0],vram_dout1[15:8],vram_dout2[7:0],vram_dout2[15:8]};
			attr  <= fetch ? vram_dout2[7:0] : 8'hFF;
			clut  <= clut_raw;
		end

		if(~io_contention) begin
			// due to permanent 1/8 I/O contention only upper 5 bits of counter are meaningful.
			lpen   <= {{5{paper}} & col, 2'd0, index[0]};
			hpen   <= (soff | (vc>192)) ? 8'd192 : vc[7:0];
			border <= border_color;
			m3_idx <= mode3_hi;
		end
	end
end

reg  [1:0] m3_idx;
reg  [3:0] index;

always_comb begin
	casex({paper, mode})
		'b0XX: index = border;
		'b10X: index = (shift[31] ^ (attr[7] & flashcnt[4])) ? {attr[6],attr[2:0]} : {attr[6],attr[5:3]};
		'b110: index = {m3_idx, shift[30], shift[31]};
		'b111: index = shift[31:28];
	endcase
end

wire I;
wire [1:0] R, G, B;
assign {G[1],R[1],B[1],I,G[0],R[0],B[0]} = (HBlank | VBlank | soff) ? 7'b0 : clut[index];

//////////////////////////////////////////////////////////////////////////

assign     dout_en = vmpr_sel | attr_sel | lpen_sel | hpen_sel;
assign     dout = port_data;
assign     video_mode = mode;

reg  [7:0] INT_line_no = 255;
reg  [6:0] vmpr;
wire [1:0] mode = vmpr[6:5];
wire [4:0] page = vmpr[4:0];

wire       vmpr_sel = (addr[7:0] == 252);
wire       clut_sel = (addr[7:0] == 248);
wire       lpen_sel = (addr[8:0] == 248);
wire       hpen_sel = (addr[8:0] == 504);
wire       intl_sel = (addr[7:0] == 249);
wire       attr_sel = (addr[7:0] == 255);

reg  [6:0] clut[16], clut_raw[16];

always @(posedge clk_sys) begin
	reg old_we;
	if(reset) begin
		vmpr <= 0;
		clut_raw <= '{default:0}; //hide the garbage (and reset screen, alas)
	end else begin
		old_we <= port_we;
		if(~old_we & port_we) begin
			if(vmpr_sel) vmpr <= din[6:0];
			if(clut_sel) clut_raw[addr[11:8]] <= din[6:0];
			if(intl_sel) INT_line_no <= din;
		end
	end
end

reg [7:0] port_data;
always_comb begin
	casex({vmpr_sel, attr_sel, lpen_sel, hpen_sel})
		'b1XXX: port_data = vmpr;
		'b01XX: port_data = attr;
		'b001X: port_data = lpen;
		'b0001: port_data = hpen;
		'b0000: port_data = 0;
	endcase
end

endmodule
