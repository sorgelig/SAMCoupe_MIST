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
	input         nMREQ,
	input         nIORQ,
	input         nRFSH,
	input         nWR,
	
	output        mem_contention,
	output        io_contention,

	output reg    INT_line,
	output reg    INT_frame,
	
	input  [11:1] Fn,

	// VRAM interfacing
	output [18:0] vram_addr1,
	output [18:0] vram_addr2,
	input  [15:0] vram_dout1,
	input  [15:0] vram_dout2,

	// Misc. signals
	input   [3:0] border_color,
	input         scandoubler_disable,
	input         soff,
	output        mode34,
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

assign io_contention  = ~&hc[2:0];
assign mem_contention = (paper | (!mode & hc[6])) ? ~&hc[2:0] : ~&hc[1:0];

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
	.VGA_Rx({2{R, R && I}}),
	.VGA_Gx({2{G, G && I}}),
	.VGA_Bx({2{B, B && I}}),
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
reg        paper;

reg  [8:0] hc  = 0;
reg  [8:0] vc  = 0;
wire [4:0] col = hc[7:3] - 5'd15;

always @(posedge clk_sys) begin
	reg  [6:0] intcnt;

	if(ce_6mp) begin
		if(!intcnt) {INT_line, INT_frame} <= 0;
			else intcnt <= intcnt - 1'd1;

		if (hc==383) begin

			if((INT_line_no < 192) & (INT_line_no == vc)) {INT_line, intcnt} <= 8'hFF;
			if(vc == 243) {INT_frame, intcnt} <= 8'hFF;

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
		if(hc == 24)  HBlank <= 1;
		if(hc == 40)  HSync  <= 1;
		if(hc == 72)  HSync  <= 0;
		if(hc == 104) HBlank <= 0;

		if((vc == 240) & (hc == 24))  VBlank <= 1;
		if( vc == 244) VSync  <= 1;
		if( vc == 248) VSync  <= 0;
		if((vc == 264) & (hc == 104)) VBlank <= 0;

		if(!hc[2:0]) pixel3hi <= mode3_hi;

		if(!hc) paper <= 0;
		if((hc>=120) & (vc<192) & !hc[2:0]) begin
			case(mode)
				0: begin
						vaddr1 <= {page, 1'b0, vc[7:6],vc[2:0],vc[5:3],col};
						vaddr2 <= {page, 4'b0110,vc[7:3],col};
					end
				1: begin
						vaddr1 <= {page, 1'b0, vc[7:0],col};
						vaddr2 <= {page, 1'b1, vc[7:0],col};
					end
				2,3: begin
						vaddr1 <= {page[4:1], vc[7:0],col, 2'b00};
						vaddr2 <= {page[4:1], vc[7:0],col, 2'b10};
					end
			endcase
			shift <= {vram_dout1[7:0],vram_dout1[15:8],vram_dout2[7:0],vram_dout2[15:8]};
			if(hc >=128) begin
				attr  <= vram_dout2[7:0];
				paper <= ~soff;
			end
		end else begin
			case(mode)
				0,1: shift <= shift << 1;
				  2: shift <= shift << 2;
				  3: shift <= shift << 4;
			endcase
		end
	end
end

reg  [1:0] pixel3hi;
reg  [3:0] index;

always_comb begin
	casex({paper, mode})
		'b0XX: index = border_color;
		'b10X: index = (shift[31] ^ (attr[7] & flashcnt[4])) ? {attr[6],attr[2:0]} : {attr[6],attr[5:3]};
		'b110: index = {pixel3hi, shift[30], shift[31]};
		'b111: index = shift[31:28];
	endcase
end

wire I;
wire [1:0] R, G, B;
assign {G[1],R[1],B[1],I,G[0],R[0],B[0]} = (HBlank | VBlank | soff) ? 7'b0 : palette[index];

//////////////////////////////////////////////////////////////////////////

assign     dout_en = vmpr_sel | attr_sel | lpen_sel | hpen_sel;
assign     dout = port_data;
assign     mode34 = vmpr[6];

reg  [7:0] INT_line_no = 255;
reg  [6:0] vmpr;
wire [1:0] mode = vmpr[6:5];
wire [4:0] page = vmpr[4:0];

wire       vmpr_sel = (addr[7:0] == 252);
wire       pal_sel  = (addr[7:0] == 248);
wire       lpen_sel = (addr[8:0] == 248);
wire       hpen_sel = (addr[8:0] == 504);
wire       intl_sel = (addr[7:0] == 249);
wire       attr_sel = (addr[7:0] == 255);

reg  [6:0] palette[16] = '{'h00, 'h11, 'h22, 'h33, 'h44, 'h55, 'h66, 'h77, 'h00, 'h19, 'h2A, 'h3B, 'h4C, 'h5D, 'h6E, 'h7F};
wire       io_wr = ~nIORQ & ~nWR;
always @(posedge clk_sys) begin
	reg old_wr;
	if(reset) begin
		vmpr <= 'b01100000; // mode 4 + screen off to hide garbage on startup.
	end else if(ce_6mn) begin
		old_wr <= io_wr;
		if(~old_wr & io_wr) begin
			if(vmpr_sel) vmpr <= din[6:0];
			if(pal_sel)  palette[addr[11:8]] <= din[6:0];
			if(intl_sel) INT_line_no <= din;
		end
	end
end

reg  [7:0] hpen;
reg  [7:0] lpen;
always @(posedge clk_sys) begin
	reg old_iorq;
	old_iorq <= nIORQ;
	if(old_iorq & ~nIORQ) begin
		lpen <= paper ? {~hc[7], hc[6:0]} : index[0];
		hpen <= (soff | (vc>192)) ? 8'd192 : vc[7:0];
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
