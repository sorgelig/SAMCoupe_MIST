//============================================================================
// 
//  SamCoupe replica for MiST board
//  Copyright (C) 2016 Sorgelig
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

`default_nettype none

module SamCoupe
(
   input         CLOCK_27,   // Input clock 27 MHz

   output  [5:0] VGA_R,
   output  [5:0] VGA_G,
   output  [5:0] VGA_B,
   output        VGA_HS,
   output        VGA_VS,

   output        LED,

   output        AUDIO_L,
   output        AUDIO_R,

   input         SPI_SCK,
   output        SPI_DO,
   input         SPI_DI,
   input         SPI_SS2,
   input         SPI_SS3,
   input         CONF_DATA0,

   output [12:0] SDRAM_A,
   inout  [15:0] SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nWE,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nCS,
   output  [1:0] SDRAM_BA,
   output        SDRAM_CLK,
   output        SDRAM_CKE
);

assign LED = ~(ioctl_erasing | ioctl_download | fdd_sel);


////////////////////   CLOCKS   ///////////////////
wire clk_sys;
wire locked;

pll pll
(
	.inclk0(CLOCK_27),
	.c0(clk_sys),
	.c1(SDRAM_CLK),
	.locked(locked)
);

reg  ce_psg;  //8MHz
reg  ce_6mp;
reg  ce_6mn;
reg  ce_24m;
reg  cpu_en;
reg  cpu_p;
reg  cpu_n;
wire ce_cpu_p = cpu_en & cpu_p;
wire ce_cpu_n = cpu_en & cpu_n;
wire ce_bus   = ce_6mn;

wire real_zx = (!video_mode && status[2]);

always @(negedge clk_sys) begin
	reg [3:0] counter = 0;
	reg [3:0] psg_div = 0;
	reg [4:0] zx_div = 0;
	reg       old_zx, orig_en, zx_en;
	reg [1:0] timeout;

	counter <=  counter + 1'd1;
	ce_24m  <= !counter[1:0];
	ce_6mp  <= !counter[3] & !counter[2:0];
	ce_6mn  <=  counter[3] & !counter[2:0];
	if(orig_en) begin
		cpu_p <= !counter[3] & !counter[2:0];
		cpu_n <=  counter[3] & !counter[2:0];
	end

	if(!counter[3:0]) cpu_en <= ~(ram_wait | io_wait) | zx_en;

	zx_div <= zx_div + 1'd1;
	if(zx_div == 26) zx_div <= 0;
	if(zx_en) begin
		cpu_p <= (zx_div == 0);
		cpu_n <= (zx_div == 13);
	end

	if(((old_zx != real_zx) & cpu_en) | (~zx_en & ~orig_en)) begin
		if(orig_en & (counter[3:0] == 1)) orig_en <= 0;
		if(zx_en   & (zx_div == 1)) zx_en <= 0;
		if(~orig_en & ~zx_en & (zx_div == 1)) timeout <= timeout - 1'b1;
		if(!timeout) begin
			old_zx <= real_zx;
			if(real_zx) zx_en <= 1;
				else orig_en <= 1;
		end
	end else begin
		timeout <= 3;
	end

	psg_div <= psg_div + 1'd1;
	if(psg_div == 11) psg_div <= 0;
	ce_psg  <= !psg_div;
end

// Contention model
wire ram_acc = ~nMREQ & nRFSH & ~rom0_sel & ~rom1_sel & ~ext_ram;
wire io_acc  = ~nIORQ & ~(nRD & nWR) & nM1 & &addr[7:3];
reg  ram_wait, io_wait;

always @(posedge clk_sys) begin
	reg old_ram, old_io, old_memcont, old_iocont;

	old_ram <= ram_acc;
	if(~old_ram & ram_acc & mem_contention) ram_wait <= ~status[1] & ~real_zx;

	old_memcont <= mem_contention;
	if(~mem_contention & old_memcont) ram_wait <= 0;
	
	old_io  <= io_acc;
	if(~old_io & io_acc & io_contention) io_wait <= ~status[1] & ~real_zx;

	old_iocont  <= io_contention;
	if(~io_contention & old_iocont) io_wait <= 0;
end


//////////////////   MIST ARM I/O   ///////////////////
wire        ps2_kbd_clk;
wire        ps2_kbd_data;

wire  [7:0] joystick_0;
wire  [7:0] joystick_1;
wire  [1:0] buttons;
wire  [1:0] switches;
wire        scandoubler_disable;
wire  [7:0] status;

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire        ioctl_erasing;
wire  [4:0] ioctl_index;
reg         ioctl_force_erase = 0;

mist_io #(.STRLEN(60)) user_io
(
	.*,
	.conf_str
	(
        "SAMCOUPE;DSK;O1,Contention,On,Off;O2,ZX mode speed,Emul,Real"
	),

	// unused
	.joystick_analog_0(),
	.joystick_analog_1(),
	.ps2_mouse_clk(),
	.ps2_mouse_data(),
	.sd_lba(),
	.sd_rd(),
	.sd_wr(),
	.sd_ack(),
	.sd_ack_conf(),
	.sd_conf(),
	.sd_sdhc(),
	.sd_buff_addr(),
	.sd_buff_dout(),
	.sd_buff_din(),
	.sd_buff_wr(),
	.sd_mounted()
);


///////////////////   CPU   ///////////////////
wire [15:0] addr;
wire  [7:0] cpu_din;
wire  [7:0] cpu_dout;
wire        nM1;
wire        nMREQ;
wire        nIORQ;
wire        nRD;
wire        nWR;
wire        nRFSH;
wire        nBUSACK;
wire        nINT   = ~(INT_line | INT_frame);
wire        reset  = buttons[1] | status[0] | cold_reset | warm_reset;
wire        cold_reset = (mod[1] & Fn[11]) | init_reset;
wire        warm_reset =  mod[2] & Fn[11];
wire        port_we    = ~nIORQ & ~nWR & nM1;

T80pa cpu
(
	.RESET_n(~reset),
	.CLK(clk_sys),
	.CEN_p(ce_cpu_p),
	.CEN_n(ce_cpu_n),
	.WAIT_n(1),
	.INT_n(nINT),
	.NMI_n(Fn[11]),
	.BUSRQ_n(1),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(1),
	.BUSAK_n(nBUSACK),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din)
);

always_comb begin
	case({nMREQ, ~nM1 | nIORQ | nRD})
	    'b01: cpu_din = ram_dout;
	    'b10: cpu_din = asic_dout;
	 default: cpu_din = 8'hFF;
	endcase
end

reg init_reset = 1;
always @(posedge clk_sys) begin
	reg old_download;
	old_download <= ioctl_download;
	if(~ioctl_download & old_download & !ioctl_index) init_reset <= 0;
end


//////////////////   MEMORY   //////////////////
reg  [24:0] ram_addr;
always_comb begin
	casex({fdd_read, rom0_sel | rom1_sel, ext_ram, addr[15:14]})
		'b1XX_XX: ram_addr = {3'd5, fdd_addr};
		'b01X_XX: ram_addr = {5'h10,addr[15], addr[13:0]};
		'b001_X0: ram_addr = {ext_c_off,      addr[13:0]};
		'b001_X1: ram_addr = {ext_d_off,      addr[13:0]};
		'b000_00: ram_addr = {page_ab,        addr[13:0]};
		'b000_01: ram_addr = {page_ab + 1'b1, addr[13:0]};
		'b000_10: ram_addr = {page_cd,        addr[13:0]};
		'b000_11: ram_addr = {page_cd + 1'b1, addr[13:0]};
	endcase
end

wire [7:0] ram_dout;
sram ram
(
	.*,
	.init(~locked),
	.clk(clk_sys),
	.addr(ram_addr),
	.dout(ram_dout),
	.din(cpu_dout),
	.we(~(rom0_sel | rom1_sel | ram_wp) & ~nMREQ & ~nWR),
	.rd((fdd_read | ~nMREQ) & ~nRD),

	.vid_addr1(vram_addr1),
	.vid_addr2(vram_addr2),
	.vid_data1(vram_dout1),
	.vid_data2(vram_dout2),

	.misc_addr(ioctl_addr),
	.misc_din(ioctl_dout),
	.misc_dout(),
	.misc_rd(0),
	.misc_we(ioctl_wr),
	.misc_ready()
);


//////////////////////  EXT.RAM  ////////////////////
reg  [7:0] ext_c;
reg  [7:0] ext_d;
wire [8:0] ext_c_off = 9'h40 + ext_c;
wire [8:0] ext_d_off = 9'h40 + ext_d;

wire       ext_c_sel = (addr[7:0] == 128);
wire       ext_d_sel = (addr[7:0] == 129);

always @(posedge clk_sys) begin
	reg old_we;
	old_we <= port_we;
	if(port_we & ~old_we) begin
		if(ext_c_sel) ext_c <= cpu_dout;
		if(ext_d_sel) ext_d <= cpu_dout;
	end
end


////////////////////  ASIC PORTS  ///////////////////
reg  [7:0] brdr;
wire [3:0] border_color = {brdr[5], brdr[2:0]};
wire       ear_out = brdr[4];
wire       mic_out = brdr[3];

reg  [7:0] lmpr;
wire [4:0] page_ab  = lmpr[4:0];
wire       rom0_sel =~lmpr[5] & !addr[15:14];
wire       rom1_sel = lmpr[6] & &addr[15:14];
wire       ram_wp   = lmpr[7] & !addr[15:14];

reg  [7:0] hmpr;
wire [4:0] page_cd  = hmpr[4:0];
wire [1:0] mode3_hi = hmpr[6:5];
wire       ext_ram  = hmpr[7] &  addr[15];

wire       stat_sel = (addr[7:0] == 249);
wire       lmpr_sel = (addr[7:0] == 250);
wire       hmpr_sel = (addr[7:0] == 251);
wire       kbdr_sel = (addr[7:0] == 254);
wire       brdr_sel = (addr[7:0] == 254);
wire       fdd1_sel = (addr[7:0] >= 224) & (addr[7:0] <= 231);
//wire       fdd2_sel = (addr[7:0] >= 240) & (addr[7:0] <= 247);

always @(posedge clk_sys) begin
	reg old_we;
	
	if(reset) begin
		lmpr <= 0;
		hmpr <= 0;
		brdr <= 'b10000000; // mode 4 + screen off to hide garbage on startup.
	end else begin
		old_we <= port_we;
		if(port_we & ~old_we) begin
			if(brdr_sel) brdr <= cpu_dout;
			if(lmpr_sel) lmpr <= cpu_dout;
			if(hmpr_sel) hmpr <= cpu_dout;
		end
	end
end

reg [7:0] asic_dout;
always_comb begin
	casex({kbdr_sel, stat_sel, lmpr_sel, hmpr_sel, vid_sel, fdd1_sel, kjoy_sel})
		'b1XXXXXX: asic_dout = {soff, tape_in, 1'b0, kbdjoy};
		'b01XXXXX: asic_dout = {key_data[7:5], 1'b1, ~INT_frame, 2'b11, ~INT_line};
		'b001XXXX: asic_dout = lmpr;
		'b0001XXX: asic_dout = hmpr;
		'b00001XX: asic_dout = vid_dout;
		'b000001X: asic_dout = fdd_dout;
		'b0000001: asic_dout = {2'b00, joystick_0[5:0] | joystick_1[5:0]};
		'b0000000: asic_dout = 8'hFF;
	endcase
end


////////////////////   AUDIO   ///////////////////
wire [7:0] sound_data;
wire [7:0] psg_ch_l;
wire [7:0] psg_ch_r;
wire       tape_in = 0;

saa1099 psg
(
	.clk_sys(clk_sys),  
	.ce(ce_psg),
	.rst_n(~reset),
	.cs_n((addr[7:0] != 255) | nIORQ),
	.a0(addr[8]),
	.wr_n(nWR),
	.din(cpu_dout),
	.out_l(psg_ch_l),
	.out_r(psg_ch_r)
);

sigma_delta_dac #(8) dac_l
(
	.CLK(clk_sys),
	.RESET(reset),
	.DACin({1'b0, psg_ch_l} + {1'b0, ear_out, mic_out, tape_in, 5'b00000}),
	.DACout(AUDIO_L)
);

sigma_delta_dac #(8) dac_r
(
	.CLK(clk_sys),
	.RESET(reset),
	.DACin({1'b0, psg_ch_r} + {1'b0, ear_out, mic_out, tape_in, 5'b00000}),
	.DACout(AUDIO_R)
);


////////////////////   VIDEO   ///////////////////
wire [18:0] vram_addr1;
wire [18:0] vram_addr2;
wire        vram_rd1;
wire        vram_rd2;
wire [15:0] vram_dout1;
wire [15:0] vram_dout2;
wire  [7:0] vid_dout;
wire        vid_sel;
wire        soff = brdr[7] & video_mode[1];
wire        INT_line;
wire        INT_frame;
wire        mem_contention;
wire        io_contention;
wire  [1:0] video_mode;

video video
(
	.*,
	.din(cpu_dout),
	.dout(vid_dout),
	.dout_en(vid_sel)
);


//////////////////   KEYBOARD   //////////////////
wire [11:1] Fn;
wire  [2:0] mod;
wire  [7:0] key_data;
keyboard kbd( .* );

wire        kjoy_sel = (addr[7:0] == 'h1F);
wire  [4:0] kbdjoy = key_data[4:0]
	& (addr[12] ? 5'b11111 : ~{joystick_0[1],  joystick_0[0], joystick_0[2], joystick_0[3], joystick_0[4] | joystick_0[5]})
	& (addr[11] ? 5'b11111 : ~{joystick_1[4] | joystick_1[5], joystick_1[3], joystick_1[2], joystick_1[0],  joystick_1[1]});


///////////////////   FDC   ///////////////////
wire [19:0] fdd_addr;
wire [19:0] fdd_size;
wire        fdd_rd;
reg         fdd_ready;
reg         fdd_side;
wire        fdd_sel  = fdd1_sel & ~nIORQ & nM1;
wire        fdd_read = fdd_rd & fdd_sel;
wire  [7:0] fdd_dout;

always @(posedge clk_sys) begin
	reg old_wr;
	reg old_download;
	reg old_m1;

	old_wr <= nWR;
	if(old_wr & ~nWR & fdd_sel) fdd_side <= addr[2];

	old_download <= ioctl_download;
	if(cold_reset) begin
		fdd_ready <= 0;
		fdd_size  <= 0;
	end else begin
		if(~ioctl_download & old_download & (ioctl_index == 1)) begin
			fdd_ready <= 1;
			fdd_size  <= ioctl_addr[19:0] + 1'b1;
		end
	end
end

wd1793 fdd
(
	.clk_sys(clk_sys),
	.ce(cpu_n),
	.reset(reset),
	.io_en(fdd_sel),
	.rd(~nRD),
	.wr(~nWR),
	.addr(addr[1:0]),
	.din(cpu_dout),
	.dout(fdd_dout),
	
	.buff_size(fdd_size),
	.buff_addr(fdd_addr),
	.buff_read(fdd_rd),
	.buff_din(ram_dout),

	.size_code(4),
	.side(fdd_side),
	.ready(fdd_ready)
);


endmodule
