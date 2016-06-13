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

assign LED = ~(ioctl_erasing | ioctl_download | fdd_io);

`include "build_id.v"
localparam CONF_STR = {"SAMCOUPE;DSK;F3,DSK;S4,DSK;O1,CPU Throttle,On,Off;O2,ZX Mode Speed,Emul,Real;O5,External RAM,On,Off;V0,v1.10.",`BUILD_DATE};


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

reg  ce_psg;   //8MHz
reg  ce_6mp;
reg  ce_6mn;
reg  ce_24m;
reg  cpu_en;
reg  cpu_p;
reg  cpu_n;
wire ce_cpu_p = cpu_en & cpu_p;
wire ce_cpu_n = cpu_en & cpu_n;
wire ce_bus   = ce_6mn;

wire real_zx = (!video_mode && status[2] && ~status[1]);

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
wire        ps2_mouse_clk;
wire        ps2_mouse_data;

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

wire [31:0] sd_lba;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire        img_mounted;
wire [31:0] img_size;

mist_io #(.STRLEN($size(CONF_STR)>>3)) user_io
(
	.*,
	.conf_str(CONF_STR),
	.sd_conf(0),
	.sd_sdhc(1),

	// unused
	.joystick_analog_0(),
	.joystick_analog_1(),
	.sd_ack_conf()
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
	.NMI_n((mod[2:1]==0) & Fn[11]),
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
	    'b01: cpu_din = ext_ena ? ram_dout : 8'hFF;
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
		'b1XX_XX: ram_addr = fdd_addr;
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
	.we(~(rom0_sel | rom1_sel | ram_wp) & ~nMREQ & ~nWR & ext_ena),
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
wire       ext_ena   = ~(ext_ram & status[5]);

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
wire       ear_out  = brdr[4];
wire       mic_out  = 0; // brdr[3]; it seems not used in SAM Coupe.

reg  [7:0] lmpr;
wire [4:0] page_ab  = lmpr[4:0];
wire       rom0_sel =~lmpr[5] & !addr[15:14];
wire       rom1_sel = lmpr[6] & &addr[15:14];
wire       ram_wp   = lmpr[7] & !addr[15:14];

reg  [7:0] hmpr;
wire [4:0] page_cd  = hmpr[4:0];
wire [1:0] mode3_hi = hmpr[6:5];
wire       ext_ram  = hmpr[7] &  addr[15];

wire       lptd_sel = (addr[7:0] == 232);
wire       lpts_sel = (addr[7:0] == 233);
wire       stat_sel = (addr[7:0] == 249);
wire       lmpr_sel = (addr[7:0] == 250);
wire       hmpr_sel = (addr[7:0] == 251);
wire       kbdr_sel = (addr[7:0] == 254);
wire       brdr_sel = (addr[7:0] == 254);
wire       fdd1_sel = (addr[7:0] >= 224) & (addr[7:0] <= 231);
wire       fdd2_sel = (addr[7:0] >= 240) & (addr[7:0] <= 247);

always @(posedge clk_sys) begin
	reg old_we;
	
	if(reset) begin
		lmpr <= 0;
		hmpr <= 0;
		brdr <= 0;
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
	casex({kbdr_sel, stat_sel, lmpr_sel, hmpr_sel, vid_sel, fdd_sel, kjoy_sel, lptd_sel | lpts_sel})
		'b1XXXXXXX: asic_dout = {soff, tape_in, 1'b0, hid_data};
		'b01XXXXXX: asic_dout = {key_data[7:5], 1'b1, ~INT_frame, 2'b11, ~INT_line};
		'b001XXXXX: asic_dout = lmpr;
		'b0001XXXX: asic_dout = hmpr;
		'b00001XXX: asic_dout = vid_dout;
		'b000001XX: asic_dout = fdd_dout;
		'b0000001X: asic_dout = {2'b00, joystick_0[5:0] | joystick_1[5:0]};
		'b00000001: asic_dout = 0; // fake LPT port.
		'b00000000: asic_dout = 8'hFF;
	endcase
end


////////////////////   AUDIO   ///////////////////
wire [7:0] psg_ch_l;
wire [7:0] psg_ch_r;
wire       tape_in = 0; //tape is not implemented (yet?)

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
	.DACin({1'b0, psg_ch_l} + {1'b0, ear_out, mic_out, tape_in, 5'b00000} + vox_l),
	.DACout(AUDIO_L)
);

sigma_delta_dac #(8) dac_r
(
	.CLK(clk_sys),
	.RESET(reset),
	.DACin({1'b0, psg_ch_r} + {1'b0, ear_out, mic_out, tape_in, 5'b00000} + vox_r),
	.DACout(AUDIO_R)
);

reg [7:0] vox_l, vox_r;
always @(posedge clk_sys) begin
	reg old_we, old_stb;
	reg [7:0] data;

	if(reset) {vox_l, vox_r, old_stb, data} <= 0;
	else begin
		old_we <= port_we;
		if(port_we & ~old_we) begin
			if(lptd_sel) data <= cpu_dout;
			if(lpts_sel) begin
				if(~old_stb & cpu_dout[0]) vox_l <= data;
				if(old_stb & ~cpu_dout[0]) vox_r <= data;
				old_stb <= cpu_dout[0];
			end
		end
	end
end


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


////////////////////   HID   /////////////////////
wire [11:1] Fn;
wire  [2:0] mod;
wire  [7:0] key_data;
reg         autostart;
keyboard kbd( .*, .restart(rom0_sel & (addr == 0) & ~nMREQ & ~nRD));

wire        kjoy_sel = (addr[7:0] == 'h1F);
wire  [4:0] hid_data = key_data[4:0] & mouse_data
	& (addr[12] ? 5'b11111 : ~{joystick_0[1],  joystick_0[0], joystick_0[2], joystick_0[3], joystick_0[4] | joystick_0[5]})
	& (addr[11] ? 5'b11111 : ~{joystick_1[4] | joystick_1[5], joystick_1[3], joystick_1[2], joystick_1[0],  joystick_1[1]});

wire  [4:0] mouse_data;
mouse mouse( .*, .dout(mouse_data), .rd(kbdr_sel & &addr[15:8] & nM1 & ~nIORQ & ~nRD));


///////////////////   FDC   ///////////////////
wire        fdd_sel  = fdd1_sel  | fdd2_sel;
wire        fdd_io   = fdd1_io   | fdd2_io;
wire        fdd_read = fdd1_read | fdd2_read;
wire [24:0] fdd_addr = fdd1_sel ? {3'd5, fdd1_addr} : {3'd6, fdd2_addr};
wire  [7:0] fdd_dout = fdd1_sel ? (fdd1w_ready ? fdd1w_dout : fdd1_dout) : fdd2_dout;

wire[127:0] edsk_sig = "EXTENDED CPC DSK";
wire[127:0] sig_pos  = edsk_sig >> (8'd120-(ioctl_addr[7:0]<<3));
wire        is_sdf   = ((ioctl_addr[19:0] == 983039) | (ioctl_addr[19:0] == 1019903));

// FDD1
wire [19:0] fdd1_addr;
wire [19:0] fdd1_size;
wire        fdd1_rd;
reg         fdd1_ready, fdd1w_ready;
reg         fdd1_side;
wire        fdd1_io   = fdd1_sel & ~nIORQ & nM1;
wire        fdd1_read = fdd1_rd & fdd1_io;
wire  [7:0] fdd1_dout, fdd1w_dout;
reg   [2:0] fdd1_type;

always @(posedge clk_sys) begin
	reg old_wr, old_wr2;
	reg old_download;
	reg old_mounted;
	reg edsk;

	old_wr <= nWR;
	if(old_wr & ~nWR & fdd1_io) fdd1_side <= addr[2];

	old_wr2 <= ioctl_wr;
	old_download <= ioctl_download;
	old_mounted <= img_mounted;

	if(cold_reset) begin
		fdd1_ready  <= 0;
		fdd1w_ready <= 0;
		fdd1_size   <= 0;
	end else begin
		if(~old_download & ioctl_download & (ioctl_index == 1)) edsk <= 1;

		if(~old_wr2 & ioctl_wr & ioctl_download & (ioctl_index == 1) & 
			(ioctl_addr[19:0] < 16) & (sig_pos[7:0] != ioctl_dout)) edsk <= 0;

		if(~ioctl_download & old_download & (ioctl_index == 1)) begin
			fdd1_ready  <= 1;
			fdd1w_ready <= 0;
			fdd1_size   <= ioctl_addr[19:0] + 1'b1;
			if(edsk) fdd1_type <= 6;
			else if(is_sdf) fdd1_type <= 5;
			else fdd1_type <= 4;
		end

		if(~old_mounted & img_mounted) begin
			fdd1w_ready <= (img_size == 819200);
			fdd1_ready  <= 0;
		end
	end
end

wd1793w fdd1w
(
	.*,

	.ce(cpu_n),
	.io_en(fdd1_io & fdd1w_ready),
	.rd(~nRD),
	.wr(~nWR),
	.addr(addr[1:0]),
	.din(cpu_dout),
	.dout(fdd1w_dout),

	.size_code(4),
	.side(fdd1_side),
	.ready(fdd1w_ready),
	.drq(),
	.intrq(),
	.busy()
);

wd1793 fdd1
(
	.clk_sys(clk_sys),
	.ce(cpu_n),
	.reset(reset),
	.io_en(fdd1_io & fdd1_ready),
	.rd(~nRD),
	.wr(~nWR),
	.addr(addr[1:0]),
	.din(cpu_dout),
	.dout(fdd1_dout),

	.input_active(ioctl_download & (ioctl_index == 1)),
	.input_addr(ioctl_addr[19:0]),
	.input_data(ioctl_dout),
	.input_wr(ioctl_wr),

	.buff_size(fdd1_size),
	.buff_addr(fdd1_addr),
	.buff_read(fdd1_rd),
	.buff_din(ram_dout),

	.size_code(fdd1_type),
	.side(fdd1_side),
	.ready(fdd1_ready)
);

always @(posedge clk_sys) begin
	reg old_download;
	integer counter;

	if(reset) begin
		autostart <= 0;
		counter   <= 0;
	end else begin
		old_download <= ioctl_download;
		if(old_download & ~ioctl_download & (ioctl_index == 1)) begin
			autostart <= 1;
			counter   <= 1000000;
		end else if(ce_6mp && counter) begin
			counter   <= counter - 1;
			if(counter == 1) autostart <= 0;
		end
	end
end


// FDD2
wire [19:0] fdd2_addr;
wire [19:0] fdd2_size;
wire        fdd2_rd;
reg         fdd2_ready;
reg         fdd2_side;
wire        fdd2_io   = fdd2_sel & ~nIORQ & nM1;
wire        fdd2_read = fdd2_rd & fdd2_io;
wire  [7:0] fdd2_dout;
reg   [2:0] fdd2_type;

always @(posedge clk_sys) begin
	reg old_wr, old_wr2;
	reg old_download;
	reg old_m1;
	reg edsk;

	old_wr <= nWR;
	if(old_wr & ~nWR & fdd2_io) fdd2_side <= addr[2];

	old_wr2 <= ioctl_wr;
	old_download <= ioctl_download;
	if(cold_reset) begin
		fdd2_ready <= 0;
		fdd2_size  <= 0;
	end else begin
		if(~old_download & ioctl_download & (ioctl_index == 2)) edsk <= 1;

		if(~old_wr2 & ioctl_wr & ioctl_download & (ioctl_index == 2) & 
			(ioctl_addr[19:0] < 16) & (sig_pos[7:0] != ioctl_dout)) edsk <= 0;

		if(~ioctl_download & old_download & (ioctl_index == 2)) begin
			fdd2_ready <= 1;
			fdd2_size  <= ioctl_addr[19:0] + 1'b1;
			if(edsk) fdd2_type <= 6;
			else if(is_sdf) fdd2_type <= 5;
			else fdd2_type <= 4;
		end
	end
end

wd1793 fdd2
(
	.clk_sys(clk_sys),
	.ce(cpu_n),
	.reset(reset),
	.io_en(fdd2_io),
	.rd(~nRD),
	.wr(~nWR),
	.addr(addr[1:0]),
	.din(cpu_dout),
	.dout(fdd2_dout),

	.input_active(ioctl_download & (ioctl_index == 2)),
	.input_addr(ioctl_addr[19:0]),
	.input_data(ioctl_dout),
	.input_wr(ioctl_wr),

	.buff_size(fdd2_size),
	.buff_addr(fdd2_addr),
	.buff_read(fdd2_rd),
	.buff_din(ram_dout),

	.size_code(fdd2_type),
	.side(fdd2_side),
	.ready(fdd2_ready)
);

endmodule
