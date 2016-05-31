`timescale 1ns / 1ps
`default_nettype none

//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: Miguel Angel Rodriguez Jodar
//
// Create Date:    17:20:11 08/09/2015
// Design Name:    SAM Coupe clone
// Module Name:    saa1099
// Project Name:   SAM Coupe clone
// Target Devices: Spartan 6
// Tool versions:  ISE 12.4
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//
// Synchronous version with fixes for volume and envelope. (Sorgelig)
// 
//
//////////////////////////////////////////////////////////////////////////////////
module saa1099
(
	input        clk_sys,
	input        ce,      // 8 MHz
	input        rst_n,
	input        cs_n,
	input        a0,      // 0=data, 1=address
	input        wr_n,
	input  [7:0] din,
	output [7:0] out_l,
	output [7:0] out_r
);

// DTACK is not implemented. Sorry about that

reg [7:0] amplit0, amplit1, amplit2, amplit3, amplit4, amplit5;
reg [7:0] freq0, freq1, freq2, freq3, freq4, freq5;
reg [7:0] oct10, oct32, oct54;
reg [7:0] freqenable;
reg [7:0] noiseenable;
reg [7:0] noisegen;
reg [7:0] envelope0, envelope1;
reg [7:0] ctrl;  // frequency reset and sound enable for all channels

reg [4:0] addr;  // holds the address of the register to write to
wire      rst = ~rst_n | ctrl[1];
reg       wr;

// Write values into internal registers
always @(posedge clk_sys) begin
	reg old_wr;
	old_wr <= wr_n;

	if(~rst_n) begin
		addr <= 0;
		wr <= 0;
		{amplit0, amplit1, amplit2, amplit3, amplit4, amplit5} <= 0;
		{freq0, freq1, freq2, freq3, freq4, freq5} <= 0;
		{oct10, oct32, oct54} <= 0;
		{freqenable, noiseenable, noisegen} <= 0;
		{envelope0, envelope1} <= 0;
		ctrl <= 0;
	end
	else begin
		wr <= 0;
		if(!cs_n & old_wr & !wr_n) begin
			wr <= 1;
			if(a0) addr <= din[4:0];
			else begin
				case (addr)
					'h00: amplit0 <= din;
					'h01: amplit1 <= din;
					'h02: amplit2 <= din;
					'h03: amplit3 <= din;
					'h04: amplit4 <= din;
					'h05: amplit5 <= din;

					'h08: freq0   <= din;
					'h09: freq1   <= din;
					'h0A: freq2   <= din;
					'h0B: freq3   <= din;
					'h0C: freq4   <= din;
					'h0D: freq5   <= din;

					'h10: oct10   <= din;
					'h11: oct32   <= din;
					'h12: oct54   <= din;

					'h14: freqenable <= din;
					'h15: noiseenable<= din;
					'h16: noisegen   <= din;

					'h18: envelope0  <= din;
					'h19: envelope1  <= din;

					'h1C: ctrl       <= din;
				endcase
			end
		end
	end
end

wire [5:0] out0_l, out0_r;
saa1099_triplet top
(
	.*,
	.vol_l('{amplit0[3:0], amplit1[3:0], amplit2[3:0]}),
	.vol_r('{amplit0[7:4], amplit1[7:4], amplit2[7:4]}),

	.freq('{freq0, freq1, freq2}),
	.octave('{oct10[2:0], oct10[6:4], oct32[2:0]}),
	.freq_en(freqenable[2:0]),

	.noise_en(noiseenable[2:0]),
	.noise_freq(noisegen[1:0]),

	.envelope(envelope0),

	.wr_addr(wr &  a0 & (din[4:0] == 'h18)),
	.wr_data(wr & !a0 & (addr == 'h18)),

	.out_l(out0_l),
	.out_r(out0_r)
);

wire [5:0] out1_l, out1_r;
saa1099_triplet bottom
(
	.*,
	.vol_l('{amplit3[3:0], amplit4[3:0], amplit5[3:0]}),
	.vol_r('{amplit3[7:4], amplit4[7:4], amplit5[7:4]}),

	.freq('{freq3, freq4, freq5}),
	.octave('{oct32[6:4], oct54[2:0], oct54[6:4]}),
	.freq_en(freqenable[5:3]),

	.noise_en(noiseenable[5:3]),
	.noise_freq(noisegen[5:4]),

	.envelope(envelope1),

	.wr_addr(wr &  a0 & (din[4:0] == 'h19)),
	.wr_data(wr & !a0 & (addr == 'h19)),

	.out_l(out1_l),
	.out_r(out1_r)
);

saa1099_output_mixer outmix_left (.*, .en(ctrl[0]), .i0(out0_l), .i1(out1_l), .o(out_l));
saa1099_output_mixer outmix_right(.*, .en(ctrl[0]), .i0(out0_r), .i1(out1_r), .o(out_r));

endmodule

/////////////////////////////////////////////////////////////////////////////////
module saa1099_triplet
(
	input        rst,
	input        clk_sys,
	input        ce,
	input  [3:0] vol_l[3],
	input  [3:0] vol_r[3],
	input  [7:0] freq[3],
	input  [2:0] octave[3],
	input  [2:0] freq_en,
	input  [2:0] noise_en,
	input  [1:0] noise_freq,
	input  [7:0] envelope,

	input        wr_addr,
	input        wr_data,

	output [5:0] out_l,
	output [5:0] out_r
);

wire       tone0, tone1, tone2, noise;
wire       pulse_noise, pulse_envelope;
wire [3:0] out0_l, out0_r, out1_l, out1_r, out2_l, out2_r;

saa1099_tone_gen  freq_gen0(.*, .out(tone0), .octave(octave[0]), .freq(freq[0]), .pulseout(pulse_noise));
saa1099_tone_gen  freq_gen1(.*, .out(tone1), .octave(octave[1]), .freq(freq[1]), .pulseout(pulse_envelope));
saa1099_tone_gen  freq_gen2(.*, .out(tone2), .octave(octave[2]), .freq(freq[2]), .pulseout());
saa1099_noise_gen noise_gen(.*, .out(noise));

saa1099_amp amp0
(
	.*,
	.mixmode({noise_en[0], freq_en[0] & (noise_freq != 3)}),
	.tone(tone0),
	.envreg(0),
	.volume_l(vol_l[0]),
	.volume_r(vol_r[0]),
	.out_left(out0_l),
	.out_right(out0_r)
);

saa1099_amp amp1
(
	.*,
	.mixmode({noise_en[1], freq_en[1] & ~envelope[7]}),
	.tone(tone1),
	.envreg(0),
	.volume_l(vol_l[1]),
	.volume_r(vol_r[1]),
	.out_left(out1_l),
	.out_right(out1_r)
);

saa1099_amp amp2
(
	.*,
	.mixmode({noise_en[2], freq_en[2]}),
	.tone(tone2),
	.envreg(envelope),
	.volume_l(vol_l[2]),
	.volume_r(vol_r[2]),
	.out_left(out2_l),
	.out_right(out2_r)
);

assign out_l = out0_l + out1_l + out2_l;
assign out_r = out0_r + out1_r + out2_r;

endmodule

/////////////////////////////////////////////////////////////////////////////////

module saa1099_tone_gen
(
	input       rst,
	input       clk_sys,
	input       ce,
	input [2:0] octave,
	input [7:0] freq,
	output reg  out,
	output reg  pulseout
);

wire [16:0] fcount = ((17'd511 - freq) << (4'd8 - octave)) - 1'd1;
always @(posedge clk_sys) begin
	reg [16:0] count;

	pulseout <= 0;
	if(rst) begin
		count <= fcount;
		out   <= 0;
	end else if(ce) begin
		if(!count) begin
			count <= fcount;
			pulseout <= 1;
			out <= ~out;
		end else begin
			count <= count - 1'd1;
		end
	end
end

endmodule

/////////////////////////////////////////////////////////////////////////////////

module saa1099_noise_gen
(
	input        rst,
	input        clk_sys,
	input        ce,
	input        pulse_noise,
	input  [1:0] noise_freq,
	output       out
);

reg  [16:0] lfsr = 0;
wire [16:0] new_lfsr = {(lfsr[0] ^ lfsr[2] ^ !lfsr), lfsr[16:1]};
wire [10:0] fcount = (11'd256 << noise_freq) - 1'b1;

always @(posedge clk_sys) begin
	reg [10:0] count;

	if(rst) begin
		count <= fcount;
	end else
	if(noise_freq != 3) begin
		if(ce) begin
			if(!count) begin
				count <= fcount;
				lfsr <= new_lfsr;
			end else begin
				count <= count - 1'd1;
			end
		end
	end else if(pulse_noise) begin
		lfsr <= new_lfsr;
	end
end

assign out = lfsr[0];

endmodule

/////////////////////////////////////////////////////////////////////////////////

module saa1099_amp
(
	input        rst,
	input        clk_sys,
	input  [7:0] envreg,
	input  [1:0] mixmode,
	input        tone,
	input        noise,
	input        wr_addr,
	input        wr_data,
	input        pulse_envelope,
	input  [3:0] volume_l,
	input  [3:0] volume_r,
	output [3:0] out_left,
	output [3:0] out_right
);

wire       phases[8] = '{0,0,0,0,1,1,0,0};
wire [1:0] env[8][2] = '{'{0,0}, '{1,1}, '{2,0}, '{2,0}, '{3,2}, '{3,2}, '{3,0}, '{3,0}};
wire [3:0] levels[4][16] = 
'{
	'{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	'{15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15},
	'{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	'{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}
};

reg [2:0] envshape;
reg       stereoshape;
wire      env_enable = envreg[7];
wire      env_resolution = envreg[4];
reg [3:0] envcounter;
reg       phase;

always @(posedge clk_sys) begin
	reg envclock;
	reg pending_data;

	if(rst | ~env_enable) begin
		pending_data <= 0;
		envshape <= envreg[3:1];
		stereoshape <= envreg[0];
		envclock <= envreg[5];
		pending_data <= 0;
		phase <= 0;
		envcounter <= 0;
	end
	else begin
		if(wr_data) pending_data <= 1;
		if(envclock ? wr_addr : pulse_envelope) begin  // pulse from internal or external clock?
			if((envcounter | env_resolution) == 15) begin
				if(phase >= phases[envshape]) begin
					if(envshape[0]) begin
						phase <= 0;
						envcounter <= 0;
					end
					if(pending_data) begin  // if we reached one of the designated points (3) or (4) and there is pending data, load it
						envshape <= envreg[3:1];
						stereoshape <= envreg[0];
						envclock <= envreg[5];
						pending_data <= 0;
						phase <= 0;
						envcounter <= 0;
					end
				end else begin
					phase <= 1;
					envcounter <= 0;
				end
			end else begin
				envcounter <= envcounter + env_resolution + 1'd1;
			end
		end
	end
end

wire [3:0] envleft  = levels[env[envshape][phase]][envcounter] & {3'b111, ~env_resolution};
wire [3:0] envright = (!stereoshape)? envleft : ~(envleft | env_resolution); // bit 0 of envreg inverts envelope shape
wire [3:0] env_out_left;
wire [3:0] env_out_right;

reg  [1:0] outint;
always_comb begin
	case(mixmode)
		0: outint <= 0;
		1: outint <= {tone,  1'b0};
		2: outint <= {noise, 1'b0};
		3: outint <= (tone & noise) ? 2'b01 : {tone, 1'b0};
	endcase
end

reg [3:0] mix_left;
reg [3:0] mix_right;

wire[3:0] vol_nft_l = outint[0] ? volume_l>>2 : volume_l;
wire[3:0] vol_nft_r = outint[0] ? volume_r>>2 : volume_r;

saa1099_mul_env modulate_left (.a(vol_nft_l), .b(envleft),  .o(env_out_left));
saa1099_mul_env modulate_right(.a(vol_nft_r), .b(envright), .o(env_out_right));

always_comb begin
	case({env_enable, outint})
		'b100, 'b101: {mix_left, mix_right} = {env_out_left, env_out_right};
		'b001, 'b010: {mix_left, mix_right} = {vol_nft_l, vol_nft_r};
		     default: {mix_left, mix_right} = 0;
	endcase
end

assign {out_left, out_right} = {mix_left, mix_right};

endmodule

/////////////////////////////////////////////////////////////////////////////////

module saa1099_mul_env
(
	input  [3:0] a, // amplitude
	input  [3:0] b, // envelope
	output [3:0] o  // output
);

wire [7:0] res =  (b[0] ?      a     : 8'd0)+ 
                  (b[1] ? {  a,1'b0} : 8'd0)+
                  (b[2] ? { a,2'b00} : 8'd0)+
                  (b[3] ? {a,3'b000} : 8'd0);

assign o = res[7:4];

endmodule

/////////////////////////////////////////////////////////////////////////////////

module saa1099_output_mixer
(
	input        clk_sys,
	input        ce,
	input        en,
	input  [5:0] i0,
	input  [5:0] i1,
	output [7:0] o
);

assign o = out;

//
// Although channels have 5 bits, actually they have 4 bits.
// 5th bit is used by noise and can be borrowed from other channel or discarded (depend on case).
wire [7:0] mix = i0 + i1;
wire [7:0] compressor_table[96] =
'{
	'h00, 'h08, 'h0E, 'h13, 'h17, 'h1C, 'h20, 'h24, 'h27, 'h2B, 'h2F, 'h32, 'h36, 'h39, 'h3C, 'h3F, 
	'h43, 'h46, 'h49, 'h4C, 'h4F, 'h52, 'h55, 'h58, 'h5A, 'h5D, 'h60, 'h63, 'h66, 'h68, 'h6B, 'h6E, 
	'h70, 'h73, 'h75, 'h78, 'h7B, 'h7D, 'h80, 'h82, 'h85, 'h87, 'h8A, 'h8C, 'h8F, 'h91, 'h94, 'h96, 
	'h98, 'h9B, 'h9D, 'h9F, 'hA2, 'hA4, 'hA6, 'hA9, 'hAB, 'hAD, 'hB0, 'hB2, 'hB4, 'hB6, 'hB9, 'hBB, 
	'hBD, 'hBF, 'hC2, 'hC4, 'hC6, 'hC8, 'hCA, 'hCC, 'hCF, 'hD1, 'hD3, 'hD5, 'hD7, 'hD9, 'hDB, 'hDE, 
	'hE0, 'hE2, 'hE4, 'hE6, 'hE8, 'hEA, 'hEC, 'hEE, 'hF0, 'hF2, 'hF4, 'hF6, 'hF8, 'hFA, 'hFC, 'hFF
};

// Clean the audio.
reg [7:0] out;
always @(posedge clk_sys) begin
	reg ced;
	ced <= ce;
	if(ced) out <= ~en ? 8'h00 : (mix>=$size(compressor_table)) ? 8'hFF : compressor_table[mix];
end


endmodule
