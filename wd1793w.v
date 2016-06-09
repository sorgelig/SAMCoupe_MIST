`default_nettype none

// ====================================================================
//                        VECTOR-06C FPGA REPLICA
//
//             Copyright (C) 2007,2008 Viacheslav Slavinsky
//
// This core is distributed under modified BSD license. 
// For complete licensing information see LICENSE.TXT.
// -------------------------------------------------------------------- 
//
// An open implementation of Vector-06C home computer
//
// Author: Viacheslav Slavinsky, http://sensi.org/~svo
// 
// Design File: wd1793.v
//
// This module approximates the inner workings of a WD1793 floppy disk
// controller to some minimal extent. Track read/write operations
// are not supported, other ops are mimicked only barely enough.
//
// --------------------------------------------------------------------
//
// Modified version by Sorgelig to work with image in RAM
//
//

module wd1793w
(
	input        clk_sys,    // sys clock
	input        ce,         // ce at CPU clock rate
	input        reset,	     // async reset
	input        io_en,
	input        rd,         // i/o read
	input        wr,         // i/o write
	input  [1:0] addr,       // i/o port addr
	input  [7:0] din,        // i/o data in
	output [7:0] dout,       // i/o data out
	output       drq,        // DMA request
	output       intrq,
	output       busy,

	input        img_mounted, // signaling that new image has been mounted
	input [31:0] img_size,    // size of image in bytes

	output[31:0] sd_lba,
	output reg   sd_rd,
	output reg   sd_wr,
	input        sd_ack,

	input  [8:0] sd_buff_addr,
	input  [7:0] sd_buff_dout,
	output [7:0] sd_buff_din,
	input        sd_buff_wr,

	input  [2:0] size_code,  // sector size code
	input        side,
	input        ready       // =1 - disk is present
);

// Possible track configs:
// 0: 26 x 128  = 3.3KB
// 1: 16 x 256  = 4.0KB
// 2:  9 x 512  = 4.5KB
// 3:  5 x 1024 = 5.0KB
// 4: 10 x 512  = 5.0KB

assign dout  = q;
assign drq   = s_drq;
assign busy  = s_busy;
assign intrq = s_intrq;

assign sd_lba = buff_addr[19:9];

reg   [7:0] sectors_per_track, sectors_per_track_var;
wire [10:0] sector_size = 11'd128 << wd_size_code;
reg   [9:0] byte_addr;
reg  [19:0] buff_addr;
reg   [1:0] wd_size_code;

wire  [7:0] buff_dout;

secbuf sbuf
(
	.clock(clk_sys),

	.address_a(sd_buff_addr),
	.data_a(sd_buff_dout),
	.wren_a(sd_buff_wr & sd_ack),
	.q_a(sd_buff_din),

	.address_b(buff_addr[8:0]),
	.data_b(din),
	.wren_b(wre & buff_wr & (addr == A_DATA)),
	.q_b(buff_dout)
);

wire  [7:0] dts = {disk_track[6:0], side};
always @* begin
	case(size_code)
				0: buff_addr = {{1'b0, dts, 4'b0000} + {dts, 3'b000} + {dts, 1'b0} + wdstat_sector - 1'd1, byte_addr[6:0]};
				1: buff_addr = {{dts, 4'b0000}       + wdstat_sector - 1'd1, byte_addr[7:0]};
				2: buff_addr = {{dts, 3'b000}  + dts + wdstat_sector - 1'd1, byte_addr[8:0]};
				3: buff_addr = {{dts, 2'b00}   + dts + wdstat_sector - 1'd1, byte_addr[9:0]};
				4: buff_addr = {{dts, 3'b000}  + {dts, 1'b0} + wdstat_sector - 1'd1, byte_addr[8:0]};
		default: buff_addr = 0;
	endcase
	case(size_code)
				0: sectors_per_track = 26;
				1: sectors_per_track = 16;
				2: sectors_per_track = 9;
				3: sectors_per_track = 5;
				4: sectors_per_track = 10;
		default: sectors_per_track = 0;
	endcase
	case(size_code)
				0: wd_size_code = 0;
				1: wd_size_code = 1;
				2: wd_size_code = 2;
				3: wd_size_code = 3;
				4: wd_size_code = 2;
		default: wd_size_code = 0;
	endcase
end

// Register addresses
parameter A_COMMAND         = 0;
parameter A_STATUS          = 0;
parameter A_TRACK           = 1;
parameter A_SECTOR          = 2;
parameter A_DATA            = 3;

// States
parameter STATE_READY       = 0;	/* Initial, idle, sector data read */
parameter STATE_WAIT_READ   = 1;	/* wait until read operation completes -> STATE_READ_2/STATE_READY */
parameter STATE_WAIT        = 2;	/* NOP operation wait -> STATE_READY */
parameter STATE_ABORT       = 3;	/* Abort current command ($D0) -> STATE_READY */
parameter STATE_READ_2      = 4;	/* Buffer-to-host: wait before asserting DRQ -> STATE_READ_3 */
parameter STATE_READ_3      = 5;	/* Buffer-to-host: load data into reg, assert DRQ -> STATE_READY */
parameter STATE_WAIT_WRITE  = 6;	/* wait until write operation completes -> STATE_READY */
parameter STATE_READ_1      = 7;	/* Buffer-to-host: increment data pointer, decrement byte count -> STATE_READ_2*/
parameter STATE_WRITE_1     = 8;	/* Host-to-buffer: wr = 1 -> STATE_WRITE_2 */
parameter STATE_WRITE_2     = 9;	/* Host-to-buffer: wr = 0, next addr -> STATE_WRITESECT/STATE_WAIT_WRITE */
parameter STATE_WRITESECT   = 10; /* Host-to-buffer: wait data from host -> STATE_WRITE_1 */
parameter STATE_READSECT    = 11; /* Buffer-to-host */
parameter STATE_WAIT_2      = 12;
parameter STATE_ENDCOMMAND  = 14; /* All commands end here -> STATE_ENDCOMMAND2 */
parameter STATE_WAIT_READ_1 = 15;
parameter STATE_WAIT_WRITE_1= 16;

// State variables
reg   [7:0] wdstat_track;
reg   [7:0] wdstat_sector;
reg   [7:0] wdstat_datareg;
reg   [7:0] wdstat_command;			// command register
reg			wdstat_pending;			// command loaded, pending execution
reg 			wdstat_stepdirection;	// last step direction
reg			wdstat_multisector;		// indicates multisector mode

reg   [7:0] disk_track;					// "real" heads position
reg  [10:0]	data_rdlength;				// this many bytes to transfer during read/write ops
reg   [4:0] state;

// common status bits
reg			s_readonly = 0, s_crcerr;
reg			s_headloaded, s_seekerr, s_index;  // mode 1
reg			s_lostdata, s_wrfault; 			     // mode 2,3

// Command mode 0/1 for status register
reg 			cmd_mode;

// DRQ/BUSY are always going together
reg	[1:0]	s_drq_busy;
wire			s_drq  = s_drq_busy[1];
wire			s_busy = s_drq_busy[0];
reg         s_intrq;

// Status register
wire  [7:0] wdstat_status = cmd_mode == 0 ?
	{~ready, s_readonly, s_headloaded, s_seekerr, s_crcerr, !disk_track, s_index, s_busy | wdstat_pending}:
	{~ready, s_readonly, s_wrfault,    s_seekerr, s_crcerr, s_lostdata,  s_drq,   s_busy | wdstat_pending};

// Watchdog
reg	      watchdog_set;
wire	      watchdog_bark;
watchdog_w	dogbert(.clk_sys(clk_sys), .ce(ce), .cock(watchdog_set), .q(watchdog_bark));

reg   [7:0] read_addr[6];
reg   [7:0] q;
always @* begin
	case (addr)
		A_TRACK:  q = wdstat_track;
		A_SECTOR: q = wdstat_sector;
		A_STATUS: q = wdstat_status;
		A_DATA:   q = (state == STATE_READY) ? wdstat_datareg : buff_rd ? buff_dout : read_addr[byte_addr[2:0]];
	endcase
end

reg         buff_rd;
reg         buff_wr;

// Reusable expressions
wire        wStepDir   = wdstat_command[6] ? wdstat_command[5] : wdstat_stepdirection;
wire  [7:0] wNextTrack = wStepDir ? disk_track - 8'd1 : disk_track + 8'd1;
wire [10:0]	wRdLengthMinus1 = data_rdlength - 1'b1;
wire [10:0]	wBuffAddrPlus1  = byte_addr + 1'b1;

always @(posedge clk_sys) begin
	integer cnt;
	if(ce) begin
		if(cnt) cnt <= cnt - 1;
			else cnt <= 35000;
		s_index <= (cnt < 100);
	end
end

wire        rde = rd & io_en;
wire        wre = wr & io_en;
always @(posedge clk_sys or posedge reset) begin
	reg old_wr, old_rd;

	reg [2:0] cur_addr;
	reg       read_data;
	reg       write_data;
	reg       read_type;
	integer   wait_time;
	reg [3:0] read_timer;
	reg [9:0] seektimer;
	reg [5:0] ack;
	reg       sd_busy;

	if(reset) begin
		read_data <= 0;
		write_data <= 0;
		wdstat_multisector <= 0;
		wdstat_stepdirection <= 0;
		disk_track <= 0;
		wdstat_track <= 0;
		wdstat_sector <= 0;
		data_rdlength <= 0;
		byte_addr <=0;
		{buff_rd,buff_wr} <= 0;
		wdstat_multisector <= 0;
		state <= STATE_READY;
		cmd_mode <= 0;
		{s_headloaded, s_seekerr, s_crcerr, s_intrq} <= 0;
		{s_wrfault, s_lostdata} <= 0;
		s_drq_busy <= 0;
		wdstat_pending <= 0;
		watchdog_set <= 0;
		seektimer <= 'h3FF;
		{ack, sd_wr, sd_rd, sd_busy} <= 0;
	end else if(ce) begin

		ack <= {ack[4:0], sd_ack};
		if(ack[5:4] == 'b01) {sd_rd,sd_wr} <= 0;
		if(ack[5:4] == 'b10) sd_busy <= 0;

		old_wr <=wre;
		old_rd <=rde;

		if((!old_rd && rde) || (!old_wr && wre)) cur_addr <= addr;

		//Register read operations
		if(old_rd && !rde && (cur_addr == A_STATUS)) s_intrq <= 0;

		//end of data reading
		if(old_rd && !rde && (cur_addr == A_DATA)) read_data <=1;

		//end of data writing
		if(old_wr && !wre && (cur_addr == A_DATA)) write_data <=1;

		/* Register write operations */
		if (!old_wr & wre) begin
			case (addr)
				A_COMMAND:
					begin
						s_intrq <= 0;
						if(din[7:4] == 'hD) begin
							// interrupt
							cmd_mode <= 0;

							if (state != STATE_READY) state <= STATE_ABORT;
								else {s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;

						end else begin
							if(!wdstat_pending) begin
								wdstat_command <= din;
								wdstat_pending <= 1;
							end
						end
					end

				A_TRACK:  if (!s_busy) wdstat_track <= din;
				A_SECTOR: if (!s_busy) wdstat_sector <= din;
				A_DATA:   wdstat_datareg <= din;
			endcase
		end

		//////////////////////////////////////////////////////////////////
		// Generic state machine is described below, but some important //
		// transitions are defined within the read/write section.       //
		//////////////////////////////////////////////////////////////////

		/* Data transfer: buffer to host. Read stage 1: increment address */
		case (state) 

		/* Idle state or buffer to host transfer */
		STATE_READY:
			begin
				// handle command
				if (wdstat_pending) begin
					wdstat_pending <= 0;
					cmd_mode <= wdstat_command[7];		// keep cmd_mode for wdstat_status
					
					case (wdstat_command[7:4]) 
					4'h0: 	// RESTORE
						begin
							// head load as specified, index, track0
							s_headloaded <= wdstat_command[3];
							wdstat_track <= 0;
							disk_track <= 0;

							// some programs like it when FDC gets busy for a while
							s_drq_busy <= 2'b01;
							state <= STATE_WAIT;
						end
					4'h1:	// SEEK
						begin
							// set real track to datareg
							disk_track <= wdstat_datareg; 
							s_headloaded <= wdstat_command[3];
							
							// get busy 
							s_drq_busy <= 2'b01;
							state <= STATE_WAIT;
						end
					4'h2,	// STEP
					4'h3,	// STEP & UPDATE
					4'h4,	// STEP-IN
					4'h5,	// STEP-IN & UPDATE
					4'h6,	// STEP-OUT
					4'h7:	// STEP-OUT & UPDATE
						begin
							// if direction is specified, store it for the next time
							if (wdstat_command[6] == 1) wdstat_stepdirection <= wdstat_command[5]; // 0: forward/in
							
							// perform step 
							disk_track <= wNextTrack;
									
							// update TRACK register too if asked to
							if (wdstat_command[4]) wdstat_track <= wNextTrack;
								
							s_headloaded <= wdstat_command[3];

							// some programs like it when FDC gets busy for a while
							s_drq_busy <= 2'b01;
							state <= STATE_WAIT;
						end
					4'h8, 4'h9: // READ SECTORS
						// seek data
						// 4: m: 0: one sector, 1: until the track ends
						// 3: S: SIDE
						// 2: E: some 15ms delay
						// 1: C: check side matching?
						// 0: 0
						begin
							// side is specified in the secondary control register ($1C)
							s_drq_busy <= 2'b01;
							{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
							
							wdstat_multisector <= wdstat_command[4];
							data_rdlength <= sector_size;
							read_type <=1;

							state <= STATE_WAIT_READ;
						end
					4'hA, 4'hB: // WRITE SECTORS
						begin
							s_drq_busy <= 2'b01;
							{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
							wdstat_multisector <= wdstat_command[4];
							
							data_rdlength <= sector_size;
							byte_addr <= 0;
							write_data <= 0;
							buff_wr <= 1;

							state <= STATE_WRITESECT;
						end								
					4'hC:	// READ ADDRESS
						begin
							// track, side, sector, sector size code, 2-byte checksum (crc?)
							s_drq_busy <= 2'b01;
							{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;

							wdstat_multisector <= 0;
							state <= STATE_WAIT_READ;
							data_rdlength <= 6;
							read_type <= 0;

							read_addr[0] <= disk_track;
							read_addr[1] <= {7'b0, side};
							read_addr[2] <= wdstat_sector;
							read_addr[3] <= wd_size_code;
							read_addr[4] <= 0;
							read_addr[5] <= 0;
						end
					4'hE,	// READ TRACK
					4'hF:	// WRITE TRACK
						begin
							{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
							//if(wdstat_command[4]) s_wrfault <= 1; // read-only
							s_drq_busy <= 2'b01;
							state <= STATE_WAIT;
						end
					default:s_drq_busy <= 2'b00;
					endcase
				end
			end

		STATE_WAIT_READ:
			begin
				if (!ready) begin
					// FAIL
					s_seekerr <= 1;
					s_crcerr <= 1;
					state <= STATE_ENDCOMMAND;
				end else begin
					sd_busy <= 1;
					sd_rd <= 1;
					seektimer <= 'h3FF;
					state <= STATE_WAIT_READ_1;
				end
			end
		STATE_WAIT_READ_1:
			begin
				if(seektimer) seektimer <= seektimer - 1'b1;
				if(!seektimer && !sd_busy) begin
					if(wdstat_multisector && (wdstat_sector > sectors_per_track)) begin
						if(wdstat_multisector) s_seekerr <= 1;
						wdstat_multisector <= 0;
						state <= STATE_ENDCOMMAND;
					end else begin
						buff_rd <= read_type;
						byte_addr <= 0;
						state <= STATE_READ_2;
					end
				end
			end
		STATE_READ_1:
			begin
				// increment data pointer, decrement byte count
				byte_addr <= wBuffAddrPlus1[9:0];
				data_rdlength <= wRdLengthMinus1[9:0];
				state <= STATE_READ_2;
			end
		STATE_READ_2:
			begin
				watchdog_set <= 1;
				read_timer <= 4'b1111;
				state <= STATE_READ_3;
				s_drq_busy <= 2'b01;
			end
		STATE_READ_3:
			begin
				if(read_timer != 0) 
					read_timer <= read_timer - 1'b1;
				else begin
					read_data <= 0;
					watchdog_set <= 0;
					s_lostdata <= 0;
					s_drq_busy <= 2'b11;
					state <= STATE_READSECT;
				end
			end
		STATE_READSECT:
			begin
				if(watchdog_bark || (read_data && s_drq)) begin
					// reset drq until next byte is read, nothing is lost
					s_drq_busy <= 2'b01;
					s_lostdata <= watchdog_bark;
					
					if (wRdLengthMinus1 == 0) begin
						// either read the next sector, or stop if this is track end
						if (wdstat_multisector) begin
							wdstat_sector <= wdstat_sector + 1'b1;
							data_rdlength <= sector_size;
							state <= STATE_WAIT_READ;
						end else begin
							if(wdstat_multisector) s_seekerr <= 1;
							wdstat_multisector <= 0;
							state <= STATE_ENDCOMMAND;
						end
					end else begin
						// everything is okay, fetch next byte
						state <= STATE_READ_1;
					end
				end
			end

		STATE_WAIT_WRITE:
			begin
				if(!ready) begin
					// FAIL
					s_wrfault <= 1;
					state <= STATE_ENDCOMMAND;
				end else begin
					sd_busy <= 1;
					sd_wr <= 1;
					seektimer <= 'h3FF;
					state <= STATE_WAIT_WRITE_1;
				end
			end
		STATE_WAIT_WRITE_1:
			begin
				if(seektimer) seektimer <= seektimer - 1'b1;
				if(!seektimer && !sd_busy) begin
					if((wdstat_multisector & (wdstat_sector >= sectors_per_track)) | ~wdstat_multisector) begin
						if(wdstat_multisector) s_seekerr <= 1;
						wdstat_multisector <= 0;
						state <= STATE_ENDCOMMAND;
					end else begin
						wdstat_sector <= wdstat_sector + 1'b1;
						byte_addr <= 0;
						state <= STATE_WRITESECT;
					end
				end
			end
		STATE_WRITESECT:
			begin
				watchdog_set <= 1;
				read_timer <= 4'b1111;
				state <= STATE_WRITE_1;
			end
		STATE_WRITE_1:
			begin
				if(read_timer != 0) 
					read_timer <= read_timer - 1'b1;
				else begin
					write_data <= 0;
					watchdog_set <= 0;
					s_lostdata <= 0;
					s_drq_busy <= 2'b11;
					state <= STATE_WRITE_2;
				end
			end
		STATE_WRITE_2:
			begin
				if(watchdog_bark || (write_data && s_drq)) begin
					s_drq_busy <= 2'b01;
					s_lostdata <= watchdog_bark;

					if(!wRdLengthMinus1) state <= STATE_WAIT_WRITE;
					else begin
						byte_addr <= wBuffAddrPlus1[9:0];
						data_rdlength <= wRdLengthMinus1[9:0];
						state <= STATE_WRITESECT;
					end
				end
			end

		// Abort current operation ($D0)
		STATE_ABORT:
			begin
				data_rdlength <= 0;
				wdstat_pending <= 0;
				state <= STATE_ENDCOMMAND;
			end

		STATE_WAIT:
			begin
				wait_time <= 4000;
				state <= STATE_WAIT_2;
			end
		STATE_WAIT_2:
			begin
				if(wait_time) wait_time <= wait_time - 1;
					else state <= STATE_ENDCOMMAND;
			end

		// End any command.
		STATE_ENDCOMMAND:
			begin
				{buff_rd,buff_wr} <= 0;
				state <= STATE_READY;
				s_drq_busy <= 2'b00;
				seektimer <= 10'h3FF;
				s_intrq <= 1;
			end
		endcase
	end
end

endmodule

// start ticking when cock goes down
module watchdog_w
(
	input  clk_sys,
	input  ce,
	input  cock,
	output q
);

parameter TIME = 16'd4096;
assign q = (timer == 0);

reg [15:0] timer;
always @(posedge clk_sys) begin
	if(ce) begin
		if(cock) timer <= TIME;
			else if(timer != 0) timer <= timer - 1'b1;
	end
end

endmodule
