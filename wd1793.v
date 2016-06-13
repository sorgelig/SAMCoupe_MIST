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

module wd1793
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

	input        input_active,
	input [19:0] input_addr,
	input  [7:0] input_data,
	input        input_wr,
	
	// Sector buffer access signals
	input [19:0] buff_size,	 // buffer RAM size (currently not used)
	output[19:0] buff_addr,	 // buffer RAM address
	output       buff_read,	 // buffer RAM read enable
	output       buff_write, // buffer RAM write enable (not tested yet)
	input  [7:0] buff_din,   // buffer RAM data input
	output [7:0] buff_dout,  // buffer RAM data output

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

assign buff_addr  = buff_a;
assign buff_read  = ((addr == A_DATA) && buff_rd);
assign buff_write = ((addr == A_DATA) && buff_wr);
assign buff_dout  = din;

reg   [7:0] sectors_per_track, sectors_per_track_var;
wire [10:0] sector_size = 11'd128 << wd_size_code;
reg   [9:0] byte_addr;
reg  [19:0] buff_a;
reg   [1:0] wd_size_code;

wire  [7:0] dts = {disk_track[6:0], side};
always @* begin
	case(size_code)
				0: buff_a = {{1'b0, dts, 4'b0000} + {dts, 3'b000} + {dts, 1'b0} + wdreg_sector - 1'd1, byte_addr[6:0]};
				1: buff_a = {{dts, 4'b0000}       + wdreg_sector - 1'd1, byte_addr[7:0]};
				2: buff_a = {{dts, 3'b000}  + dts + wdreg_sector - 1'd1, byte_addr[8:0]};
				3: buff_a = {{dts, 2'b00}   + dts + wdreg_sector - 1'd1, byte_addr[9:0]};
				4: buff_a = {{dts, 3'b000}  + {dts, 1'b0} + wdreg_sector - 1'd1, byte_addr[8:0]};
		default: buff_a = sdf_offset + byte_addr;
	endcase
	case(size_code)
				0: sectors_per_track = 26;
				1: sectors_per_track = 16;
				2: sectors_per_track = 9;
				3: sectors_per_track = 5;
				4: sectors_per_track = 10;
		default: sectors_per_track = sectors_per_track_var;
	endcase
	case(size_code)
				0: wd_size_code = 0;
				1: wd_size_code = 1;
				2: wd_size_code = 2;
				3: wd_size_code = 3;
				4: wd_size_code = 2;
		default: wd_size_code = sdf_sizecode;
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
parameter STATE_SDF_SEARCH  = 15;
parameter STATE_SDF_SEARCH_1= 16;

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

reg   [7:0] wdreg_track;
reg   [7:0] wdreg_sector;
reg   [7:0] wdreg_data;
reg   [7:0] wdreg_command;
wire  [7:0] wdreg_status = cmd_mode == 0 ?
	{~ready, s_readonly, s_headloaded, s_seekerr, s_crcerr, !disk_track, s_index, s_busy | pending}:
	{~ready, s_readonly, s_wrfault,    s_seekerr, s_crcerr, s_lostdata,  s_drq,   s_busy | pending};

// Watchdog
reg	      watchdog_set;
wire	      watchdog_bark;
watchdog	dogbert(.clk_sys(clk_sys), .ce(ce), .cock(watchdog_set), .q(watchdog_bark));

reg   [7:0] read_addr[6];
reg   [7:0] q;
always @* begin
	case (addr)
		A_STATUS: q = wdreg_status;
		A_TRACK:  q = wdreg_track;
		A_SECTOR: q = wdreg_sector;
		A_DATA:   q = (state == STATE_READY) ? wdreg_data : buff_rd ? buff_din : read_addr[byte_addr[2:0]];
	endcase
end

reg         buff_rd;
reg         buff_wr;
reg			pending;			  // command loaded, pending execution
reg 			step_direction;  // last step direction

reg   [7:0] disk_track;					// "real" heads position
reg  [10:0]	data_length;				// this many bytes to transfer during read/write ops
reg   [4:0] state, pending_state;

// Reusable expressions
wire  [7:0] next_track  = (wdreg_command[6] ? wdreg_command[5] : step_direction) ? disk_track - 1'd1 : disk_track + 1'd1;
wire [10:0]	next_length = data_length - 1'b1;
wire [10:0]	next_addr   = byte_addr + 1'b1;

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
	reg [7:0] ra_sector;
	reg       multisector;
	reg       compare_sector;

	if(reset) begin
		read_data <= 0;
		write_data <= 0;
		multisector <= 0;
		step_direction <= 0;
		disk_track <= 0;
		wdreg_track <= 0;
		wdreg_sector <= 0;
		data_length <= 0;
		byte_addr <=0;
		{buff_rd,buff_wr} <= 0;
		multisector <= 0;
		state <= STATE_READY;
		cmd_mode <= 0;
		{s_headloaded, s_seekerr, s_crcerr, s_intrq} <= 0;
		{s_wrfault, s_lostdata} <= 0;
		s_drq_busy <= 0;
		pending <= 0;
		watchdog_set <= 0;
		seektimer <= 10'h3FF;
		ra_sector <= 1;
	end else if(ce) begin
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
							if(!pending) begin
								wdreg_command <= din;
								pending <= 1;
							end
						end
					end

				A_TRACK:  if (!s_busy) wdreg_track <= din;
				A_SECTOR: if (!s_busy) {ra_sector, wdreg_sector} <= {din,din};
				A_DATA:   wdreg_data <= din;
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
				if (pending) begin
					pending <= 0;
					cmd_mode <= wdreg_command[7];		// keep cmd_mode for wdreg_status
					
					case (wdreg_command[7:4]) 
					4'h0: 	// RESTORE
						begin
							// head load as specified, index, track0
							s_headloaded <= wdreg_command[3];
							wdreg_track <= 0;
							disk_track <= 0;

							// some programs like it when FDC gets busy for a while
							s_drq_busy <= 2'b01;
							state <= STATE_WAIT;
						end
					4'h1:	// SEEK
						begin
							// set real track to datareg
							disk_track <= wdreg_data; 
							s_headloaded <= wdreg_command[3];
							
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
							if (wdreg_command[6] == 1) step_direction <= wdreg_command[5]; // 0: forward/in
							
							// perform step 
							disk_track <= next_track;
									
							// update TRACK register too if asked to
							if (wdreg_command[4]) wdreg_track <= next_track;
								
							s_headloaded <= wdreg_command[3];

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
							
							multisector <= wdreg_command[4];
							data_length <= sector_size;
							read_type <=1;

							if(size_code >= 5) begin
								pending_state <= STATE_WAIT_READ;
								state <= STATE_SDF_SEARCH;
								sdf_start <= 0;
							end else begin
								state <= STATE_WAIT_READ;
							end
						end
					4'hA, 4'hB: // WRITE SECTORS
						begin
							s_drq_busy <= 2'b11;
							{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
							multisector <= wdreg_command[4];
							
							data_length <= sector_size;
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

							multisector <= 0;
							data_length <= 6;
							read_type <= 0;

							if(size_code >= 5) begin
								sdf_start    <= sdf_next;
								pending_state<= STATE_WAIT_READ;
								state        <= STATE_SDF_SEARCH;
							end else begin
								read_addr[0] <= disk_track;
								read_addr[1] <= {7'b0, side};
								read_addr[2] <= ra_sector;
								read_addr[3] <= wd_size_code;
								read_addr[4] <= 0;
								read_addr[5] <= 0;
								if(ra_sector >= sectors_per_track) ra_sector <= 1;
									else ra_sector <= ra_sector + 1'd1;
								state <= STATE_WAIT_READ;
							end
						end
					4'hE,	// READ TRACK
					4'hF:	// WRITE TRACK
						begin
							{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
							if(wdreg_command[4]) s_wrfault <= 1; // read-only
							s_drq_busy <= 2'b01;
							state <= STATE_WAIT;
						end
					default:s_drq_busy <= 2'b00;
					endcase
				end
			end

		STATE_SDF_SEARCH:
			begin
				sdf_addr <= sdf_start;
				state    <= STATE_SDF_SEARCH_1;
				spt_addr <= (side ? spt_size>>1 : 8'd0) + disk_track;
			end

		STATE_SDF_SEARCH_1:
			begin
				if(read_type & (sdf_track == disk_track) & 
					            (sdf_side == side) &
					            (sdf_sector == wdreg_sector))
				begin
					state <= pending_state;
				end
				else
				if(~read_type & (sdf_track == disk_track) & 
					             (sdf_side == side))
				begin
					state        <= pending_state;
					read_addr[0] <= sdf_trackf;
					read_addr[1] <= sdf_sidef;
					read_addr[2] <= sdf_sector;
					read_addr[3] <= sdf_sizecode;
					read_addr[4] <= sdf_crc1;
					read_addr[5] <= sdf_crc2;
				end
				else
				if(sdf_next == sdf_start) begin
					state <= STATE_WAIT;
					s_seekerr <= 1;
				end else begin
					sdf_addr <= sdf_next;
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
					seektimer <= seektimer - 1'b1;
					if(!seektimer) begin
						if(multisector && (wdreg_sector > sectors_per_track)) begin
							if(multisector) s_seekerr <= 1;
							multisector <= 0;
							state <= STATE_ENDCOMMAND;
						end else begin
							buff_rd <= read_type;
							byte_addr <= 0;
							state <= STATE_READ_2;
						end
					end
				end
			end
		STATE_READ_1:
			begin
				// increment data pointer, decrement byte count
				byte_addr <= next_addr[9:0];
				data_length <= next_length[9:0];
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
					
					if (next_length == 0) begin
						// either read the next sector, or stop if this is track end
						if (multisector) begin
							wdreg_sector <= wdreg_sector + 1'b1;
							data_length <= sector_size;
							if(size_code >= 5) begin
								pending_state <= STATE_WAIT_READ;
								state <= STATE_SDF_SEARCH;
							end else begin
								state <= STATE_WAIT_READ;
							end
						end else begin
							if(multisector) s_seekerr <= 1;
							multisector <= 0;
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
				if (!ready) begin
					s_wrfault <= 1;
					state <= STATE_ENDCOMMAND;
				end else begin
					if (multisector && wdreg_sector < sectors_per_track) begin
						wdreg_sector <= wdreg_sector + 1'b1;
						s_drq_busy <= 2'b11;
						data_length <= sector_size;
						byte_addr <= 0;
						state <= STATE_WRITESECT;
					end else begin
						multisector <= 0;
						state <= STATE_ENDCOMMAND;
					end
				end
			end
		STATE_WRITESECT:
			begin
				if (write_data) begin
					s_drq_busy <= 2'b01;			// busy, clear drq
					s_lostdata <= 0;
					state <= STATE_WRITE_2;
					write_data <= 0;
				end
			end
		STATE_WRITE_2:
			begin
				// increment data pointer, decrement byte count
				byte_addr <= next_addr[9:0];
				data_length <= next_length;
								
				if (next_length == 0) begin
					// Flush data --
					state <= STATE_WAIT_WRITE;
				end else begin
					s_drq_busy <= 2'b11;		// request next byte
					state <= STATE_WRITESECT;
				end				
			end

		// Abort current operation ($D0)
		STATE_ABORT:
			begin
				data_length <= 0;
				pending <= 0;
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

reg  [1:0] sdf_sizecode;      // sector size: 0=128K, 1=256K, 2=512K, 3=1024K
reg        sdf_side;          // Side number (0 or 1)
reg  [6:0] sdf_track;         // Track number
reg  [7:0] sdf_sector;        // Sector number 0..15
reg  [7:0] sdf_crc1;          // ID field CRC MSB
reg  [7:0] sdf_crc2;          // ID field CRC LSB
reg [19:0] sdf_offset;
reg  [7:0] sdf_trackf, sdf_sidef;

wire[10:0] sdf_next = ((sdf_addr + 1'd1) >= sdf_size) ? 11'd0 : sdf_addr + 1'd1;
reg [10:0] sdf_size;
reg [10:0] sdf_addr, sdf_start;
reg [69:0] sdf[1992];

reg  [7:0] spt_size;
reg  [7:0] spt_addr;
reg  [7:0] spt[166];

always @(posedge clk_sys) begin
	{sdf_track,sdf_side,sdf_trackf,sdf_sidef,sdf_sector,sdf_sizecode,sdf_crc1,sdf_crc2,sdf_offset} <= sdf[sdf_addr];
	sectors_per_track_var <= spt[spt_addr];
end

always @(posedge clk_sys) begin
	reg old_active, old_wr;
	reg [13:0] hdr_pos, bcnt;
	reg  [7:0] idStatus;
   reg  [6:0] track;
   reg        side;
   reg  [7:0] sector;
   reg  [1:0] sizecode;
   reg  [7:0] crc1;
   reg  [7:0] crc2;
	reg  [7:0] sectors;
	reg        is_sdf;
	reg  [7:0] tsize[166];
	reg [15:0] track_size, track_pos;
	reg [19:0] offset, offset1;
	reg  [7:0] size_lo;
	reg  [7:0] pos;
	reg [10:0] secpos;
	reg  [7:0] trackf, sidef;

	old_active <= input_active;
	if(input_active & ~old_active) begin
		sdf_size  <=0;
		spt_size  <=0;
		track_pos <=0;
		is_sdf    <=0;
	end

	old_wr <= input_wr;
	if(input_wr & ~old_wr & input_active) begin
		if((!input_addr && (input_data != 'h45)) | is_sdf) begin
			is_sdf <= 1;
			track_pos <= track_pos + 1'b1;
			if(track_pos == 6143) track_pos <= 0;

			if(!track_pos) begin
				sectors <= input_data;
				spt[spt_size] <= input_data;
				spt_size <= spt_size + 1'd1;
				hdr_pos <= 0;
				bcnt    <= 0;
			end else if(sectors) begin
				hdr_pos <= hdr_pos + 1'd1;
				case(hdr_pos)
					0: idStatus <= input_data;
					1: /*dataStatus <= input_data*/;
					2: track <= input_data[6:0];
					3: side <= input_data[0];
					4: sector <= input_data;
					5: sizecode <= input_data[1:0];
					6: crc1 <= input_data;
					7: begin
							if(idStatus != 0) begin 
								sectors <= sectors - 1'd1;
								hdr_pos <= 0;
								sdf[sdf_size] <= {track,side,1'b0,track,7'b0,side,sector,sizecode,crc1,input_data,20'd0};
								sdf_size <= sdf_size + 1'd1;
							end
							crc2 <= input_data;
							bcnt <= 11'd128 << sizecode;
						end
					8: begin
							sdf[sdf_size] <= {track,side,1'b0,track,7'b0,side,sector,sizecode,crc1,crc2,input_addr};
							sdf_size <= sdf_size + 1'd1;
							bcnt     <= bcnt - 1'd1;
						end
					default: begin
						bcnt <= bcnt - 1'd1;
						if(bcnt == 1) begin
							sectors <= sectors - 1'd1;
							hdr_pos <= 0;
						end
					end
				endcase
			end
		end else begin
			if( input_addr == 48) spt_size <= input_data; else
			if((input_addr == 49) & (input_data == 2)) spt_size <= spt_size << 1; else
			if( input_addr == 52) begin
				track_size <= {input_data, 8'd0};
				track_pos  <= 0;
				pos <= 1;
			end else
			if((input_addr  > 52) & (input_addr < 218)) begin
				tsize[input_addr - 52] <= input_data;
				spt[input_addr - 52] <= 0;
			end else 
			if((input_addr >= 256) && track_size) begin
				track_pos <= track_pos + 1'd1;
				case(track_pos)
					00: offset  <= input_addr + 9'd256;
					16: track   <= input_data[6:0];
					17: side    <= input_data[0];
					21: sectors <= input_data;
					22: spt[(side ? (spt_size >> 1) : 8'd0) + track] <= sectors;
					default:
						if((track_pos >= 24) && sectors) begin
							case(track_pos[2:0])
								0: begin
										trackf  <= input_data;
										secpos  <= sdf_size;
										offset1 <= offset;
									end
								1: sidef   <= input_data;
								2: sector  <= input_data;
								3: sizecode<= input_data[1:0];
								6: size_lo <= input_data;
								7: begin
										if({input_data, size_lo}) begin
											sdf[secpos] <= {track,side,trackf,sidef,sector,sizecode,8'd0,8'd0,offset1};
											sdf_size <= sdf_size + 1'd1;
											offset <= offset + {input_data, size_lo};
										end
										sectors <= sectors - 1'd1;
									end
								default:;
							endcase
						end
				endcase
				if(track_pos >= (track_size - 1'd1)) begin
					track_size <= {tsize[pos], 8'd0};
					track_pos  <= 0;
					pos <= pos + 1'd1;
				end
			end
		end
	end
end

endmodule

// start ticking when cock goes down
module watchdog
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
