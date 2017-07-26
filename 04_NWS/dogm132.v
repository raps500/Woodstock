/* Handles the display
 * DOGM132 132x32 graphic display
 * Glyphs are stored in ROM
 * 15 digits of 8 pixels
 * The decimal point in the 67 occupies one digit
 */



module ws_display_dogm132(
	input wire			clk_in,
    input wire          reset_in,
	input wire			op_disp_off_in,
	input wire			op_disp_toggle_in,
	input wire	[3:0]	ra_in,
	input wire	[3:0]	rb_in,
    input wire          seq_fetch_op0,      // asserted when data for the display is to be latched/used
    output wire [3:0]   disp_curr_nibble_o, // controls which nibble should be output now
	output wire	      	disp_cs_n_o,
	output wire         disp_res_n_o,
    output wire         disp_data_o,
    output wire         disp_addr_o,
    output wire         disp_sck_o,
	output wire         disp_acq_o
	);
	
reg [5:0] state = 5'h0;
reg [3:0] scanned_digit = 4'h0; // index used to extract the information fromA and B
reg [3:0] digit_position = 4'h0; // display position
reg [2:0] x = 3'h0;
reg [4:0] curr_bit = 5'h0;
reg ss = 1'b0;
reg dispon = 1'b0;
reg send_ready = 1'b0;
reg disp_sck = 1'b0;
reg clock_active = 1'b0;
reg [3:0] glyph_addr;
reg [3:0] glyph_addr_latched = 4'h0;
reg [7:0] out_data = 8'h0;
reg [7:0] cmddata = 8'h0;
wire [7:0] glyph_data;
reg force_refresh = 1'h0;
reg [7:0] glyphs [127:0];
reg [7:0] clear_cnt = 8'h0;

`define ST_RESET            6'h00
`define ST_INIT_LAST        6'h11
`define ST_CLR_DISP0        6'h12
`define ST_CLR_DISP1        6'h16
`define ST_CLR_DISP2        6'h1A
`define ST_CLR_DISP3        6'h1E
`define ST_WAIT_FOR_TOGGLE  6'h1F
`define ST_SET_ADDR         6'h20
`define ST_PRE_REFRESH      6'h22
`define ST_WAIT_FOR_DATA    6'h23
`define ST_REFRESH          6'h24
`define ST_SLEEP_CMD        6'h25
`define ST_POWER_DOWN       6'h26
`define ST_WAKE_UP          6'h27

assign disp_sck_o = disp_sck;
assign disp_cs_n_o = ss;

assign disp_data_o = out_data[7]; // MSB first
assign disp_addr_o = (state == `ST_REFRESH) || (state == `ST_WAIT_FOR_DATA) ||
                     (state == `ST_CLR_DISP0) || (state == `ST_CLR_DISP1) || 
                     (state == `ST_CLR_DISP2) || (state == `ST_CLR_DISP3);
assign disp_res_n_o = reset_in;

assign disp_curr_nibble_o = scanned_digit;

assign disp_acq_o = force_refresh;

// set address 

// [11:8] = 4'h1  3.45
// [11:8] = 4'h0 -3.45      entry
// [11:8] = 4'h3  3.45e-45  entry
// [11:8] = 4'h2 -3.45e-45  entry
// [11:8] = 4'h8 -3.45e-45  after enter
// [11:8] = 4'h9  3.45e-45  after enter
//

always @(*)
    begin
        if (rb_in == 4'h3) // decimal point
            glyph_addr <= 4'hb;
        else
            case (digit_position)
                4'd0: glyph_addr <= (rb_in == 4'h2) ? 4'hf:ra_in; // blank if 2
                4'd1: glyph_addr <= (rb_in == 4'h2) ? 4'hf:ra_in; // blank if 2
                4'd2: glyph_addr <= ((ra_in == 4'h3) || (ra_in == 4'h8) || (ra_in == 4'h9)) ? 4'hd:4'hf;
                4'd3: glyph_addr <= ra_in;
                4'd4: glyph_addr <= ra_in;
                4'd5: glyph_addr <= ra_in;
                4'd6: glyph_addr <= ra_in;
                4'd7: glyph_addr <= ra_in;
                4'd8: glyph_addr <= ra_in;
                4'd9: glyph_addr <= ra_in;
                4'd10:glyph_addr <= ra_in;
                4'd11:glyph_addr <= ra_in;
                4'd12:glyph_addr <= ra_in;
                4'd13:glyph_addr <= ra_in;
                4'd14:glyph_addr <= ((ra_in == 4'h0) || (ra_in == 4'h2) || (ra_in == 4'h8)) ? 4'hd:4'hf; // used for minus sign
                default: glyph_addr <= 4'hf; // blank default;
            endcase
    end

assign glyph_data = glyphs[{ glyph_addr_latched, x } ];

always @(posedge clk_in)
    begin
        if (~reset_in)
            begin
                state <= 5'h0;
                ss <= 1'b1;
                send_ready <= 1'b1;
                scanned_digit <= 4'h0;
                digit_position <= 4'h0;
                x <= 3'h7;
                curr_bit <= 5'h0;
				force_refresh <= 1'b0;
                clear_cnt <= 8'h0;
            end
        else
            begin
				if (op_disp_off_in)
					begin
						dispon <= 1'b0;
					end
                if (op_disp_toggle_in)
                    begin
                        if (dispon)
                            dispon <= 1'b0;
                        else
                            begin
                                dispon <= 1'b1;
                                force_refresh <= 1'b1;
                            end
                    end
				if (clock_active)
                    disp_sck <= ~disp_sck;
                else
                    disp_sck <= 1'b0;
                case (state)
                    `ST_WAIT_FOR_TOGGLE: 
                        begin 
                            if (force_refresh)
                                begin
                                    state <= state + 5'd1;
                                    scanned_digit <= 4'h0; // refresh is from right to left exponent first
                                    digit_position <= 4'h0;
                                    dispon <= dispon ^ 1'b1;
                                    x <= 3'h7; // from right to left because the column 0 is on the right
                                    force_refresh <= 1'b0;
                                    curr_bit <= 5'h0;
                                end
                        end
                    `ST_WAIT_FOR_DATA:
                        if (seq_fetch_op0)
                            begin
                                glyph_addr_latched <= glyph_addr;
                                curr_bit <= 5'h0;
                                state <= state + 5'd1;
                                send_ready <= 1'b0;
                                x <= 3'h7;
                                if (digit_position == 4'hd) // display sign
                                    begin
                                        scanned_digit <= 4'h2;
                                    end
                                else
                                    scanned_digit <= scanned_digit + 4'd1;
                                digit_position <= digit_position + 4'h1;
                            end
                    `ST_REFRESH:
                        if (send_ready == 1'b1)
                            begin
                                if (x == 3'd0)
                                    begin
                                        if (digit_position == 4'he)
                                            state <= `ST_WAIT_FOR_TOGGLE;
                                        else
                                            state <= `ST_WAIT_FOR_DATA;
                                    end
                                else
                                    begin
                                        x <= x - 3'd1; // from right to left because the column 0 is on the right
                                        curr_bit <= 5'h0;
                                        send_ready <= 1'b0;
                                    end
                            end
                    `ST_CLR_DISP0, `ST_CLR_DISP1, `ST_CLR_DISP2, `ST_CLR_DISP3:
                        begin
                            if (send_ready == 1'b1)
                                begin
                                    if (clear_cnt == 8'd131)
                                        begin
                                            clear_cnt <= 8'd0;
                                            state <= state + 6'd1;
                                        end
                                    else
                                        clear_cnt <= clear_cnt + 8'd1;
                                    curr_bit <= 5'h0;
                                    send_ready <= 1'b0;
                                end
                        end
                    default:
                        if (send_ready == 1'b1)
                            begin
                                state <= state + 6'd1;
                                curr_bit <= 5'h0;
                                send_ready <= 1'b0;
                            end
                endcase
                // only send when curr_bit is < 19
                if (~send_ready)
                    begin
                        case (curr_bit)
                            5'd0: out_data <= cmddata; // data to shift
                            5'd1: begin clock_active <= 1'b1; ss <= 1'b0; end
                            5'd3, 4'h5, 4'h7, 4'h9,
                            5'd11, 5'd13, 5'd15: out_data <= out_data << 1;
                            5'd16: begin clock_active <= 1'b0; end
                            5'd18: begin ss <= 1'b1; send_ready <= 1'b1; end
                        endcase
                        curr_bit <= curr_bit + 5'h1;
                    end

            end
    end

    
// command/data according to the state machine
always @(*)
    begin
        cmddata = 8'h00;
        case (state)
            6'h00: cmddata = 8'h00;
            6'h01: cmddata = 8'h40;
            6'h02: cmddata = 8'ha1; /* A1 for inverted display */
            6'h03: cmddata = 8'hc0; /* C0 for inverted display */
            6'h04: cmddata = 8'ha6;
            6'h05: cmddata = 8'ha2;
            6'h06: cmddata = 8'h2f;
            6'h07: cmddata = 8'hf8;
            6'h08: cmddata = 8'h00;
            6'h09: cmddata = 8'h23;
            6'h0a: cmddata = 8'h81;
            6'h0b: cmddata = 8'h1f;
            6'h0c: cmddata = 8'hac;
            6'h0d: cmddata = 8'h00;
            6'h0e: cmddata = 8'haf;
            6'h0f: cmddata = 8'hb0; // page address 0 // sets addresses
            6'h10: cmddata = 8'h10; // col address high 0
            6'h11: cmddata = 8'h00; // col address low 0
            6'h12: cmddata = 8'h00; // data 0 to clear whole display
            6'h13: cmddata = 8'hb1; // page address 0 // sets addresses
            6'h14: cmddata = 8'h10; // col address high 0
            6'h15: cmddata = 8'h00; // col address low 0
            6'h16: cmddata = 8'h00; // data 0 to clear whole display
            6'h17: cmddata = 8'hb2; // page address 0 // sets addresses
            6'h18: cmddata = 8'h10; // col address high 0
            6'h19: cmddata = 8'h00; // col address low 0
            6'h1A: cmddata = 8'h00; // data 0 to clear whole display
            6'h1B: cmddata = 8'hb3; // page address 0 // sets addresses
            6'h1C: cmddata = 8'h10; // col address high 0
            6'h1D: cmddata = 8'h00; // col address low 0
            6'h1E: cmddata = 8'h00; // data 0 to clear whole display
            6'h1F: cmddata = 8'hb2; // no command, waits for toggle, command is latched on this stage
            6'h20: cmddata = 8'hb2; // page address 0 // sets addresses
            6'h21: cmddata = 8'h10; // col address high 0
            6'h22: cmddata = 8'h06; // col address low 0
            6'h23: cmddata = glyph_data;
            6'h24: cmddata = glyph_data;
            6'h25: cmddata = 8'hae; // power save
            6'h26: cmddata = 8'h00; // No command
            6'h27: cmddata = 8'haf; // wake up
            default:
                   cmddata = 8'h00;
        endcase
    end
    


/* Graphic display State Machine */
/*
Befehl RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0 Hex Bemerkung Function Set 
Command                     | A0 D7 D6 D5 D4 D3 D2 D1 D0 Hex Remark 
Display start line set      |  0  0  1  0  0  0  0  0  0 $40 Display start line 0 
ADC set                     |  0  1  0  1  0  0  0  0  1 $A1 ADC reverse *) 
Common output mode select   |  0  1  1  0  0  0  0  0  0 $C0 Normal COM0~COM31 
Display normal/reverse      |  0  1  0  1  0  0  1  1  0 $A6 Display normal 
LCD bias set                |  0  1  0  1  0  0  0  1  0 $A2 Set bias 1/9 (Duty 1/33) 
Power control set           |  0  0  0  1  0  1  1  1  1 $2F Booster, Regulator and Follower on
Booster ratio set           |  0  1  1  1  1  1  0  0  0 $F8 Set internal Booster to 3x / 4x
                            |     0  0  0  0  0  0  0  0 $00 
V0 voltage regulator set    |  0  0  0  1  0  0  0  1  1 $23 Contrast set
Electronic volume mode set  |  0  1  0  0  0  0  0  0  1 $81 
                            |     0  0  0  1  1  1  1  1 $1F
Static indicator set        |  0  1  0  1  0  1  1  0  0 $AC No indicator  
No indicator                |     0  0  0  0  0  0  0  0 $00 
Display ON/OFF              |  0  1  0  1  0  1  1  1  1 $AF Display on

 */   
    
initial
	begin
        $readmemh("glyphs.hex", glyphs);
	end
	
endmodule