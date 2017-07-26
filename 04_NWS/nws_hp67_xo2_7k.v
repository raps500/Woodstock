/* Woodstock - Top file
 *
 * Woodstock 
 * 10-bit sync memory for program
 *
 * (c) Copyright 2014-2017 R.A. Paz S.
 * 
 */
`include "ws_defs.v" 
`default_nettype none
`timescale 1ns/1ns

module ws_hp67_xo2_7k #(
    parameter [0:0] MXO2 = 0,
    parameter [0:0] MAX10 = 0,
    parameter [0:0] HP67 = 1,
    parameter [0:0] HW_TRACE = 1,
    parameter [0:0] HW_TRACE_MINI = 0,
    parameter [0:0] DISP_DOGM132 = 1,
    parameter [0:0] DISP_SSD1603 = 0    
)
(
    input wire          clk_in,     // only used in Simulation or Max 10 variant
    output wire         disp_cs_n_o,
    output wire         disp_addr_o,
    output wire         disp_sck_o,
    output wire         disp_data_o,
    output wire         disp_res_n_o,
    
    inout wire          col0l_o,
    inout wire          col1l_o,
    inout wire          col2lt_o, // only top key
    inout wire          col2l_o,  // three key below, power together
    inout wire          col3l_o,
    inout wire          col4l_o,
    inout wire          col0r_o,
    inout wire          col1r_o,
    inout wire          col2r_o,
    inout wire          col3r_o,
    inout wire          col4r_o,
    
    input wire [3:0]    rowsl_in,
    input wire [3:0]    rowsr_in,
    output wire         txd_o
`ifdef SIMULATOR
    ,
    input wire          simkbd_activate_key_pending_in,
    input wire [7:0]    simkbd_keycode_in
`endif
	,
	output wire [23:0]  debug_o
    );

/* Reset and clocking */
reg cpu_reset_n = 1'b0;
reg [2:0] reset_cnt = 3'h0;
wire osc_clk, cpu_clk, tx_clk;

/* Decoder */

reg [3:0] dec_op_alu;
reg dec_f_set_carry;
reg [6:0] dec_reg_op1;
reg [6:0] dec_reg_dst;
reg [3:0] dec_lit_operand;
reg [3:0] dec_field_start;
reg [3:0] dec_field_end;

reg dec_f_bank;
reg dec_f_crc_0;
reg dec_f_crc_6;
reg dec_f_clr_status;
reg dec_f_clr_flag;
reg dec_f_delayed;
reg dec_f_disp_toggle;
reg dec_f_disp_off;
reg dec_f_gto_c_clr;
reg dec_f_gto_c_set;
reg dec_f_jump;
reg dec_f_gonc;
reg dec_f_use_delay_rom;
reg dec_f_nop;
reg dec_f_pop_pc;
reg dec_f_push_pc;
reg [2:0] dec_p_func;
reg dec_f_set_hex;
reg dec_f_set_dec;
reg dec_f_set_flag;
reg dec_f_test_clr_flag;
reg dec_f_test_set_flag;
reg dec_f_kbd_ack;
reg dec_f_set_carry_early;
reg [11:0] dec_jaddr;
reg dec_f_rol      ;
reg dec_f_lsl      ;
reg dec_f_lsr      ;
reg dec_f_tfr      ;
reg dec_f_exchange ;
reg dec_f_wr_alu   ;
reg dec_f_wr_lit   ; // load constant into C
reg dec_f_push_c   ;
reg dec_f_pop_a    ;
reg dec_f_clr_regs ;
reg dec_f_clr_dregs;
reg dec_f_rot_stack;
// CRC opcodes
reg dec_f_crc_060;
reg dec_f_crc_100; // 
reg dec_f_crc_160; // 
reg dec_f_crc_260;
reg dec_f_crc_300; // check prgm keys
reg dec_f_crc_360; // 
reg dec_f_crc_400;
reg dec_f_crc_500; // 
reg dec_f_crc_560; // 
reg dec_f_crc_660;
reg dec_f_crc_760;
reg dec_f_crc_1000; // set normal keys for first row (cleared) or labels (set)
reg dec_f_crc_1100; // test for normal keys (s=1) for first row or labels (s=0)
reg dec_f_crc_1200; // 
reg dec_f_crc_1300; // 
reg dec_f_crc_1400; // 
reg dec_f_crc_1500; // 
reg dec_f_crc_1700; // 

// Opcode field translators

wire [3:0] dec_lit_ldi_p, dec_lit_cmp_p;
wire [3:0] dec_op_field_start, dec_op_field_end;

// Registers
reg [3:0] SPC = 4'h0;
reg [11:0] RPC = 12'h0;             // Program counter
reg [11:0] STACK [1:0];
reg [3:0] RP = 4'h0;                // Pointer register
reg [5:0] DATAADDR = 6'h0;          // Data address register
reg [3:0] RF = 4'h0;                // F Register
reg [11:4] RA;                      // copy of RA[11:4] for jump to a
reg f_carry = 1'b0;                 // result from current instrction
reg f_latched_carry = 1'b0;         // result from last instruction
reg f_decimal = 1'b0;               // Decimal mode, starts in hexadecimal mode
wire [15:0] f_hw_status;
reg f_hw_status_14, f_hw_status_13, f_hw_status_12;
reg f_hw_status_11, f_hw_status_10,  f_hw_status_9,  f_hw_status_8;
reg f_hw_status_7,  f_hw_status_6,   f_hw_status_4;
reg f_hw_status_2,  f_hw_status_1,   f_hw_status_0;
reg batt_status = 1'b0;
reg bank = 1'b0;
reg [3:0] delayed_rom = 3'h0;
reg delayed_rom_active = 1'b0;

reg f_crc_status = 1'b0;
reg f_crc_disp_mode = 1'b0;
reg f_first_row_pgm = 1'b0;
reg f_crc_key = 1'b0;
reg f_crc_merge = 1'b0;
reg f_crc_pause = 1'b0;
// Register P outputs

reg reg_p_set_carry;
reg p_crossed_inc = 1'b0;
reg p_crossed_dec = 1'b0;
// Registers and Arithmetic outputs

(* syn_keep=1 *) wire [10:0] regbank_addr_a;
(* syn_keep=1 *) wire [10:0] regbank_addr_b;

(* syn_keep=1 *) wire [3:0] reg_path_a;
(* syn_keep=1 *) wire [3:0] reg_path_b;
(* syn_keep=1 *) wire [3:0] data_from_regbank_path_a;
(* syn_keep=1 *) wire [3:0] data_from_regbank_path_b;
(* syn_keep=1 *) wire [3:0] data_to_reg_b;


// ALU registers and signals
wire [3:0] alu_out;             // alu output to register bank
wire alu_set_carry;
// intermediate values
wire [3:0] alu_q_add, alu_q_sub, alu_q_rsub;

wire alu_qc_add, alu_qc_sub, alu_qc_rsub;

/* compare unit */
reg [2:0] alu_comp_res = 3'h0;
wire alu_eq, alu_neq, alu_gt;
reg alu_icarry = 1'b0;

// Sequencer outputs
// these outputs are only active for one, sometimes more than one state of the sequencer

wire seq_fetch_op0;
wire seq_fetch_op1;
wire seq_fetch_op2;
wire seq_decode;
wire seq_exe;
wire seq_jump;
wire seq_alu_prep;
wire seq_alu_wback;
wire seq_alu_read3;


wire seq_reg_transfer;
wire seq_reg_write_alu_or_lit;
wire seq_reg_read;
wire seq_reg_exchange;

(* syn_keep=1 *) reg [6:0] seq_op1;             // operand 1 name from sequencer
(* syn_keep=1 *) reg [6:0] seq_dst;             // operand 2 and destination name from sequencer
(* syn_keep=1 *) reg [3:0] seq_op1_nibble;      // operand 1 current nibble from sequencer
(* syn_keep=1 *) reg [3:0] seq_dst_nibble;      // operand 2/destination current nibble from sequencer

reg seq_inc_da = 1'b0;          // increment data address when clearing group of 16 data registers
reg seq_fetch_op1_delayed = 1'b0;
reg [9:0] seq_opcode = 10'h0;   // latched opcode
reg [9:0] seq_opcode2 = 10'h0;   // latched opcode 2nd word
reg [11:0] seq_fetched_addr;    // address of the  current fetched opcode 1st word if multiword
reg [11:0] seq_ifaddr = 12'h0;  // target address of the two word if opcode
reg [3:0] seq_da_reg_cnt;
reg seq_wback_carry = 1'b0;

reg [3:0] seq_state = `ST_INIT;
reg seq_field_ofs = 1'h0;
reg [3:0] seq_field_counter= 4'h0;

assign seq_fetch_op0 = seq_state == `ST_FETCH_OP0;
assign #100 seq_fetch_op1 = seq_state == `ST_FETCH_OP1;
assign seq_fetch_op2 = seq_state == `ST_FETCH_OP2;
assign seq_decode = seq_state == `ST_DECODE;
assign seq_exe = seq_state == `ST_EXE_NORM;
assign seq_jump = seq_state == `ST_JUMP;
assign seq_alu_prep = seq_state == `ST_AT_PREP;
assign seq_alu_read3 = seq_state == `ST_AT_READ3;
assign seq_alu_wback = seq_state == `ST_AT_WBACK;
// these three signals are not state dependant
assign seq_reg_transfer = dec_f_tfr | dec_f_clr_regs | dec_f_rol | dec_f_lsl | dec_f_lsr |
                          dec_f_clr_dregs | dec_f_pop_a | dec_f_push_c;

assign seq_reg_write_alu_or_lit = (dec_op_alu == `ALU_ADD) | (dec_op_alu == `ALU_SUB) | (dec_op_alu == `ALU_RSUB);
assign seq_reg_exchange = dec_f_exchange | dec_f_rot_stack;
assign seq_reg_read = (seq_state == `ST_FETCH_OP1) | (seq_state == `ST_FETCH_OP2) | (seq_state == `ST_HW_TRACE_READ) | (seq_state == `ST_HW_TRACE_OUTPUT);
// using combinatorial outputs as write enable seems to cause erratic synthesis results, sometimes works sometimes doesn't
// registered enables seem to solve the problem
reg seq_write_path_b = 1'b0; // registered write enable for the register bank path a
reg seq_write_path_a = 1'b0; // registered write enable for the register bank path b

// Keyboard connections

wire [7:0] kbd_keycode; // current pressed key
wire kbd_pending;
wire [4:0] kbd_columns;
wire kbd_pgm_mode;
wire kbd_on_off;
//
wire disp_acq;

assign debug_o[ 3 :0] = reg_path_a; //data_from_regbank_path_a;//regbank_addr_a[10:8];
assign debug_o[ 7: 4] = reg_path_b; //data_from_regbank_path_b;//regbank_addr_a[7:4];
assign debug_o[11: 8] = alu_out;//regbank_addr_a[3:0];
assign debug_o[14:12] = 3'b0;
assign debug_o[15:15] = alu_icarry;
assign debug_o[16:16] = alu_set_carry;
assign debug_o[17:17] = seq_alu_wback;

assign debug_o[18:18] = RPC == 12'h6B4;
assign debug_o[19:19] = cpu_clk;
assign debug_o[23:20] = SPC;

// Display output

wire [3:0] disp_curr_nibble;

ws_display_dogm132 ws_dogm132(
    .clk_in(cpu_clk),
    .reset_in(cpu_reset_n),
    .op_disp_off_in(dec_f_disp_off & seq_exe),
    .op_disp_toggle_in(dec_f_disp_toggle & seq_exe),
    .ra_in(data_from_regbank_path_a),
    .rb_in(data_from_regbank_path_b),
    .seq_fetch_op0(seq_fetch_op1_delayed),      // asserted when data for the display is to be latched/used
    .disp_curr_nibble_o(disp_curr_nibble), // controls which nibble should be output now
    .disp_cs_n_o(disp_cs_n_o),
    .disp_res_n_o(disp_res_n_o),
    .disp_data_o(disp_data_o),
    .disp_addr_o(disp_addr_o),
    .disp_sck_o(disp_sck_o),
	.disp_acq_o(disp_acq)
);

// divider for the 2.08 MHz clock
//GSR GSR_INST (.GSR (gbl_reset));
//   Internal Oscillator 
//   defparam OSCH_inst.NOM_FREQ = "2.08";
//   This is the default frequency     

generate
    if (MXO2)
        begin
            defparam OSCH_inst.NOM_FREQ = "2.08"; 
            OSCH OSCH_inst( .STDBY(kbd_on_off), //  0=Enabled, 1=Disabled //  also Disabled with Bandgap=OFF                
                .OSC(osc_clk),                
                .SEDSTDBY()); //  this signal is not required if not //  using SED 

reg [3:0] clk_div = 4'h0;

            always @(posedge osc_clk)
                begin
                    clk_div <= clk_div + 4'd1;
                end
            
            assign cpu_clk = clk_div[1];
            assign tx_clk  = clk_div[1];
        end
    else
        begin

reg [6:0] clk_div = 7'h0;

            always @(posedge clk_in) // 50 MHz clock
                begin
                    clk_div <= clk_div + 7'd1;
                end//

            assign cpu_clk = clk_in;//clk_div[6];
            assign tx_clk  = clk_in;//clk_div[6];
                
        end
endgenerate

    always @(posedge cpu_clk)
        begin
            if (reset_cnt != 3'd5)
                reset_cnt <= reset_cnt + 3'd1;
            else
                cpu_reset_n <= 1'b1;
        end

wire [12:0] fetch_addr;
wire [9:0] rom_opcode;
assign fetch_addr = { bank, RPC }; // use actual PC value, it is incremented automatically on stages FETCH_OP0 and FETCH_OP2

sync_rom rom(
    .clk_in(cpu_clk),
    
    .addr_in(fetch_addr),
    
    .data_o(rom_opcode)

    );  
// Trace unit 
wire [7:0] trace_tx_data;
reg trace_tx_start;
wire trace_tx_busy;
reg trace_tx_space;
reg trace_tx_cr;
wire [3:0] trace_tx_hex_d;
reg [4:0] trace_state;
wire trace_kbd_enable;  // TRACE mode enabled by keyboard command

generate if (HW_TRACE)
    begin

assign trace_tx_hex_d = (seq_op1 == `OP1_PC) ? ((seq_op1_nibble == 4'h3) ? { 3'h0, bank }:
                                                (seq_op1_nibble == 4'h2) ? seq_fetched_addr[11:8]:
                                                (seq_op1_nibble == 4'h1) ? seq_fetched_addr[ 7:4]:seq_fetched_addr[3:0]):
                        (seq_op1 == `OP1_CNT) ?((seq_op1_nibble == 4'h2) ? { 2'h0, seq_opcode[9:8] }:
                                                (seq_op1_nibble == 4'h1) ? seq_opcode[ 7:4]:seq_opcode[3:0]): 
                        (seq_op1 == `OP1_0)   ? RP:
                        reg_path_a;

assign trace_tx_data = trace_tx_space ? 8'h20:
                       trace_tx_cr ? 8'h0d:
                       trace_tx_hex_d > 4'h9 ? { 4'h4, 4'h7 + trace_tx_hex_d }:{ 4'h3, trace_tx_hex_d };

        async_transmitter txer(
            .clk(tx_clk),
            .TxD_start(trace_tx_start),
            .TxD_data(trace_tx_data),
            .TxD(txd_o),
            .TxD_busy(trace_tx_busy)
        );
    end
endgenerate


//wire kbd_read_ack;
//
// Landscape-style keyboard like the 11C
//
ws_keys_landscape keys(
    .clk_in(cpu_clk), // use processor clock for sync purposes
    .reset_in(cpu_reset_n),
    .key_read_ack_in(dec_f_kbd_ack & (seq_exe | seq_alu_wback)),
    .keys_rows_in({ rowsl_in[3:0], rowsr_in[3:0] }),
    .key_cols_o(kbd_columns),
    .keycode_o(kbd_keycode),
    .key_pending_o(kbd_pending),
    .pgm_mode_o(kbd_pgm_mode),
    .on_off_o(kbd_on_off),
    
    .operand_in(dec_lit_operand),
    .hw_flag_clr_in(dec_f_clr_flag & seq_exe),
    
    .switch_0_o(trace_kbd_enable),
    .switch_1_o()
    
`ifdef SIMULATOR
    ,
    .simkbd_activate_key_pending_in(simkbd_activate_key_pending_in),
    .simkbd_keycode_in(simkbd_keycode_in)
`endif
    );

assign col0l_o = kbd_columns[0] ? 1'b1:1'bz; // they have pull-downs
assign col0r_o = kbd_columns[0] ? 1'b1:1'bz; // they have pull-downs
assign col1l_o = kbd_columns[1] ? 1'b1:1'bz; // they have pull-downs
assign col1r_o = kbd_columns[1] ? 1'b1:1'bz; // they have pull-downs
assign col2lt_o= kbd_columns[2] ? 1'b1:1'bz; // they have pull-downs, top key has another pin due to routing issues
assign col2l_o = kbd_columns[2] ? 1'b1:1'bz; // they have pull-downs
assign col2r_o = kbd_columns[2] ? 1'b1:1'bz; // they have pull-downs
assign col3l_o = kbd_columns[3] ? 1'b1:1'bz; // they have pull-downs
assign col3r_o = kbd_columns[3] ? 1'b1:1'bz; // they have pull-downs
assign col4l_o = kbd_columns[4] ? 1'b1:1'bz; // they have pull-downs
assign col4r_o = kbd_columns[4] ? 1'b1:1'bz; // they have pull-downs


/*************************************************************************/
/*************************************************************************/
// Decoder


always @(*)
    begin
        dec_op_alu =            `ALU_NONE;
        dec_f_set_carry =       1'b0;
        dec_reg_op1 =           `OP1_A;
        dec_reg_dst =           `DST_A;
        dec_lit_operand =       seq_opcode[9:6];
        dec_field_start =       4'h0;
        dec_field_end =         4'hd;
        dec_f_bank =            1'b0;
        dec_f_crc_0 =           1'b0;
        dec_f_crc_6 =           1'b0;
        dec_f_clr_status =      1'b0;
        dec_f_clr_flag =        1'b0;
        dec_f_delayed =         1'b0;
        dec_f_disp_toggle =     1'b0;
        dec_f_disp_off =        1'b0;
        dec_f_gto_c_clr =       1'b0; /* goto on carry clear, for tests */
        dec_f_gto_c_set =       1'b0; /* goto on carry set, for tests */
        dec_f_jump =            1'b0; // jumps without delay rom 
        dec_f_use_delay_rom   = 1'b0; // selects delay rom during write back of new PC
        dec_f_gonc            = 1'b0; // checks for carry cleared during ST_JUMP
        dec_f_nop =             1'b0;
        dec_f_pop_pc =          1'b0; /* rtn */
        dec_f_push_pc =         1'b0;
        dec_p_func =            `P_NONE;
        dec_f_set_hex =         1'b0;
        dec_f_set_dec =         1'b0;
        dec_f_set_flag =        1'b0;
        dec_f_test_clr_flag =   1'b0;
        dec_f_test_set_flag =   1'b0;
        dec_f_kbd_ack =         1'b0;
        dec_f_set_carry_early = 1'b0; // use to set the carry for opcodes that need a 1 as dec_lit_operand
        dec_jaddr =             { RPC[11:8], seq_opcode[9:2] };  // jump address 
        dec_f_rol             = 1'b0;
        dec_f_lsl             = 1'b0;
        dec_f_lsr             = 1'b0;
        dec_f_tfr             = 1'b0;
        dec_f_exchange        = 1'b0;
        dec_f_wr_alu          = 1'b0;
        dec_f_wr_lit          = 1'b0; // load constant into C
        dec_f_push_c          = 1'b0;
        dec_f_pop_a           = 1'b0;
        dec_f_clr_regs        = 1'b0;
        dec_f_clr_dregs       = 1'b0;
        dec_f_rot_stack       = 1'b0;
        dec_f_crc_060         = 1'b0;
        dec_f_crc_100         = 1'b0;
        dec_f_crc_160         = 1'b0;
        dec_f_crc_300         = 1'b0;
        dec_f_crc_360         = 1'b0;
        dec_f_crc_400         = 1'b0;
        dec_f_crc_500         = 1'b0;
        dec_f_crc_560         = 1'b0;
        dec_f_crc_660         = 1'b0;
        dec_f_crc_760         = 1'b0;
        dec_f_crc_1000        = 1'b0;
        dec_f_crc_1100        = 1'b0;
        dec_f_crc_1200        = 1'b0;
        dec_f_crc_1300        = 1'b0;
        dec_f_crc_1400        = 1'b0;
        dec_f_crc_1500        = 1'b0;
        dec_f_crc_1700        = 1'b0;
        case (seq_opcode[1:0])
            2'b00: // general opcodes
                begin
                    case (seq_opcode[9:2])
                        8'b0000_0000: dec_f_nop = 1'b1;
                        8'b0001_0000: dec_f_crc_100 = 1'b1;
                        8'b0010_0000: dec_f_nop = 1'b1;
                        8'b0011_0000: dec_f_crc_300 = 1'b1;
                        8'b0100_0000: dec_f_crc_400 = 1'b1;
                        8'b0101_0000: dec_f_crc_500 = 1'b1;
                        8'b0110_0000: dec_f_nop = 1'b1;
                        8'b0111_0000: dec_f_nop = 1'b1;
                        8'b1000_0000: dec_f_crc_1000 = 1'b1;
                        8'b1001_0000: dec_f_crc_1100 = 1'b1;
                        8'b1010_0000: dec_f_crc_1200 = 1'b1;
                        8'b1011_0000: dec_f_crc_1300 = 1'b1;
                        8'b1100_0000: dec_f_crc_1400 = 1'b1;
                        8'b1101_0000: dec_f_crc_1500 = 1'b1;
                        8'b1110_0000: dec_f_nop = 1'b1;
                        8'b1111_0000: dec_f_crc_1700 = 1'b1;
                        
                        8'b0000_0001, 8'b0001_0001, 8'b0010_0001, 8'b0011_0001,
                        8'b0100_0001, 8'b0101_0001, 8'b0110_0001, 8'b0111_0001,
                        8'b1000_0001, 8'b1001_0001, 8'b1010_0001, 8'b1011_0001,
                        8'b1100_0001, 8'b1101_0001, 8'b1110_0001, 8'b1111_0001: dec_f_set_flag = 1'b1;
                        
                        8'b0000_0010: dec_f_clr_regs = 1'b1;
                        8'b0001_0010: dec_f_clr_status = 1'b1;
                        8'b0010_0010: dec_f_disp_toggle = 1'b1;
                        8'b0011_0010: dec_f_disp_off = 1'b1;
                        8'b0100_0010: begin dec_f_exchange = 1'b1;  dec_reg_dst = `DST_C; dec_reg_op1 = `OP1_M1; dec_field_end = 4'hd; end// c <-> m1
                        8'b0101_0010: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_C; dec_reg_op1 = `OP1_M1; dec_field_end = 4'hd; end // m1 -> C
                        8'b0110_0010: begin dec_f_exchange = 1'b1;  dec_reg_dst = `DST_C; dec_reg_op1 = `OP1_M2; dec_field_end = 4'hd; end// c <-> m2
                        8'b0111_0010: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_C; dec_reg_op1 = `OP1_M2; dec_field_end = 4'hd; end// m2 -> C
                        8'b1000_0010: dec_f_pop_a = 1'b1; // a = y, y = z, z = t
                        8'b1001_0010: dec_f_rot_stack = 1'b1;
                        8'b1010_0010: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_A; dec_reg_op1 = `OP1_Y; dec_field_end = 4'hd; end
                        8'b1011_0010: dec_f_push_c = 1'b1; // t = z, z = y, y = c
                        8'b1100_0010: dec_f_set_dec = 1'b1;
                        8'b1110_0010: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_A; dec_reg_op1 = `OP1_F; dec_field_start = 4'h0; dec_field_end = 4'h0; end
                        8'b1111_0010: begin dec_f_exchange = 1'b1; dec_reg_dst = `DST_F; dec_reg_op1 = `OP1_A; dec_field_start = 4'h0; dec_field_end = 4'h0; end
                        
                        8'b0000_0011, 8'b0001_0011, 8'b0010_0011, 8'b0011_0011,
                        8'b0100_0011, 8'b0101_0011, 8'b0110_0011, 8'b0111_0011,
                        8'b1000_0011, 8'b1001_0011, 8'b1010_0011, 8'b1011_0011,
                        8'b1100_0011, 8'b1101_0011, 8'b1110_0011, 8'b1111_0011: dec_f_clr_flag = 1'b1;
                        
                        8'b0000_0100: begin dec_jaddr = { RPC[11:8], kbd_keycode[7:0] }; dec_f_jump = 1'b1; dec_f_kbd_ack = 1'b1; end// keys-> rom 
                        8'b0001_0100: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_A; dec_reg_op1 = `OP1_KEY; dec_field_end = 4'h2; dec_field_start = 4'h1; dec_f_kbd_ack = 1'b1; end // Keys -> A
                        8'b0010_0100: begin dec_jaddr = { RPC[11:8], RA[11:4] }; dec_f_jump = 1'b1; dec_f_use_delay_rom = 1'b1; end// a-> rom 
                        8'b0011_0100: dec_f_nop = 1'b1; // HP-67 display reset 
                        8'b0100_0100: dec_f_set_hex = 1'b1;
                        8'b0101_0100: begin dec_f_rol = 1'b1; dec_reg_dst = `DST_A; dec_reg_op1 = `OP1_A; dec_field_start = 4'h0; dec_field_end = 4'hd; end
                        8'b0110_0100: dec_p_func = `P_DEC_P;
                        8'b0111_0100: dec_p_func = `P_INC_P;
                        8'b1000_0100: begin dec_f_pop_pc = 1'b1; end
                        
                        8'b0000_0101, 8'b0001_0101, 8'b0010_0101, 8'b0011_0101,
                        8'b0100_0101, 8'b0101_0101, 8'b0110_0101, 8'b0111_0101,
                        8'b1000_0101, 8'b1001_0101, 8'b1010_0101, 8'b1011_0101,
                        8'b1100_0101, 8'b1101_0101, 8'b1110_0101, 8'b1111_0101: begin dec_f_test_set_flag = 1'b1; dec_f_gto_c_clr = 1'b1; end // if 1 = s
                        
                        8'b0000_0110, 8'b0001_0110, 8'b0010_0110, 8'b0011_0110,
                        8'b0100_0110, 8'b0101_0110, 8'b0110_0110, 8'b0111_0110,
                        8'b1000_0110, 8'b1001_0110, 8'b1010_0110, 8'b1011_0110,
                        8'b1100_0110, 8'b1101_0110, 8'b1110_0110, 8'b1111_0110: 
                            begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_C; dec_reg_op1 = `OP1_CNT; dec_field_start = RP; dec_field_end = RP; dec_p_func = `P_DEC_P; end // Load Constant @P
                        
                        8'b0000_0111, 8'b0001_0111, 8'b0010_0111, 8'b0011_0111,
                        8'b0100_0111, 8'b0101_0111, 8'b0110_0111, 8'b0111_0111,
                        8'b1000_0111, 8'b1001_0111, 8'b1010_0111, 8'b1011_0111,
                        8'b1100_0111, 8'b1101_0111, 8'b1110_0111,
                        8'b1111_0111: begin dec_f_test_clr_flag = 1'b1; dec_f_gto_c_clr = 1'b1; end // if 0 = s
                        
                        8'b0000_1000, 8'b0001_1000, 8'b0010_1000, 8'b0011_1000,
                        8'b0100_1000, 8'b0101_1000, 8'b0110_1000, 8'b0111_1000,
                        8'b1000_1000, 8'b1001_1000, 8'b1010_1000, 8'b1011_1000,
                        8'b1100_1000, 8'b1101_1000, 8'b1110_1000,
                        8'b1111_1000: begin dec_jaddr = { seq_opcode[9:6], RPC[7:0] }; dec_f_jump = 1'b1; end // select rom
                        
                        8'b0000_1001, 8'b0001_1001, 8'b0010_1001, 8'b0011_1001,
                        8'b0100_1001, 8'b0101_1001, 8'b0110_1001, 8'b0111_1001,
                        8'b1000_1001, 8'b1001_1001, 8'b1010_1001, 8'b1011_1001,
                        8'b1100_1001, 8'b1101_1001, 8'b1110_1001,
                        8'b1111_1001: begin dec_p_func = `P_CMP_EQ; dec_f_gto_c_clr = 1'b1; end// if p =/# 0 
                        
                        8'b0000_1011, 8'b0001_1011, 8'b0010_1011, 8'b0011_1011,
                        8'b0100_1011, 8'b0101_1011, 8'b0110_1011, 8'b0111_1011,
                        8'b1000_1011, 8'b1001_1011, 8'b1010_1011, 8'b1011_1011,
                        8'b1100_1011, 8'b1101_1011, 8'b1110_1011,
                        8'b1111_1011: begin dec_p_func = `P_CMP_NEQ; dec_f_gto_c_clr = 1'b1; end  // if p =/# 0 
                        
                        8'b0000_1010, 8'b0001_1010, 8'b0010_1010, 8'b0011_1010,
                        8'b0100_1010, 8'b0101_1010, 8'b0110_1010, 8'b0111_1010,
                        8'b1000_1010, 8'b1001_1010, 8'b1010_1010, 8'b1011_1010, 
                        8'b1100_1010, 8'b1101_1010, 8'b1110_1010,
                        8'b1111_1010: begin dec_f_tfr = 1'b1; dec_reg_dst = { 1'b0, DATAADDR[5:4], seq_opcode[9:6]};  dec_reg_op1 = `OP1_C; end // C -> reg n
                        
                        8'b0000_1100: dec_f_crc_060 = 1'b1;
                        8'b0001_1100: dec_f_crc_160 = 1'b1;
                        8'b0010_1100: dec_f_crc_260 = 1'b1;
                        8'b0011_1100: dec_f_crc_360 = 1'b1;
                        8'b0100_1100: dec_f_nop = 1'b1;
                        8'b0101_1100: dec_f_crc_560 = 1'b1;
                        8'b0110_1100: dec_f_crc_660 = 1'b1;
                        8'b0111_1100: dec_f_crc_760 = 1'b1;
                        
                        8'b1000_1100: dec_f_bank = 1'b1;
                        8'b1001_1100: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_DA; dec_reg_op1 = `OP1_C; dec_field_end = 4'h1; end// C -> DA
                        8'b1010_1100: dec_f_clr_dregs = 1'b1;
                        8'b1011_1100: begin dec_f_tfr = 1'b1; dec_reg_dst = { 1'b0, DATAADDR }; dec_reg_op1 = `OP1_C; end // C -> reg[DA]
                        8'b1111_1100: dec_f_nop = 1'b1; // Woodstock ?
                        
                        8'b0000_1101, 8'b0001_1101, 8'b0010_1101, 8'b0011_1101,
                        8'b0100_1101, 8'b0101_1101, 8'b0110_1101, 8'b0111_1101,
                        8'b1000_1101, 8'b1001_1101, 8'b1010_1101, 8'b1011_1101,
                        8'b1100_1101, 8'b1101_1101, 8'b1110_1101,
                        8'b1111_1101: dec_f_delayed = 1'b1;
                        // next opcode uses the data address to fetch the register value and not the opcode field
                        //8'b0000_1110: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_C; dec_reg_op1 = { 1'b0, DATAADDR }; end // reg[DA] -> C
                        
                        8'b0000_1110,
                        8'b0001_1110, 8'b0010_1110, 8'b0011_1110, 8'b0100_1110,
                        8'b0101_1110, 8'b0110_1110, 8'b0111_1110, 8'b1000_1110,
                        8'b1001_1110, 8'b1010_1110, 8'b1011_1110, 8'b1100_1110,
                        8'b1101_1110, 8'b1110_1110, 8'b1111_1110: begin dec_f_tfr = 1'b1; dec_reg_dst = `DST_C; dec_reg_op1 = { 1'b0, DATAADDR[5:4], (seq_opcode[9:6] == 4'h0) ? DATAADDR[3:0]:seq_opcode[9:6]}; end // reg n -> C
                        // Load Constant to P, carry cleared from reg_p module
                        8'b0000_1111, 8'b0001_1111, 8'b0010_1111, 8'b0011_1111,
                        8'b0100_1111, 8'b0101_1111, 8'b0110_1111, 8'b0111_1111,
                        8'b1000_1111, 8'b1001_1111, 8'b1010_1111, 8'b1011_1111,
                        8'b1100_1111, 8'b1101_1111, 8'b1110_1111, 8'b1111_1111: dec_p_func = `P_LOAD; 
                        default: 
                            begin
                                dec_f_nop = 1'b1;
                                $display("%04o %10b unrecognized opcode", seq_fetched_addr, seq_opcode);
                            end
                    endcase
                end
            2'b01: // jsb
                begin
                    dec_f_push_pc = 1'b1;
                    dec_f_use_delay_rom = 1'b1;
                    dec_f_jump = 1'b1;
                end
            2'b10: // arithmetic opcodes
                begin
                    dec_field_start = dec_op_field_start; // load translated field pointers
                    dec_field_end = dec_op_field_end; // load translated field pointers
                    case (seq_opcode[9:5])
                        5'b00000: begin dec_f_tfr = 1'b1; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_A; end // 0 -> a[w ]
                        5'b00001: begin dec_f_tfr = 1'b1; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_B; end // 0 -> b[w ]
                        5'b00010: begin dec_f_exchange = 1'b1;  dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_B; end // a exchange b[wp]
                        5'b00011: begin dec_f_tfr = 1'b1; dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_B; end //  a -> b[x ]
                        5'b00100: begin dec_f_exchange = 1'b1;  dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_C; end // a exchange c[w ]
                        5'b00101: begin dec_f_tfr = 1'b1; dec_reg_op1 = `OP1_C; dec_reg_dst = `DST_A; end // c -> a[w ]
                        5'b00110: begin dec_f_tfr = 1'b1; dec_reg_op1 = `OP1_B; dec_reg_dst = `DST_C; end // b -> c[wp]
                        5'b00111: begin dec_f_exchange = 1'b1;  dec_reg_op1 = `OP1_B; dec_reg_dst = `DST_C; end // b exchange c[w ]
                        5'b01000: begin dec_f_tfr = 1'b1; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_C; end // 0 -> c[w ]
                        5'b01001: begin dec_op_alu = `ALU_ADD; dec_reg_op1 = `OP1_B; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; end // a + b -> a[ms]
                        5'b01010: begin dec_op_alu = `ALU_ADD; dec_reg_op1 = `OP1_C; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; end // a + c -> a[m ]
                        5'b01011: begin dec_op_alu = `ALU_ADD; dec_reg_op1 = `OP1_C; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; end // c + c -> c[w ]
                        5'b01100: begin dec_op_alu = `ALU_ADD; dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; end // a + c -> c[x ]
                        5'b01101: begin dec_op_alu = `ALU_ADD; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; dec_f_set_carry_early = 1'b1; end // a + 1 -> a[p ] carry used as constant 1
                        5'b01110: begin dec_f_lsl = 1'b1; dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_A; end // shift left a[w ] 
                        5'b01111: begin dec_op_alu = `ALU_ADD; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; dec_f_set_carry_early = 1'b1; end // c + 1 -> c[xs] carry used as constant 1
                        5'b10000: begin dec_op_alu = `ALU_SUB; dec_reg_op1 = `OP1_B; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; end // a - b -> a[ms]
                        5'b10001: begin dec_op_alu = `ALU_RSUB;dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; end // a - c -> c[s ]
                        5'b10010: begin dec_op_alu = `ALU_SUB; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; dec_f_set_carry_early = 1'b1; end // a - 1 -> a[s ] carry used as constant 1
                        5'b10011: begin dec_op_alu = `ALU_SUB; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; dec_f_set_carry_early = 1'b1; end // c - 1 -> c[x ] carry used as constant 1
                        5'b10100: begin dec_op_alu = `ALU_RSUB;dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; end // 0 - c -> c[s ]
                        5'b10101: begin dec_op_alu = `ALU_RSUB;dec_reg_op1 = `OP1_9; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; end // 0 - c - 1 -> c[s ]
                        5'b10110: begin dec_op_alu = `ALU_EQ;  dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_B; dec_f_set_carry = 1'b1; dec_f_gto_c_set = 1'b1; end // if 0 = b
                        5'b10111: begin dec_op_alu = `ALU_EQ;  dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; dec_f_gto_c_set = 1'b1; end // if 0 = c
                        5'b11000: begin dec_op_alu = `ALU_GTEQ;dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; dec_f_gto_c_set = 1'b1; end // if a >= c
                        5'b11001: begin dec_op_alu = `ALU_GTEQ;dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_B; dec_f_set_carry = 1'b1; dec_f_gto_c_set = 1'b1; end // if a >= b
                        5'b11010: begin dec_op_alu = `ALU_NEQ; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; dec_f_gto_c_set = 1'b1; end // if 0 # a
                        5'b11011: begin dec_op_alu = `ALU_NEQ; dec_reg_op1 = `OP1_0; dec_reg_dst = `DST_C; dec_f_set_carry = 1'b1; dec_f_gto_c_set = 1'b1; end // if 0 # c
                        5'b11100: begin dec_op_alu = `ALU_SUB; dec_reg_op1 = `OP1_C; dec_reg_dst = `DST_A; dec_f_set_carry = 1'b1; end // a - c -> a[wp]
                        5'b11101: begin dec_f_lsr = 1'b1; dec_reg_op1 = `OP1_A; dec_reg_dst = `DST_A; end // shift right a[wp]
                        5'b11110: begin dec_f_lsr = 1'b1; dec_reg_op1 = `OP1_B; dec_reg_dst = `DST_B; end // shift right b[wp]
                        5'b11111: begin dec_f_lsr = 1'b1; dec_reg_op1 = `OP1_C; dec_reg_dst = `DST_C; end // shift right c[w ]
                    endcase
                end
            2'b11: // gonc
                begin
                    dec_f_gonc = 1'b1;
                    dec_f_use_delay_rom = 1'b1;
                end
        endcase
    end

/*************************************************************************/
/*************************************************************************/
// Dis assembler
wire [199:0] dis_opcode;
wire [55:0] dis_label;
/*
ws_hp67_disasm disasm(
    .addr({ bank, seq_fetched_addr }),
    .o(dis_opcode),
    .l(dis_label)
    );
*/
assign dis_label = "    ";
nws_woodstock_disasm disasm(
    .addr_in(seq_fetched_addr),
    .bank_in(bank),
    .opcode_in(seq_opcode),
    .op2_in(seq_opcode2),
    .o_o(dis_opcode)
    );
    
/************************************************************************/
/************************************************************************/
// FIeld translators

ws_field_decoder field_decoder(
    .field_in(seq_opcode[4:2]), // .....fff..
    .p_in(RP), // register P
    .start_o(dec_op_field_start), 
    .end_o(dec_op_field_end)
    );

ws_loadto_p loadto(.value_in(seq_opcode[9:6]), .p_o(dec_lit_ldi_p));
ws_cmpwith_p cmpwith(.value_in(seq_opcode[9:6]), .p_o(dec_lit_cmp_p));

/************************************************************************/
/************************************************************************/
// PC & Stack

always @(posedge cpu_clk)
    begin
        if (seq_fetch_op0 | seq_fetch_op2)
            RPC <= RPC + 12'd1;
        else
            if (dec_f_pop_pc & seq_exe) // returns
                begin
                    RPC <= STACK[1'h0];
                    STACK[1'h0] <= STACK[1'h1];
                    STACK[1'h1] <= 12'h0;
                end
            else
                if (seq_jump)
                    begin
                        // flag tests, register compare
                        if (dec_f_gto_c_set) // double word instructions use current carry
                            begin
                                if (f_carry) RPC <= seq_ifaddr; 
                            end
                        else
                            if (dec_f_gto_c_clr) // double word instructions use current carry
                                begin
                                    if (~f_carry)
                                        RPC <= seq_ifaddr;
                                end
                            else
                                begin
                                    RPC <= (dec_f_use_delay_rom & delayed_rom_active) ? { delayed_rom, dec_jaddr[7:0] }: dec_jaddr;
                                end
                    end
        if (dec_f_push_pc & seq_exe)
            begin
                STACK[1'h1] <= STACK[1'h0];
                STACK[1'h0] <= RPC;
            end
    end

/************************************************************************/
/************************************************************************/
// Register P

always @(posedge cpu_clk)
    begin
        if (seq_exe | seq_alu_wback)
            begin
                case (dec_p_func)
                    `P_NONE: begin end
                    `P_INC_P: 
                        begin
                            if (RP == 4'hd) begin RP <= 4'h0; p_crossed_inc <= 1'b1; end
                            else
                                begin 
                                    RP <= RP + 4'h1;
                                    if ((p_crossed_inc) && (RP != 4'h0))
                                        p_crossed_inc <= 1'b0;
                                end
                            p_crossed_dec <= 1'b0;
                        end
                    `P_DEC_P:
                        begin
                            if (RP == 4'h0) begin RP <= 4'hd; p_crossed_dec <= 1'b1; end
                            else 
                                begin 
                                    RP <= RP - 4'h1; 
                                    if ((p_crossed_dec) && (RP != 4'hd)) // extend it for one cycle if is a chain of decrements
                                        p_crossed_dec <= 1'b0;
                                end
                            p_crossed_inc <= 1'b0;
                        end
                    `P_LOAD: 
                        begin
                            RP <= dec_lit_ldi_p;
                            p_crossed_inc <= 1'b0;
                            p_crossed_dec <= 1'b0;
                        end
                endcase
            end
    end

always @(*)
    begin
        reg_p_set_carry = 1'b0;
        case (dec_p_func)
            `P_CMP_EQ:
                begin
                    if (dec_lit_cmp_p == 4'h0)
                        begin
                            if (p_crossed_inc || p_crossed_dec)
                                reg_p_set_carry = 1'b1; // not equal
                            else
                                if (RP != 4'h0) reg_p_set_carry = 1'b1;
                        end
                    else
                        if (RP != dec_lit_cmp_p) reg_p_set_carry = 1'b1;
                end
            `P_CMP_NEQ:
                begin
                    if (dec_lit_cmp_p == 4'h0) 
                        begin
                            if (p_crossed_inc || p_crossed_dec)
                                reg_p_set_carry = 1'b1; // not equal
                            else
                                if (RP == 4'h0) reg_p_set_carry = 1'b1;
                        end
                    else
                        if (RP == dec_lit_cmp_p) reg_p_set_carry = 1'b1;
                end
            default: reg_p_set_carry = 1'b0;
        endcase
    end

/************************************************************************/
/************************************************************************/
// Data and Arithmetic Registers

/*
 * A dual ported memory is used for all registers
 * path a is operand 1 and path b is operand 2/destination
 * two clocks are needed to read and write to the memory
 * clock one reads, clock two writes
 * The first 64 registers are used as registers and the next 64 registers as CPU registers A, B, C. X, Y Z and M1, M2
 */

assign #100 regbank_addr_a = { seq_op1, seq_op1_nibble };
assign #100 regbank_addr_b = { seq_dst, seq_dst_nibble };

// Data comes to port b from transfer (also shifts), exchange or add/sub
assign data_to_reg_b = (seq_reg_exchange | seq_reg_transfer) ? reg_path_a:alu_out; // 

generate 
    if (MXO2)
        begin
            regbank_lattice u1 (
                .DataInA(reg_path_b), 
                .AddressA(regbank_addr_a), 
                .ClockA(cpu_clk), 
                .ClockEnA(1'b1),//seq_reg_read | (seq_reg_exchange & seq_alu_wback)), 
                .WrA(seq_write_path_a), //seq_reg_exchange & seq_alu_wback),//
                .ResetA(1'b0), 
                .QA(data_from_regbank_path_a), 
                
                .DataInB(data_to_reg_b), 
                .AddressB(regbank_addr_b), 
                .ClockB(cpu_clk), 
                .ClockEnB(1'b1),//((seq_reg_transfer | seq_reg_write_alu_or_lit | seq_reg_exchange) & seq_alu_wback) | seq_reg_read), 
                .WrB(seq_write_path_b), //(seq_reg_transfer | seq_reg_write_alu_or_lit | seq_reg_exchange) & seq_alu_wback),//
                .ResetB(1'b0), 
                .QB(data_from_regbank_path_b)
                );
        end
    else
        if (MAX10)
            begin
                regbank_max10 u1(
                    .address_a(regbank_addr_a), 
                    .address_b(regbank_addr_b), 
                    .clock(cpu_clk), 
                    .data_a(reg_path_b), 
                    .data_b(data_to_reg_b), 
                    .wren_a(seq_reg_exchange & seq_alu_wback), 
                    .wren_b((seq_reg_transfer | seq_reg_write_alu_or_lit | seq_reg_exchange) & seq_alu_wback), 
                    .q_a(data_from_regbank_path_a), 
                    .q_b(data_from_regbank_path_b));
            end//
        else
            begin
                dp_reg_bank regs(
                    .clk_in(cpu_clk),
                    .seq_decode_in(seq_decode),
                    .wea_in(seq_write_path_a), //seq_reg_exchange & seq_alu_wback),
                    .dataa_in(reg_path_b),
                    .addra_in(regbank_addr_a),
                    .dataa_o(data_from_regbank_path_a),
                    
                    .web_in(seq_write_path_b), //(seq_reg_transfer | seq_reg_write_alu_or_lit | seq_reg_exchange) & seq_alu_wback),
                    .datab_in(data_to_reg_b),
                    .addrb_in(regbank_addr_b),
                    .datab_o(data_from_regbank_path_b)
                    );            
            end
endgenerate

assign #50 reg_path_a = (seq_op1 == `OP1_0) ? 4'h0:
                    (seq_op1 == `OP1_9) ? (f_decimal ? 4'h9:4'hf):
                    //(seq_op1 == `OP1_F) ? RF:
                    (seq_op1 == `OP1_CNT) ? dec_lit_operand:
                    (seq_op1 == `OP1_KEY) ? (seq_op1_nibble == 4'h2 ? kbd_keycode[7:4]:kbd_keycode[3:0]):
                    data_from_regbank_path_a;
                   

assign #50 reg_path_b = (seq_dst == `DST_0) ? 4'h0:
                        (seq_dst == `DST_9) ? (f_decimal ? 4'h9:4'hf):
                        //(seq_dst == `OP_F) ? RF:
                        data_from_regbank_path_b;
    
/* Updates some registers that are not in the register bank register */
always @(posedge cpu_clk)
    begin
        if (seq_inc_da)
            DATAADDR[3:0] <= DATAADDR[3:0] + 4'h1; // used to clear registers
        if ((seq_reg_transfer | seq_reg_write_alu_or_lit | seq_reg_exchange) & seq_alu_wback) // any write
            begin
                case (seq_dst)
                    `DST_A:
                        case (seq_dst_nibble)
                            4'h1: RA[7:4] <= data_to_reg_b;
                            4'h2: RA[11:8] <= data_to_reg_b;
                        endcase
                    `DST_DA:
                        if (seq_dst_nibble[0])
                            DATAADDR[5:4] <= data_to_reg_b[1:0];
                        else
                            DATAADDR[3:0] <= data_to_reg_b;
                    //`OP_F: RF <= data_to_reg_b;
                endcase
            end
        if (seq_reg_exchange & seq_alu_wback)
            begin
                case (seq_op1)
                    `OP1_A:
                        case (seq_dst_nibble)
                            4'h1: RA[7:4] <= reg_path_b;
                            4'h2: RA[11:8] <= reg_path_b;
                        endcase
                    //`OP1_F: RF <= reg_path_b;
                endcase
            end
    end    

/*********************************************************************/
/*********************************************************************/
// ALU

subbcd sub(
    .a_in(reg_path_b),
    .b_in(reg_path_a),
    .c_in(alu_icarry),
    .dec_in(f_decimal),
    .q_out(alu_q_sub),
    .qc_out(alu_qc_sub)
);

subbcd rsub(
    .a_in(reg_path_a),
    .b_in(reg_path_b),
    .c_in(alu_icarry),
    .dec_in(f_decimal),
    .q_out(alu_q_rsub),
    .qc_out(alu_qc_rsub)
);
    
addbcd add(
    .a_in(reg_path_a),
    .b_in(reg_path_b),
    .c_in(alu_icarry),
    .dec_in(f_decimal),
    .q_out(alu_q_add),
    .qc_out(alu_qc_add)
);

comp_unit comp(
    .left_in(reg_path_a), 
    .right_in(reg_path_b), 
    .equal_o(alu_eq),
    .notequal_o(alu_neq),
    .gt_o(alu_gt)
    );
                   
assign #50 alu_out       = (dec_op_alu == `ALU_SUB)  ? alu_q_sub:
                       (dec_op_alu == `ALU_RSUB) ? alu_q_rsub:alu_q_add;

assign #50 alu_set_carry = (dec_op_alu == `ALU_ADD)  ? alu_qc_add:
                       (dec_op_alu == `ALU_SUB)  ? alu_qc_sub:
                       (dec_op_alu == `ALU_RSUB) ? alu_qc_rsub:
                       (dec_op_alu == `ALU_EQ)   ? (alu_comp_res == `CMP_EQ):
                       (dec_op_alu == `ALU_NEQ)  ? (alu_comp_res == `CMP_NEQ):
                       (dec_op_alu == `ALU_GTEQ) ? ((alu_comp_res == `CMP_EQ) | (alu_comp_res == `CMP_GT)):1'b0;

always @(posedge cpu_clk)
    begin
        if (seq_alu_prep)
            begin
                alu_comp_res <= `CMP_NONE;
                alu_icarry <= dec_f_set_carry_early;//f_carry;
             end
        if (seq_alu_read3)
            begin
                case (dec_op_alu)
                    `ALU_EQ: if (alu_eq & (alu_comp_res != `CMP_NEQ))
                                alu_comp_res  <= `CMP_EQ; // EQ is NOT sticky
                             else 
                                alu_comp_res <= `CMP_NEQ; // NEQ is sticky
                    `ALU_NEQ: 
                            if (alu_neq)
                                begin
                                    alu_comp_res  <= `CMP_NEQ; // NEQ is sticky
                                end
                             else
                                if (alu_comp_res != `CMP_NEQ) alu_comp_res <= `CMP_EQ;
                    `ALU_GTEQ:
                        begin
                            if (alu_eq)
                                begin
                                    if (alu_comp_res == `CMP_NONE) alu_comp_res  <= `CMP_EQ;
                                end
                            else
                                if (alu_gt)
                                    begin
                                        if ((alu_comp_res == `CMP_NONE) | (alu_comp_res == `CMP_EQ)) alu_comp_res  <= `CMP_GT;
                                    end
                                else
                                    if ((alu_comp_res == `CMP_NONE) | (alu_comp_res == `CMP_EQ))
                                        alu_comp_res <= `CMP_LT;
                        end
                endcase
            end
        if (seq_alu_wback)
            begin
                alu_icarry <= alu_set_carry;
            end
    end


// HW Status registers
assign f_hw_status = { kbd_pending,    f_hw_status_14, f_hw_status_13, f_hw_status_12,
                       f_hw_status_11, f_hw_status_10,  f_hw_status_9,  f_hw_status_8,
                        f_hw_status_7,  f_hw_status_6,    batt_status,  f_hw_status_4,
                         f_crc_status,  f_hw_status_2,  f_hw_status_1,  f_hw_status_0 };

always @(posedge cpu_clk)
    begin
        if (~cpu_reset_n)
            begin
                f_decimal <= 1'b0;
                f_hw_status_14 <= 1'b0;
                f_hw_status_13 <= 1'b0;
                f_hw_status_12 <= 1'b0;
                f_hw_status_11 <= 1'b0;
                f_hw_status_10 <= 1'b0;
                f_hw_status_9  <= 1'b0;
                f_hw_status_8  <= 1'b0;
                f_hw_status_7  <= 1'b0;
                f_hw_status_6  <= 1'b0;
                f_hw_status_4  <= 1'b0;
                f_hw_status_2  <= 1'b0;
                f_hw_status_1  <= 1'b0;
                f_hw_status_0  <= 1'b0;
                f_carry <= 1'b0;
                f_latched_carry <= 1'b0;
                f_crc_status <= 1'b0;
                f_crc_disp_mode <= 1'b1;
                f_first_row_pgm <= 1'b0;
                f_crc_key <= 1'b0;
                f_crc_merge <= 1'b0;
                f_crc_pause <= 1'b0;
            end
        else
            begin
                if (seq_decode)
                    begin
                        f_latched_carry <= f_carry; // used for jumps
                        f_carry <= 1'b0; // cleared automatically
                    end
                if (seq_exe)
                    begin
                        if (dec_f_set_hex)
                            f_decimal <= 1'b0;
                        if (dec_f_set_dec)
                            f_decimal <= 1'b1;
                        if (dec_f_test_set_flag)
                            f_carry <= ~f_hw_status[dec_lit_operand];
                        if (dec_f_test_clr_flag)
                            f_carry <= f_hw_status[dec_lit_operand];
                        //if (reg_p_set_carry) 
                        //    f_carry <= 1'b1;
                        if (dec_f_clr_status)
                            begin
                                f_hw_status_14 <= 1'b0;
                                f_hw_status_13 <= 1'b0;
                                f_hw_status_12 <= 1'b0;
                                f_hw_status_11 <= 1'b0;
                                f_hw_status_10 <= 1'b0;
                                f_hw_status_9  <= 1'b0;
                                f_hw_status_8  <= 1'b0;
                                f_hw_status_7  <= 1'b0;
                                f_hw_status_6  <= 1'b0;
                                f_hw_status_4  <= 1'b0;
                                f_hw_status_2  <= 1'b0;
                                f_hw_status_1  <= 1'b0;
                                f_hw_status_0  <= 1'b0;
                            end
                        if (dec_f_clr_flag)
                            case (dec_lit_operand)
                                4'hE: f_hw_status_14 <= 1'b0;
                                4'hD: f_hw_status_13 <= 1'b0;
                                4'hC: f_hw_status_12 <= 1'b0;
                                4'hB: f_hw_status_11 <= 1'b0;
                                4'hA: f_hw_status_10 <= 1'b0;
                                4'h9: f_hw_status_9  <= 1'b0;
                                4'h8: f_hw_status_8  <= 1'b0;
                                4'h7: f_hw_status_7  <= 1'b0;
                                4'h6: f_hw_status_6  <= 1'b0;
                                4'h4: f_hw_status_4  <= 1'b0;
                                4'h3: f_crc_status   <= 1'b0;
                                4'h2: f_hw_status_2  <= 1'b0;
                                4'h1: f_hw_status_1  <= 1'b0;
                                4'h0: f_hw_status_0  <= 1'b0; 
                            endcase
                        if (dec_f_set_flag)
                            case (dec_lit_operand)
                                4'hE: f_hw_status_14 <= 1'b1;
                                4'hD: f_hw_status_13 <= 1'b1;
                                4'hC: f_hw_status_12 <= 1'b1;
                                4'hB: f_hw_status_11 <= 1'b1;
                                4'hA: f_hw_status_10 <= 1'b1;
                                4'h9: f_hw_status_9  <= 1'b1;
                                4'h8: f_hw_status_8  <= 1'b1;
                                4'h7: f_hw_status_7  <= 1'b1;
                                4'h6: f_hw_status_6  <= 1'b1;
                                4'h4: f_hw_status_4  <= 1'b1;
                                4'h2: f_hw_status_2  <= 1'b1;
                                4'h1: f_hw_status_1  <= 1'b1;
                                4'h0: f_hw_status_0  <= 1'b1; 
                            endcase
                        if (dec_f_crc_060) // set display mode
                            f_crc_disp_mode <= 1'b1;
                        if (dec_f_crc_100) // device ready
                            f_crc_status <= 1'b1;
                        if (dec_f_crc_160) // test display mode
                            begin f_crc_status <= f_crc_disp_mode; f_crc_disp_mode <= 1'b0; end
                        if (dec_f_crc_260) // start motor
                            begin end
                        if (dec_f_crc_300) // getsstatus of the PGM input
                            f_crc_status <= kbd_pgm_mode;
                        if (dec_f_crc_360) // stop motor
                            begin end
                        if (dec_f_crc_400) // set key
                            f_crc_key <= 1'b1;
                        if (dec_f_crc_500) // test key
                            begin f_crc_status <= f_crc_key; f_crc_key <= 1'b0; end
                        if (dec_f_crc_560) // test card inserted
                            f_crc_status <= 1'b0;
                        if (dec_f_crc_660) // set write mode
                            begin end
                        if (dec_f_crc_760) // set read mode
                            begin end
                        if (dec_f_crc_1000) // set normal keys
                            f_first_row_pgm <= 1'b1;
                        if (dec_f_crc_1100) // gets status of 1st row keys
                            begin f_crc_status <= f_first_row_pgm; f_first_row_pgm <= 1'b0; end
                        if (dec_f_crc_1200) // set merge
                            f_crc_merge <= 1'b1;
                        if (dec_f_crc_1300) // test merge
                            begin f_crc_status <= f_crc_merge; f_crc_merge <= 1'b0; end
                        if (dec_f_crc_1400) // set pause
                            f_crc_pause <= 1'b1;
                        if (dec_f_crc_1500) // test pause
                            begin f_crc_status <= f_crc_pause; f_crc_pause <= 1'b0; end
                        if (dec_f_crc_1700) // r/w card
                            f_crc_pause <= 1'b1;
                    end
                end
                if (seq_jump)
                    f_carry <= 1'b0; // do not propagate current carry
                if (seq_alu_wback)
                    begin
                        if ((alu_set_carry & seq_wback_carry) | reg_p_set_carry) 
                            f_carry <= 1'b1;
                    end
            end


/***************************************************************************/
/***************************************************************************/
// Sequencer and HW Trace unit

always @(posedge cpu_clk or negedge cpu_reset_n)
    begin
         if (~cpu_reset_n)
            begin
                seq_state <= `ST_INIT;
                seq_opcode <= 10'h0;
                seq_opcode2 <= 10'h0;

                seq_ifaddr[11:0] <= 12'h0;
                seq_op1_nibble[3:0]  <= 4'h0;
                seq_dst_nibble[3:0]  <= 4'h0;
                seq_field_ofs <= 1'h0;
                seq_field_counter[3:0] <= 4'h0;
                delayed_rom_active <= 1'h0;
                delayed_rom[3:0] <= 4'h0;
                bank <= 1'h0;
                seq_wback_carry <= 1'b0; 
                seq_inc_da <= 1'b0;
                seq_dst <= 7'h0;
                seq_op1 <= 7'h0;
                seq_da_reg_cnt <= 4'h0;
                trace_tx_start <= 1'b0;
                trace_state <= 4'h0;
                trace_tx_cr <= 1'b0;
                trace_tx_space <= 1'b0;
                seq_write_path_b <= 1'b0;
                seq_write_path_a <= 1'b0;
                seq_fetch_op1_delayed <= 1'b0;
				SPC <= 4'h0;
            end
        else
			begin
				seq_fetch_op1_delayed <= seq_fetch_op1;
                case (seq_state)
                    `ST_INIT:
                        begin
                            seq_state <= #2 `ST_FETCH_OP0;
                            seq_wback_carry <= 1'b0; 
                            // display refresh
                            seq_inc_da <= 1'b0;
                            seq_op1 <= `OP1_A;
                            seq_dst <= `DST_B;
                            seq_op1_nibble <= disp_curr_nibble;
                            seq_dst_nibble <= disp_curr_nibble;
							SPC <= RPC[11:8];
                        end
                    `ST_FETCH_OP0: // fetch 1st word
                        begin
                            seq_state <= #2 `ST_FETCH_OP1; 
                            seq_fetched_addr <= RPC;
                            
                        end
                    `ST_FETCH_OP1: // store 1st word
                        begin
                            seq_opcode <= rom_opcode; 
                            // look for double word opcodes, second opcode is 10 bit address
                            if (({ rom_opcode[9:6], rom_opcode[1:0] } == 6'b1011_10 ) || //if 0 = b/c
                                ({ rom_opcode[9:6], rom_opcode[1:0] } == 6'b1100_10 ) || // if a >= b/c
                                ({ rom_opcode[9:6], rom_opcode[1:0] } == 6'b1101_10 ) || // if 0 # a/c
                                ({ rom_opcode[5:4], rom_opcode[2:0] } == 5'b01__100 ) ||  // if 1/0 = s
                                ({ rom_opcode[5:4], rom_opcode[2:0] } == 5'b10__100 ))    // if 0 #/= p
                                    seq_state <= #2 `ST_FETCH_OP2;
                                else
                                    begin
                                        if (HW_TRACE && trace_kbd_enable)
                                            seq_state <= #2 `ST_HW_TRACE_START;
                                        else
                                            seq_state <= #2 `ST_DECODE;
                                    end
                            //$display("%1h:%03h %1h:%04o: %s %s", bank, seq_fetched_addr, bank, seq_fetched_addr, dis_label, dis_opcode);
                            //$display("%1h:%03h %1h:%04o: %s %s A:%014x B:%014x", bank, seq_fetched_addr, bank, seq_fetched_addr, dis_label, dis_opcode);
							SPC <= RPC[7:4];
                        end
                    `ST_FETCH_OP2: // store second word if first was an if
                        begin
                            if (HW_TRACE && trace_kbd_enable)
                                seq_state <= #2 `ST_HW_TRACE_START;
                            else
                                seq_state <= #2 `ST_DECODE;
                            //seq_fetch_op1 <= 1'b0;
                            seq_ifaddr <= { RPC[11:10], rom_opcode };
                            seq_opcode2 <= rom_opcode;
                        end
                    `ST_HW_TRACE_START:
                        if (HW_TRACE)
                            begin
                                //seq_fetch_op1 <= 1'b0;
                                seq_op1 <= `OP1_0; //
                                trace_state <= 4'h0;
                                seq_state <= `ST_HW_TRACE_PREPARE;
                            end
                    `ST_HW_TRACE_PREPARE:
                        if (HW_TRACE)
                            begin
                                if (trace_state != 4'hD)
                                    begin
                                        if (HW_TRACE_MINI)
                                            begin
                                                if (trace_state == 4'h1)
                                                    trace_state <= 4'h8;
                                                else
                                                    trace_state <= trace_state + 4'd1;
                                            end
                                        else
                                            trace_state <= trace_state + 4'd1;
                                        seq_state <= #2 `ST_HW_TRACE_READ;
                                    end
                                else
                                    seq_state <= #2 `ST_DECODE;
                                case (trace_state)
                                    4'h0: begin seq_op1 <= `OP1_PC; seq_op1_nibble <= 4'h3; end
                                    4'h1: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    4'h2: begin seq_op1 <= `OP1_CNT; seq_op1_nibble <= 4'h2; end
                                    4'h3: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    4'h4: begin seq_op1 <= `OP1_0; seq_op1_nibble <= 4'h0; end
                                    4'h5: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    //4'h6: begin seq_op1 <= `OP1_F; seq_op1_nibble <= 4'h0; end
                                    //4'h7: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    4'h6: begin seq_op1 <= `OP1_KEY; seq_op1_nibble <= 4'h2; end
                                    4'h7: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    4'h8: begin seq_op1 <= `OP1_A; seq_op1_nibble <= 4'hD; end
                                    4'h9: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    4'hA: begin seq_op1 <= `OP1_B; seq_op1_nibble <= 4'hD; end
                                    4'hB: begin trace_tx_space <= 1'b1; seq_op1_nibble <= 4'h0; end
                                    4'hC: begin seq_op1 <= `OP1_C; seq_op1_nibble <= 4'hD; end
                                    4'hD: begin trace_tx_cr <= 1'b1; seq_op1_nibble <= 4'h0; end
                                endcase
                            end
                    `ST_HW_TRACE_READ:
                        if (HW_TRACE)
                            begin
                                // reads argument
                                trace_tx_start <= 1'b1;
                                seq_state <= `ST_HW_TRACE_OUTPUT;
                            end
                    `ST_HW_TRACE_OUTPUT:
                        if (HW_TRACE)
                            begin
                                trace_tx_space <= 1'b0;
                                trace_tx_cr <= 1'b0;
                                trace_tx_start <= 1'b0;
                                if ((~trace_tx_busy) & (~trace_tx_start))
                                    if (seq_op1_nibble == 4'h0)
                                        seq_state <= `ST_HW_TRACE_PREPARE;
                                    else
                                        begin
                                            seq_op1_nibble <= seq_op1_nibble - 4'h1;
                                            seq_state <= `ST_HW_TRACE_READ;
                                        end
                            end
                    `ST_DECODE:
                        begin
                            $display("%1h:%03h %1h:%04o: %03x %s", bank, seq_fetched_addr, bank, seq_fetched_addr, seq_opcode, dis_opcode);
                            if ((dec_op_alu != `ALU_NONE) || dec_f_rol || dec_f_lsl || dec_f_lsr || dec_f_tfr ||
                                dec_f_exchange || dec_f_wr_alu || dec_f_wr_lit || dec_f_push_c ||
                                dec_f_pop_a || dec_f_clr_regs || dec_f_clr_dregs || dec_f_rot_stack)
                                seq_state <= #2 `ST_AT_PREP; // execute
                            else
                                seq_state <= #2 `ST_EXE_NORM;
							SPC <= RPC[3:0];
                        end
                    `ST_EXE_NORM: // jumps, flags, reg-p opcodes
                        begin
                            if (dec_f_delayed)
                                begin
                                    delayed_rom_active <= 1'b1;
                                    delayed_rom <= seq_opcode[9:6];
                                end
                            if (dec_f_bank)
                                bank <= ~bank;  
                            if (dec_f_jump | //dec_f_bank | 
                                ((~reg_p_set_carry) & dec_f_gto_c_clr) | // actual carry is written back in this stage it cannot de checked now                    
                                ((~f_latched_carry) & dec_f_gonc))
                                seq_state <= #2 `ST_JUMP;
                            else
                                seq_state <= #2 `ST_INIT;
                        end
                    `ST_JUMP: // reached only if jsb, ifs or goto n/c when f_latched_carry is zero
                        begin
                            seq_state <= #2 `ST_INIT;
                                            
                            if (dec_f_use_delay_rom)
                                begin
                                    delayed_rom_active <= 1'b0;
                                    if (delayed_rom[3:2] == 2'b00) // clear bank on jump to first bank
                                        bank <= 1'b0;
                                end
                        end
                    `ST_AT_PREP: // get counters up to date
                        begin
                            seq_field_counter <= dec_field_end - dec_field_start; // use whatever the decoder provides: fixed or translated
                            seq_op1 <= dec_reg_op1;
                            seq_dst <= dec_reg_dst;
                            seq_field_ofs <= 1'b0; // means +1, from left to right for add/sub and tfr/ex
                            seq_state <=`ST_AT_READ;
                            seq_dst_nibble <= dec_field_start;
                            seq_op1_nibble <= dec_field_start;
                            seq_inc_da <= 1'b0;
                            if ((dec_op_alu == `ALU_EQ) || (dec_op_alu == `ALU_NEQ) || (dec_op_alu == `ALU_GTEQ))
                                begin
                                    seq_dst_nibble <= dec_field_end;
                                    seq_op1_nibble <= dec_field_end;                             
                                    seq_field_ofs <= 1'h1; // decrement pointer
                                end
                            if (dec_f_rol)
                                begin 
                                    seq_dst_nibble <= 4'he; // save left most nibble in unused nibble 15th
                                    seq_op1_nibble <= dec_field_end;                              
                                    seq_field_ofs <= 1'h1; // decrement pointer
                                    seq_field_counter <= 4'he;
                                end
                            else if (dec_f_lsl)
                                begin 
                                    seq_dst_nibble <= dec_field_end;
                                    seq_op1_nibble <= dec_field_end - 4'h1;                              
                                    seq_field_ofs <= 1'h1; // decrement pointer
                                end
                            else if(dec_f_lsr)
                                begin 
                                    seq_op1_nibble <= dec_op_field_start + 4'h1;
                                end
                            else if(dec_f_clr_dregs)
                                begin
                                    seq_dst_nibble <= 4'h0;
                                    seq_op1_nibble <= 4'h0;
                                    seq_field_counter <= 4'hd;
                                    seq_op1 <= `OP1_0;
                                    if (HP67)
                                        begin
                                            seq_da_reg_cnt <= 4'h0;
                                            seq_dst <= { 1'b0, DATAADDR }; // access data registers through DA pointer
                                        end
                                    else
                                        seq_dst <= `DST_R0;
                                end
                            else if(dec_f_clr_regs)
                                begin
                                    seq_dst_nibble <= 4'h0;
                                    seq_op1_nibble <= 4'h0;
                                    seq_field_counter <= 4'hd;
                                    seq_op1 <= `OP1_0;
                                    seq_dst <= `DST_A;
                                end
                            else if(dec_f_push_c) // c->y->z->t
                                begin
                                    seq_dst_nibble <= 4'h0;
                                    seq_op1_nibble <= 4'h0;
                                    seq_field_counter <= 4'hd;
                                    seq_op1 <= `OP1_Z;
                                    seq_dst <= `DST_T;
                                end
                            else if(dec_f_pop_a) // t->z->y->a
                                begin
                                    seq_dst_nibble <= 4'h0;
                                    seq_op1_nibble <= 4'h0;
                                    seq_field_counter <= 4'hd;
                                    seq_op1 <= `OP1_Y;
                                    seq_dst <= `DST_A;
                                end
                            else if(dec_f_rot_stack) // 
                                begin
                                    seq_dst_nibble <= 4'h0;
                                    seq_op1_nibble <= 4'h0;
                                    seq_field_counter <= 4'hd;
                                    seq_op1 <= `OP1_C;
                                    seq_dst <= `DST_T;
                                end
                        end
                    `ST_AT_READ:
                        begin
                            seq_inc_da <= 1'b0;
                            seq_state <= `ST_AT_READ3;
                        end
                    `ST_AT_READ3:
                        begin
                            seq_state <= `ST_AT_WBACK;
                            if (seq_field_counter == 4'h0)
                               seq_wback_carry <= dec_f_set_carry;
                               
                            seq_write_path_b <= seq_reg_transfer | seq_reg_write_alu_or_lit | seq_reg_exchange;
                            seq_write_path_a <= seq_reg_exchange;
                        end
                    `ST_AT_WBACK:
                        begin
                            seq_state <= `ST_AT_READ2;
                            seq_write_path_b <= 1'b0;
                            seq_write_path_a <= 1'b0;
                        end
                    `ST_AT_READ2:
                        begin
                            seq_state <= `ST_AT_READ;
                            if (seq_field_counter != 4'h0)
                                begin
                                    seq_dst_nibble <= seq_dst_nibble + (seq_field_ofs ? 4'hf:4'h1);
                                    seq_op1_nibble <= seq_op1_nibble + (seq_field_ofs ? 4'hf:4'h1);
                                end
                            case (dec_op_alu)
                                `ALU_EQ, `ALU_NEQ, `ALU_GTEQ:
                                    if (seq_field_counter == 4'h0) seq_state <= `ST_JUMP; // jump if carry
                            endcase
                            if (dec_f_lsl | dec_f_lsr) // don't merge with next case
                                begin
                                    if (seq_field_counter == 4'h1)
                                        seq_op1 <= `OP1_0; // shift a zero from the left or right 
                                end
                            if (dec_f_rol) // don't merge with next case
                                begin
                                    if (seq_field_counter == 4'h1)
                                        seq_op1_nibble <= 4'he; // shift nibble D, saved in nibble E, to niblle 0
                                end
                            if (dec_f_clr_dregs) // cycle through all data registers (R0..R15)
                                begin
                                    if (seq_field_counter != 4'h0)
                                        seq_field_counter <= seq_field_counter - 4'h1;
                                    else
                                        begin
                                            seq_dst_nibble <= 4'h0;
                                            seq_op1_nibble <= 4'h0;
                                            seq_field_counter <= 4'hd;
                                            
                                            if (seq_dst[3:0] == 4'hf)
                                                seq_state <= `ST_INIT;
                                            else
                                                seq_dst[3:0] <= seq_dst[3:0] + 4'h1;
                                        end
                                end
                            else if (dec_f_clr_regs) // cycle through all registers(A, B, C, Mx, Y, Z, T)
                                begin
                                    if (seq_field_counter != 4'h0)
                                        seq_field_counter <= seq_field_counter - 4'h1;
                                    else
                                        begin
                                            seq_dst_nibble <= 4'h0;
                                            seq_op1_nibble <= 4'h0;
                                            seq_field_counter <= 4'hd;
                                            if (seq_dst == `DST_T)
                                                seq_state <= `ST_INIT;
                                            else
                                                seq_dst[3:0] <= seq_dst[3:0] + 4'd1;
                                        end
                                end
                            else if (dec_f_push_c) // c->y->z->t
                                begin
                                    if (seq_field_counter != 4'h0)
                                        seq_field_counter <= seq_field_counter - 4'h1;
                                    else
                                        begin
                                            seq_dst_nibble <= 4'h0;
                                            seq_op1_nibble <= 4'h0;
                                            seq_field_counter <= 4'hd;
                                            case (seq_op1)
                                                `OP1_Z: begin seq_op1 <= `OP1_Y; seq_dst <= `DST_Z; end
                                                `OP1_Y: begin seq_op1 <= `OP1_C; seq_dst <= `DST_Y; end
                                                `OP1_C: begin seq_state <= `ST_INIT; end
                                            endcase
                                        end
                                end
                            else if (dec_f_pop_a) // t->z->y->a
                                begin
                                    if (seq_field_counter != 4'h0)
                                        seq_field_counter <= seq_field_counter - 4'h1;
                                    else
                                        begin
                                            seq_dst_nibble <= 4'h0;
                                            seq_op1_nibble <= 4'h0;
                                            seq_field_counter <= 4'hd;
                                            case (seq_op1)
                                                `OP1_Y: begin seq_op1 <= `OP1_Z; seq_dst <= `DST_Y; end
                                                `OP1_Z: begin seq_op1 <= `OP1_T; seq_dst <= `DST_Z; end
                                                `OP1_T: begin seq_state <= `ST_INIT; end
                                            endcase
                                        end
                                end
                            else if (dec_f_rot_stack) // uses 3 exchanges
                                begin
                                    if (seq_field_counter != 4'h0)
                                        seq_field_counter <= seq_field_counter - 4'h1;
                                    else
                                        begin
                                            seq_dst_nibble <= 4'h0;
                                            seq_op1_nibble <= 4'h0;
                                            seq_field_counter <= 4'hd;
                                            case (seq_op1)
                                                `OP1_C: begin seq_op1 <= `OP1_Y; seq_dst <= `DST_C; end
                                                `OP1_Y: begin seq_op1 <= `OP1_Z; seq_dst <= `DST_Y; end
                                                `OP1_Z: begin seq_state <= `ST_INIT; end
                                            endcase
                                        end
                                end//
                            else
                                begin
                                    if (seq_field_counter != 4'h0)
                                        seq_field_counter <= seq_field_counter - 4'h1;
                                    else
                                        begin
                                            if (dec_f_gto_c_clr | dec_f_gto_c_set)
                                                seq_state <= #2 `ST_JUMP;
                                            else
                                                seq_state <= `ST_INIT;
                                        end
                                end
                        end
                    default:
                        seq_state <= `ST_INIT;
                endcase
            end
    end
    
initial
    begin
    end

endmodule


module ws_field_decoder(
    input wire [2:0] field_in, // 3 left most bits
    input wire [3:0] p_in, // register P
    output reg [3:0] start_o, 
    output reg [3:0] end_o
    );

always @(field_in, p_in)
    case(field_in)
        3'h0: begin start_o = p_in; end_o = p_in; end // Pointer field
        3'h1: begin start_o = 4'h0; end_o = p_in; end // WP
        3'h2: begin start_o = 4'h2; end_o = 4'h2; end // XS
        3'h3: begin start_o = 4'h0; end_o = 4'h2; end // X
        3'h4: begin start_o = 4'hd; end_o = 4'hd; end // S
        3'h5: begin start_o = 4'h3; end_o = 4'hc; end // M
        3'h6: begin start_o = 4'h0; end_o = 4'hd; end // W
        3'h7: begin start_o = 4'h3; end_o = 4'hd; end // MS
    endcase

endmodule

module ws_loadto_p(
    input wire [3:0] value_in,
    output reg [3:0] p_o
    );
    
always @(value_in)
    case (value_in)
        4'b0000: p_o = 4'he; 
        4'b0001: p_o = 4'h4;
        4'b0010: p_o = 4'h7;
        4'b0011: p_o = 4'h8;
        4'b0100: p_o = 4'hb;
        4'b0101: p_o = 4'h2;
        4'b0110: p_o = 4'ha;
        4'b0111: p_o = 4'hc;
        4'b1000: p_o = 4'h1;
        4'b1001: p_o = 4'h3;
        4'b1010: p_o = 4'hd;
        4'b1011: p_o = 4'h6;
        4'b1100: p_o = 4'h0;
        4'b1101: p_o = 4'h9;
        4'b1110: p_o = 4'h5;
        4'b1111: p_o = 4'he; 
    endcase
endmodule

module ws_cmpwith_p(
    input wire [3:0] value_in,
    output reg [3:0] p_o
    );
    
always @(value_in)
    case (value_in) // Scrambled to real P. Not all values are used
        4'b0000: p_o = 4'h4; 
        4'b0001: p_o = 4'h8;
        4'b0010: p_o = 4'hc;
        4'b0011: p_o = 4'h2;
        4'b0100: p_o = 4'h9;
        4'b0101: p_o = 4'h1;
        4'b0110: p_o = 4'h6;
        4'b0111: p_o = 4'h3;
        4'b1000: p_o = 4'h1;
        4'b1001: p_o = 4'hd;
        4'b1010: p_o = 4'h5;
        4'b1011: p_o = 4'h0;
        4'b1100: p_o = 4'hb;
        4'b1101: p_o = 4'ha;
        4'b1110: p_o = 4'h7;
        4'b1111: p_o = 4'h4; 
        default: p_o = 4'h4; 
    endcase
endmodule

/**
 * 
 *
 */
`ifdef LATTICE_MXO2
/*
module sync_rom(
    input wire          clk_in,
    
    input wire [12:0]   addr_in,
    output wire [9:0]   data_o
    );

wire [12:0] new_addr;
assign new_addr = addr_in[12:10] == 3'b000 ? { 1'b0, addr_in }:
                  addr_in[12:10] == 3'b001 ? { 1'b0, addr_in }:
                  addr_in[12:10] == 3'b010 ? { 1'b0, addr_in }:
                  addr_in[12:10] == 3'b011 ? { 1'b0, addr_in }:
                  addr_in[12:10] == 3'b100 ? { 1'b0, addr_in }:
                  addr_in[12:10] == 3'b101 ? { 3'b100, addr_in[9:0] }: // bank 1 is located in the 4096..5119 area
                  addr_in[12:10] == 3'b110 ? { 1'b0, addr_in }:
                                             { 1'b0, addr_in };

wire [8:0] mem8_0;
wire mem9;

bits8_0 b08 (.Address(new_addr), .OutClock(clk_in), .OutClockEn(1'b1), .Reset(1'b0), .Q(mem8_0));
bits9 b9 (.Address(new_addr), .OutClock(clk_in), .OutClockEn(1'b1), .Reset(1'b0), .Q(mem9));

assign data_o = { mem9, mem8_0 };
    
endmodule
*/
module sync_rom(
    input wire          clk_in,
    
    input wire [12:0]   addr_in,
    output reg [9:0]    data_o
    );

wire [12:0] new_addr;
assign new_addr = addr_in[12:10] == 3'b000 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b001 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b010 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b011 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b100 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b101 ? { 3'b100, addr_in[9:0] }: // bank 1 is located in the 4096..5119 area
                  addr_in[12:10] == 3'b110 ? { 1'b0, addr_in[11:0] }:
                                             { 1'b0, addr_in[11:0] };
reg [9:0] mem[5119:0];

always @(posedge clk_in)
    begin
        data_o <= #5 mem[new_addr];
    end

    
integer i;
initial
    begin
        $readmemb("../../01_Verilog/hp-67.bin", mem);
    end
    
endmodule

`else
module sync_rom(
    input wire          clk_in,
    
    input wire [12:0]   addr_in,
    output reg [9:0]    data_o
    );

wire [12:0] new_addr;
assign new_addr = addr_in[12:10] == 3'b000 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b001 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b010 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b011 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b100 ? { 1'b0, addr_in[11:0] }:
                  addr_in[12:10] == 3'b101 ? { 3'b100, addr_in[9:0] }: // bank 1 is located in the 4096..5119 area
                  addr_in[12:10] == 3'b110 ? { 1'b0, addr_in[11:0] }:
                                             { 1'b0, addr_in[11:0] };
reg [9:0] mem[5119:0];

always @(posedge clk_in)
    begin
        data_o <= #5 mem[new_addr];
    end

    
integer i;
initial
    begin
`ifdef CPU_TEST
        $readmemb("cpu_test.bin", mem);
`else
        $readmemb("hp-67.bin", mem);
`endif
    end
    
endmodule

module dp_reg_bank(
    input wire          clk_in,
    input wire          seq_decode_in,
    
    input wire          wea_in,
    input wire [3:0]    dataa_in,
    input wire [10:0]   addra_in,
    output reg [3:0]    dataa_o,
    
    input wire          web_in,
    input wire [3:0]    datab_in,
    input wire [10:0]   addrb_in,
    output reg [3:0]    datab_o
    );
    
reg [3:0] regs[2047:0];

wire [55:0] A, B, C, M1, M2;

assign A = {                                                             regs[ 80 * 16 + 0 * 16 + 13], regs[ 80 * 16 + 0 * 16 + 12], 
             regs[ 80 * 16 + 0 * 16 + 11], regs[ 80 * 16 + 0 * 16 + 10], regs[ 80 * 16 + 0 * 16 +  9], regs[ 80 * 16 + 0 * 16 +  8], 
             regs[ 80 * 16 + 0 * 16 +  7], regs[ 80 * 16 + 0 * 16 +  6], regs[ 80 * 16 + 0 * 16 +  5], regs[ 80 * 16 + 0 * 16 +  4], 
             regs[ 80 * 16 + 0 * 16 +  3], regs[ 80 * 16 + 0 * 16 +  2], regs[ 80 * 16 + 0 * 16 +  1], regs[ 80 * 16 + 0 * 16 +  0] };
assign B = {                                                             regs[ 80 * 16 + 1 * 16 + 13], regs[ 80 * 16 + 1 * 16 + 12], 
             regs[ 80 * 16 + 1 * 16 + 11], regs[ 80 * 16 + 1 * 16 + 10], regs[ 80 * 16 + 1 * 16 +  9], regs[ 80 * 16 + 1 * 16 +  8], 
             regs[ 80 * 16 + 1 * 16 +  7], regs[ 80 * 16 + 1 * 16 +  6], regs[ 80 * 16 + 1 * 16 +  5], regs[ 80 * 16 + 1 * 16 +  4], 
             regs[ 80 * 16 + 1 * 16 +  3], regs[ 80 * 16 + 1 * 16 +  2], regs[ 80 * 16 + 1 * 16 +  1], regs[ 80 * 16 + 1 * 16 +  0] };
assign C = {                                                             regs[ 80 * 16 + 2 * 16 + 13], regs[ 80 * 16 + 2 * 16 + 12], 
             regs[ 80 * 16 + 2 * 16 + 11], regs[ 80 * 16 + 2 * 16 + 10], regs[ 80 * 16 + 2 * 16 +  9], regs[ 80 * 16 + 2 * 16 +  8], 
             regs[ 80 * 16 + 2 * 16 +  7], regs[ 80 * 16 + 2 * 16 +  6], regs[ 80 * 16 + 2 * 16 +  5], regs[ 80 * 16 + 2 * 16 +  4], 
             regs[ 80 * 16 + 2 * 16 +  3], regs[ 80 * 16 + 2 * 16 +  2], regs[ 80 * 16 + 2 * 16 +  1], regs[ 80 * 16 + 2 * 16 +  0] };
assign M1 = {                                                            regs[ 80 * 16 + 3 * 16 + 13], regs[ 80 * 16 + 3 * 16 + 12], 
             regs[ 80 * 16 + 3 * 16 + 11], regs[ 80 * 16 + 3 * 16 + 10], regs[ 80 * 16 + 3 * 16 +  9], regs[ 80 * 16 + 3 * 16 +  8], 
             regs[ 80 * 16 + 3 * 16 +  7], regs[ 80 * 16 + 3 * 16 +  6], regs[ 80 * 16 + 3 * 16 +  5], regs[ 80 * 16 + 3 * 16 +  4], 
             regs[ 80 * 16 + 3 * 16 +  3], regs[ 80 * 16 + 3 * 16 +  2], regs[ 80 * 16 + 3 * 16 +  1], regs[ 80 * 16 + 3 * 16 +  0] };
assign M2 = {                                                            regs[ 80 * 16 + 4 * 16 + 13], regs[ 80 * 16 + 4 * 16 + 12], 
             regs[ 80 * 16 + 4 * 16 + 11], regs[ 80 * 16 + 4 * 16 + 10], regs[ 80 * 16 + 4 * 16 +  9], regs[ 80 * 16 + 4 * 16 +  8], 
             regs[ 80 * 16 + 4 * 16 +  7], regs[ 80 * 16 + 4 * 16 +  6], regs[ 80 * 16 + 4 * 16 +  5], regs[ 80 * 16 + 4 * 16 +  4], 
             regs[ 80 * 16 + 4 * 16 +  3], regs[ 80 * 16 + 4 * 16 +  2], regs[ 80 * 16 + 4 * 16 +  1], regs[ 80 * 16 + 4 * 16 +  0] };

always @(posedge clk_in)
    begin
        if (seq_decode_in)
            $display("              A: %014x B: %014x C: %014x M1: %014x M2: %014x %8t", A, B, C, M1, M2, $time);
        if (wea_in)
            regs[addra_in] <= dataa_in;
        dataa_o <= #2 regs[addra_in];
        
        if (web_in)
            regs[addrb_in] <= datab_in;
        datab_o <= #2 regs[addrb_in];
    end

integer i;
initial
    begin
        $display("Using simulated dp-ram");
        for (i = 32'd0; i <= 32'd2047; i = i + 32'd1)
            begin
                regs[i[11:0]] = 4'd0;
            end
        
    end

endmodule

`endif

/*
 * Scans a 8x5 keyboard matrix
 *
 * col1 col2 col3 col4 col5
 *  A    B    C    D    E     R7
 * E+   GTO  DSP  (i)  SST    R6
 *  f    g   STO  RCL   h     R5
 * Enter     CHS  EEX  CLx    R4
 *  -    7    8    9          R3
 *  +    4    5    6          R2
 *  x    1    2    3          R1
 *  /    0    .    R/S        R0
 */     


/*
 * Scans a landscape-styled keyboard 
 * The 67 needs only the kbd_keycode, doesn't care how the keyboard looks like or how is it
 * layed out
 *
 *row   --------------   col   --------------   row
 *      0   1   2   3   4   4   3   2   1   0 
 * 0   .A. .B. .C. .D. .E. CHS .7. .8. .9. ./.   4
 * 1   R/S SST DSP GTO (i) EEX .4. .5. .6. .x.   5
 * 2   P/R .h. Rv x<>y CLX .E. .1. .2. .3. .-.   6
 * 3   ... .f. .g. STO RCL .N. .0.  .  .E+ .+.   7
 *
 *
 * 00 00 00 00 00 00
 * 00 
 *
 */

module ws_keys_landscape(
    input wire          clk_in,             // processor clock
    input wire          reset_in,
    input wire [7:0]    keys_rows_in,
    input wire          key_read_ack_in,    // asserted when the last key was read
    input wire          hw_flag_clr_in,
    input wire [3:0]    operand_in,
    output wire [4:0]   key_cols_o,
    output wire [7:0]   keycode_o,
    output wire         key_pending_o,
    output wire         pgm_mode_o,
    output wire         on_off_o,
    output wire         switch_0_o,         // keyboard switches for special functions
    output wire         switch_1_o          // keyboard switches for special functions
    
`ifdef SIMULATOR
    ,
    input wire          simkbd_activate_key_pending_in,
    input wire [7:0]    simkbd_keycode_in
`endif
    );

reg keys_col0_p0, keys_col0_p1, keys_col0_p2;
reg keys_col1_p0, keys_col1_p1, keys_col1_p2;
reg keys_col2_p0, keys_col2_p1, keys_col2_p2;
reg keys_col3_p0, keys_col3_p1, keys_col3_p2;
reg keys_col4_p0, keys_col4_p1, keys_col4_p2;

reg [12:0] scan_tick;
reg [7:0] key;
reg kbd_pending;
reg [3:0] idx;
reg pgm_mode;
reg [6:0] curr_key;
reg on_off = 1'b0;
reg process_new_key = 1'b0;

reg switch_0 = 1'b0;
reg switch_1 = 1'b0;

reg switch_0_pre = 1'b0;
reg switch_1_pre = 1'b0;

assign pgm_mode_o = pgm_mode;
assign on_off_o = on_off;

assign switch_0_o = switch_0;
assign switch_1_o = switch_1;

assign keycode_o = key;
assign key_pending_o = kbd_pending;

assign key_cols_o[0] = ({ scan_tick[12:9], scan_tick[7:3] } == 12'b0_000__0000_0___) | on_off; // used to wake up the calc
assign key_cols_o[1] = { scan_tick[12:9], scan_tick[7:3] } == 12'b0_000__0001_0___;
assign key_cols_o[2] = { scan_tick[12:9], scan_tick[7:3] } == 12'b0_000__0010_0___;
assign key_cols_o[3] = { scan_tick[12:9], scan_tick[7:3] } == 12'b0_000__0011_0___;
assign key_cols_o[4] = { scan_tick[12:9], scan_tick[7:3] } == 12'b0_000__0100_0___;

wire col0_x, col1_x, col2_x, col3_x, col4_x;
// change detection
assign col0_x = { keys_col0_p0, keys_col0_p1 & keys_col0_p2} == 2'b01; // _/--
assign col1_x = { keys_col1_p0, keys_col1_p1 & keys_col1_p2} == 2'b01;
assign col2_x = { keys_col2_p0, keys_col2_p1 & keys_col2_p2} == 2'b01;
assign col3_x = { keys_col3_p0, keys_col3_p1 & keys_col3_p2} == 2'b01;
assign col4_x = { keys_col4_p0, keys_col4_p1 & keys_col4_p2} == 2'b01;

/*
 *row   --------------   col   --------------   row
 *      0   1   2   3   4   4   3   2   1   0 
 * 0   .A. .B. .C. .D. .E. CHS .7. .8. .9. ./.   4
 * 1   R/S SST DSP GTO (i) EEX .4. .5. .6. .x.   5
 * 2   P/R .h. Rv x<>y CLX .E. .1. .2. .3. .-.   6
 * 3   ... .f. .g. STO RCL .N. .0.  .  .E+ .+.   7

 *      00  00  00  00  00  00  42  41  40  00
 *      00  00  00  00  00  00  52  51  50  00
 *      00  00  00  00  00  00  62  61  60  00
 *      00  00  00  00  00  00  72  71  94  00
 *
*/ 
//`define WITH_EXTRA_CLR 1


always @(*)
    case ({ scan_tick[6:4], scan_tick[2:0] })
        6'h00: curr_key = 7'h37; // /
        6'h01: curr_key = 7'h36; // *
        6'h02: curr_key = 7'h34; // -
        6'h03: curr_key = 7'h35; // +
        
        6'h04: curr_key = 7'h4A; //  A
        6'h05: curr_key = 7'h07; // R/S
        6'h06: curr_key = 7'h38; // P/R
        6'h07: curr_key = 7'h3B; // ON/OFF
        
        6'h08: curr_key = 7'h04; // 9
        6'h09: curr_key = 7'h05; // 6
        6'h0a: curr_key = 7'h06; // 3
        6'h0b: curr_key = 7'h49; // E+
        
        6'h0c: curr_key = 7'h3A; //  B
        6'h0d: curr_key = 7'h09; // SST
        6'h0e: curr_key = 7'h01; // h
        6'h0f: curr_key = 7'h41; // f
        
        6'h10: curr_key = 7'h14; // 8
        6'h11: curr_key = 7'h15; // 5
        6'h12: curr_key = 7'h16; // 2
        6'h13: curr_key = 7'h17; // .
        
        6'h14: curr_key = 7'h2A; //  C
        6'h15: curr_key = 7'h29; // DSP
        6'h16: curr_key = 7'h1A; // Rv /// check if this one changes when the keys A..E lose their alternate function
        6'h17: curr_key = 7'h31; //  g
        
        6'h18: curr_key = 7'h24; // 7
        6'h19: curr_key = 7'h25; // 4
        6'h1a: curr_key = 7'h26; // 1
        6'h1b: curr_key = 7'h27; // 0
        
        6'h1c: curr_key = 7'h1A; //  D
        6'h1d: curr_key = 7'h39; // GTO
        6'h1e: curr_key = 7'h0A; // x<>y /// check if this one changes when the keys A..E lose their alternate function
        6'h1f: curr_key = 7'h21; // STO
        
        6'h20: curr_key = 7'h23; // CHS
        6'h21: curr_key = 7'h13; // EEX
        6'h22: curr_key = 7'h33; // ENTER
        6'h23: curr_key = 7'h33; // ENTER
        
        6'h24: curr_key = 7'h0A; //  E
        6'h25: curr_key = 7'h19; // (i)
        6'h26: curr_key = 7'h03; // CLX
        6'h27: curr_key = 7'h11; // RCL
        default: 
            curr_key = 7'h00; 
    endcase

    reg [6:0] latched_curr_key;
`define WITH_EXTRA_CLR 1
always @(posedge clk_in)
    begin
        if (~reset_in)
            begin
                on_off <= 1'b0;
                pgm_mode <= 1'b0;
                kbd_pending <= 1'b0;
                scan_tick <= 13'h0;
                key <= 8'h0;
                process_new_key <= 1'b0;
                switch_0 <= 1'b0;
                switch_1 <= 1'b0;

                switch_0_pre <= 1'b0;
                switch_1_pre <= 1'b0;
                latched_curr_key <= 7'h0;
            end
        else
            begin
    
`ifdef WITH_EXTRA_CLR
                if (key_read_ack_in || (hw_flag_clr_in && (operand_in == 4'hf)))
                    begin
                        kbd_pending <= 1'b0;//(keys_col0_p0 | keys_col1_p0 | keys_col2_p0 | keys_col3_p0 | keys_col4_p0);
                        scan_tick <= 13'h0;
                    end
`else
                if (key_read_ack_in)
                    begin // keep the key-pressed asserted as long as a key is pressed
                        kbd_pending <= 1'b0;
                        //if (kbd_pending)
                        //    kbd_pending <= (keys_col0_p0 | keys_col1_p0 | keys_col2_p0 | keys_col3_p0 | keys_col4_p0);
                end
`endif
            scan_tick <= scan_tick + 13'h1;                
                
`ifdef SIMULATOR
                if (simkbd_activate_key_pending_in)
                    begin
                        case (simkbd_keycode_in)
`else
                if (process_new_key)
                    begin
                        case (latched_curr_key)
`endif                            
                            7'h01:
								begin
									switch_0_pre <= 1'b1; // pre condition for kbd switches
									switch_1_pre <= 1'b1; // pre condition for kbd switches
								end
                            7'h41:
                                if (switch_0_pre)
                                    begin
                                        switch_0_pre <= 1'b0;
                                        switch_0 <= switch_0 ^ 1'b1;
                                    end
                            7'h31:
                                if (switch_1_pre)
                                    begin
                                        switch_1_pre <= 1'b0;
                                        switch_1 <= switch_1 ^ 1'b1;
                                    end
                            default:
                                begin
                                    switch_0_pre <= 1'b0; // pre condition for kbd switches
                                    switch_1_pre <= 1'b0; // pre condition for kbd switches
                                end
                        endcase
`ifdef SIMULATOR
                        case (simkbd_keycode_in)
`else
                        case (latched_curr_key)
`endif
                            7'h3B:
                                on_off <= 1'b1; // enable oscillator stand-by
                            7'h38:
                                pgm_mode <= pgm_mode ^ 1'b1;
                            default:
                                begin
`ifdef SIMULATOR
                                    key <= simkbd_keycode_in[3:0] == 4'h0 ? 8'h00:{ simkbd_keycode_in[3:0], 1'b0, simkbd_keycode_in[6:4] };
                                    kbd_pending <= simkbd_keycode_in != 7'h00;
`else
                                    key <= latched_curr_key[3:0] == 4'h0 ? 8'h00:{ latched_curr_key[3:0], 1'b0, latched_curr_key[6:4] };
                                    kbd_pending <= latched_curr_key != 7'h00;
`endif
                                end
                        endcase
                        process_new_key <= 1'b0;
                    end
                case (scan_tick)
                    13'h00:
                        begin
                            keys_col0_p0 <= keys_col0_p1;
                            keys_col0_p1 <= keys_col0_p2;
                            keys_col0_p2 <= |keys_rows_in;
                        end
                    13'h10:
                        begin
                            keys_col1_p0 <= keys_col1_p1;
                            keys_col1_p1 <= keys_col1_p2;
                            keys_col1_p2 <= |keys_rows_in;
                        end
                    13'h20:
                        begin
                            keys_col2_p0 <= keys_col2_p1;
                            keys_col2_p1 <= keys_col2_p2;
                            keys_col2_p2 <= |keys_rows_in;
                        end
                    13'h30:
                        begin
                            keys_col3_p0 <= keys_col3_p1;
                            keys_col3_p1 <= keys_col3_p2;
                            keys_col3_p2 <= |keys_rows_in;
                        end
                    13'h40:
                        begin
                            keys_col4_p0 <= keys_col4_p1;
                            keys_col4_p1 <= keys_col4_p2;
                            keys_col4_p2 <= |keys_rows_in;
                        end
                    13'h100, 13'h101, 13'h102, 13'h103, 13'h104, 13'h105, 13'h106, 13'h107: // col 0
                        begin
                            if (col0_x & (~kbd_pending) )
                                begin
                                    if (keys_rows_in[scan_tick[2:0]] == 1'b1)
                                        begin
                                            process_new_key <= 1'b1;
                                            latched_curr_key <= curr_key;
                                        end
                                end
                        end
                    13'h110, 13'h111, 13'h112, 13'h113, 13'h114, 13'h115, 13'h116, 13'h117:
                        begin
                            if (col1_x & (~kbd_pending) )
                                begin
                                    if (keys_rows_in[scan_tick[2:0]] == 1'b1)
                                        begin
                                            process_new_key <= 1'b1;
                                            latched_curr_key <= curr_key;
                                        end
                                end
                        end
                    13'h120, 13'h121, 13'h122, 13'h123, 13'h124, 13'h125, 13'h126, 13'h127:
                        begin
                            if (col2_x & (~kbd_pending) )
                                begin
                                    if (keys_rows_in[scan_tick[2:0]] == 1'b1)
                                        begin
                                            process_new_key <= 1'b1;
                                            latched_curr_key <= curr_key;
                                        end
                                end
                        end
                    13'h130, 13'h131, 13'h132, 13'h133, 13'h134, 13'h135, 13'h136, 13'h137:
                        begin
                            if (col3_x & (~kbd_pending) )
                                begin
                                    if (keys_rows_in[scan_tick[2:0]] == 1'b1)
                                       begin
                                            process_new_key <= 1'b1;
                                            latched_curr_key <= curr_key;
                                       end
                                end
                        end
                    13'h140, 13'h141, 13'h142, 13'h143, 13'h144, 13'h145, 13'h146, 13'h147:
                        begin
                            if (col4_x & (~kbd_pending) )
                                begin
                                    if (keys_rows_in[scan_tick[2:0]] == 1'b1)
                                        begin
                                            process_new_key <= 1'b1;
                                            latched_curr_key <= curr_key;
                                        end
                                end
                        end
                endcase
            end
    end
initial
    begin
        key = 0;
        scan_tick = 0;
        kbd_pending = 0;
    end
endmodule


module addbcd(
    input wire [3:0] a_in,
    input wire [3:0] b_in,
    input wire c_in,
    input wire dec_in,
    output wire [3:0] q_out,
    output wire qc_out
);

wire [4:0] a_plus_b_plus_c;
wire [3:0] fixed_a_plus_b_plus_c;

assign a_plus_b_plus_c = { 1'b0, a_in } + { 1'b0, b_in } + { 4'h0, c_in };
assign qc_out = (dec_in & (a_plus_b_plus_c > 5'h09)) | a_plus_b_plus_c[4];
assign fixed_a_plus_b_plus_c = a_plus_b_plus_c[3:0] + 4'h6;
assign q_out = (dec_in & qc_out) ? fixed_a_plus_b_plus_c:a_plus_b_plus_c[3:0];

endmodule


module subbcd(
    input wire [3:0] a_in,
    input wire [3:0] b_in,
    input wire c_in,
    input wire dec_in,
    output wire [3:0] q_out,
    output wire qc_out
);

wire [3:0] c9;
wire [4:0] sub;

assign sub = { 1'b1, a_in } - { 1'b0, b_in } - { 4'b0, c_in };
assign c9 = sub[3:0] - 4'h6;
assign qc_out = ~sub[4]; // carry flag is base independent
assign q_out = (dec_in & (~sub[4])) ? c9:sub[3:0];

endmodule

module comp_unit(
    input wire [3:0] left_in, 
    input wire [3:0] right_in, 
    
    output wire equal_o,
    output wire notequal_o,
    output wire gt_o
    );

wire equal;

assign equal = left_in == right_in;

assign equal_o = equal;
assign notequal_o = !equal;

wire greater = left_in > right_in;

assign gt_o = greater;

endmodule
    

module async_transmitter(
    input wire clk,
    input wire TxD_start,
    input wire [7:0] TxD_data,
    output wire TxD,
    output wire TxD_busy
);

////////////////////////////////

wire BitTick = 1'b1;  // output one bit per clock cycle

reg [3:0] TxD_state = 0;
wire TxD_ready = (TxD_state==0);
assign TxD_busy = ~TxD_ready;

reg [7:0] TxD_shift = 0;
always @(posedge clk)
begin
    if(TxD_ready & TxD_start)
        TxD_shift <= TxD_data;
    else
    if(TxD_state[3] & BitTick)
        TxD_shift <= (TxD_shift >> 1);

    case(TxD_state)
        4'b0000: if(TxD_start) TxD_state <= 4'b0100;
        4'b0100: if(BitTick) TxD_state <= 4'b1000;  // start bit
        4'b1000: if(BitTick) TxD_state <= 4'b1001;  // bit 0
        4'b1001: if(BitTick) TxD_state <= 4'b1010;  // bit 1
        4'b1010: if(BitTick) TxD_state <= 4'b1011;  // bit 2
        4'b1011: if(BitTick) TxD_state <= 4'b1100;  // bit 3
        4'b1100: if(BitTick) TxD_state <= 4'b1101;  // bit 4
        4'b1101: if(BitTick) TxD_state <= 4'b1110;  // bit 5
        4'b1110: if(BitTick) TxD_state <= 4'b1111;  // bit 6
        4'b1111: if(BitTick) TxD_state <= 4'b0010;  // bit 7
        4'b0010: if(BitTick) TxD_state <= 4'b0011;  // stop1
        4'b0011: if(BitTick) TxD_state <= 4'b0000;  // stop2
        default: if(BitTick) TxD_state <= 4'b0000;
    endcase
end

assign TxD = (TxD_state<4) | (TxD_state[3] & TxD_shift[0]);  // put together the start, data and stop bits
endmodule

