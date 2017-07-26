/*
 * Woodstock definitions
 * (c) Copyright 2014-2016 R.A. Paz S.
 * 
 * This software is provided "as-is" without any warranty
 * as described by the GPL v2
 *
 */
`default_nettype none

 // ALU opcodes
`define ALU_NONE 		3'h0
`define ALU_ADD 		3'h1
`define ALU_SUB 		3'h2
`define ALU_RSUB        3'h3
`define ALU_EQ			3'h4
`define ALU_NEQ			3'h5
`define ALU_GTEQ		3'h6

/* reg opcodes */
`define R_NONE 			4'h0
`define R_LSL 			4'h1
`define R_LSR 			4'h2
`define R_TFR 			4'h3
`define R_EX			4'h4
`define R_CLRDREGS		4'h5
`define R_CLRREGS		4'h6
`define R_PUSH_C		4'h7
`define R_POP_A			4'h8
`define R_STACK_DOWN	4'h9
/* compare unit */
`define CMP_NONE		3'h0
`define CMP_EQ			3'h1
`define CMP_NEQ			3'h2
`define CMP_LT			3'h3
`define CMP_LTEQ		3'h4
`define CMP_GT			3'h5
`define CMP_GTEQ		3'h6

`define ST_INIT 		4'd0
`define ST_FETCH_OP0	4'd1
`define ST_FETCH_OP1	4'd2
`define ST_FETCH_OP2	4'd3
`define ST_HW_TRACE_START   4'd4
`define ST_HW_TRACE_PREPARE 4'd5
`define ST_HW_TRACE_READ    4'd6
`define ST_HW_TRACE_OUTPUT  4'd7
`define ST_DECODE       4'd8
`define ST_EXE_NORM		4'd9
`define ST_JUMP         4'd10
`define ST_AT_PREP		4'd11
`define ST_AT_READ		4'd12
`define ST_AT_READ3		4'd13
`define ST_AT_WBACK		4'd14
`define ST_AT_READ2		4'd15
// ALU operands
`define DST_R0			7'h00
`define DST_R15			7'h0F
`define DST_A			7'h50
`define DST_B			7'h51
`define DST_C			7'h52
`define DST_M1			7'h53
`define DST_M2			7'h54
`define DST_Y           7'h55	// stack registers
`define DST_Z           7'h56
`define DST_T           7'h57
`define DST_DATA        7'h58   // access to data registers using DA as address
`define DST_0           7'h59
`define DST_9           7'h5a
`define DST_DA          7'h5b	// data address register
`define DST_F           7'h5c   // f register
`define DST_CNT			7'h5d
`define DST_KEY         7'h5e
`define DST_PC          7'h5f   // Used for trace

`define OP1_R0			7'h00
`define OP1_R15			7'h0F
`define OP1_A			7'h50
`define OP1_B			7'h51
`define OP1_C			7'h52
`define OP1_M1			7'h53
`define OP1_M2			7'h54
`define OP1_Y           7'h55	// stack registers
`define OP1_Z           7'h56
`define OP1_T           7'h57
`define OP1_DATA        7'h58   // access to data registers using DA as address
`define OP1_0           7'h59
`define OP1_9           7'h5a
`define OP1_DA          7'h5b	// data address register
`define OP1_F           7'h5c   // f register
`define OP1_CNT			7'h5d
`define OP1_KEY         7'h5e
`define OP1_PC          7'h5f   // Used for trace


// P opcodes
`define P_NONE			3'd0
`define P_INC_P			3'd1
`define P_DEC_P			3'd2
`define P_CMP_EQ		3'd3
`define P_CMP_NEQ		3'd4
`define P_LOAD          3'd5
// operation on the carry
`define	C_NONE			2'b00
`define	C_SET			2'b01
`define	C_CLEAR			2'b10
`define	C_USERES		2'b11
