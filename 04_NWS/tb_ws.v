/*
 * Test bench for Woodstock top entity
 *
 * (c) Copyright 2014-2016 R.A. Paz S.
 * 
 * This software is provided "as-is" without any warranty
 * as described by the GPL v2
 *
 */
`timescale 1ns/1ns
 
module tb_ws();

reg clk;

reg [7:0] simkbd_keycode_in;
reg simkbd_activate_key_pending_in;
defparam woodstock.HW_TRACE = 1;

ws_hp67_xo2_7k woodstock(
	.clk_in(clk)
`ifdef SIMULATOR
    ,
    .simkbd_activate_key_pending_in(simkbd_activate_key_pending_in),
    .simkbd_keycode_in(simkbd_keycode_in)
`endif
	);

always 
	#240.38 clk = ~clk; // 2.08 MHz clock
	
initial
	begin
	$dumpvars;
`ifdef SIMULATOR
    $display("Using simulated keyboard input");
`endif
	clk = 0;
    simkbd_keycode_in = 0;
    simkbd_activate_key_pending_in = 0;
    #5000000
    simkbd_activate_key_pending_in = 1'b1;
    simkbd_keycode_in = 8'h38;// simulate P/R
	#500
    simkbd_activate_key_pending_in = 0;
	#5000000
    simkbd_activate_key_pending_in = 1'b1;
    simkbd_keycode_in = 8'h14;// simulate 8
	#500
    simkbd_activate_key_pending_in = 0;
	#7000000
    simkbd_activate_key_pending_in = 1'b1;
    simkbd_keycode_in = 8'h38;// P/R
	#500
    simkbd_activate_key_pending_in = 0;
	#7000000 //00
    simkbd_activate_key_pending_in = 1'b1;
    simkbd_keycode_in = 8'h3A;// B
	#500
    simkbd_activate_key_pending_in = 0;
	#12000000 //00
    
    $finish;
	end
	
	
endmodule
