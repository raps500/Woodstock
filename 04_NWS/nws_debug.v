module NWS(
    output wire			disp_cs_n_o,
	output wire			disp_addr_o,
	output wire			disp_sck_o,
	output wire			disp_data_o,
	output wire			disp_res_n_o,
	
	inout wire			col0l_o,
	inout wire			col1l_o,
	inout wire 			col2lt_o, // only top key
	inout wire			col2l_o,  // three key below, power together
	inout wire			col3l_o,
	inout wire			col4l_o,
	inout wire			col0r_o,
	inout wire			col1r_o,
	inout wire			col2r_o,
	inout wire			col3r_o,
	inout wire			col4r_o,
	
	input wire [3:0]	rowsl_in,
	input wire [3:0]	rowsr_in,
	output wire			txd_o,
	
	output wire [23:0]  debug_o

);

defparam ws.MXO2 = 1;
defparam ws.MAX10 = 0;
defparam ws.HP67 = 1;
defparam ws.HW_TRACE = 1;
defparam ws.HW_TRACE_MINI = 0;
defparam ws.DISP_DOGM132 = 1;
defparam ws.DISP_SSD1603 = 0;   



ws_hp67_xo2_7k  ws(
	.clk_in(),     // only used in Simulation or Max 10 variant
    .disp_cs_n_o(disp_cs_n_o),
	.disp_addr_o(disp_addr_o),
	.disp_sck_o(disp_sck_o),
	.disp_data_o(disp_data_o),
	.disp_res_n_o(disp_res_n_o),
	
	.col0l_o(col0l_o),
	.col1l_o(col1l_o),
	.col2lt_o(col2lt_o), // only top key
	.col2l_o(col2l_o),  // three key below, power together
	.col3l_o(col3l_o),
	.col4l_o(col4l_o),
	.col0r_o(col0r_o),
	.col1r_o(col1r_o),
	.col2r_o(col2r_o),
	.col3r_o(col3r_o),
	.col4r_o(col4r_o),
	
	.rowsl_in(rowsl_in),
	.rowsr_in(rowsr_in),
	.txd_o(txd_o),	
	.debug_o(debug_o)
	);
endmodule
