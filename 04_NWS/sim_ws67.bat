@echo off
set path=c:\iverilog\bin;%PATH%
iverilog -o tb_ws.out -D SIMULATOR=1 tb_ws.v nws_hp67_xo2_7k.v ws_hp67_disasm.v dogm132.v
if errorlevel == 1 goto error
vvp tb_ws.out
:error