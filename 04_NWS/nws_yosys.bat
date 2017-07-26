@echo off
set path c:\02_Elektronik\yosys-win32-mxebin-0.7;%PATH%
c:\02_Elektronik\yosys-win32-mxebin-0.7\yosys -o nws.bliff -S nws_hp67_xo2_7k.v ws_hp67_disasm.v dogm132.v