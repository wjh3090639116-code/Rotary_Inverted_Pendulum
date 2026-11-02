@echo off
cd /d "%~dp0"
gw_sh build.tcl
if errorlevel 1 pause
