@echo off
chcp 65001 >nul
cd /d "%~dp0"
python tools\adc_gui.pyw
if errorlevel 1 pause
