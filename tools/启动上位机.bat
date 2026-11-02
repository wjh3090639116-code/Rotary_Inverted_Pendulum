@echo off
chcp 65001 >nul
cd /d "%~dp0"
python adc_gui.pyw
if errorlevel 1 pause
