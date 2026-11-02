@echo off
cd /d "%~dp0.."
python sim\run_modelsim.py %*
if errorlevel 1 pause
