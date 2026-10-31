@echo off
title Flight Radar Serial - COM5
start "Flight Radar Serial" powershell.exe -NoExit -ExecutionPolicy Bypass -File "%~dp0float_serial.ps1"
