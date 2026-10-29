@echo off
cd /d "%~dp0"
set "TASK_PANEL_PY=C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%TASK_PANEL_PY%" (
  "%TASK_PANEL_PY%" control_panel_server.py --open-browser
) else (
  py -3 control_panel_server.py --open-browser
)
pause
