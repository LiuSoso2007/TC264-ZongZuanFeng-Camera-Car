@echo off
setlocal enabledelayedexpansion
set "sourceFile=.project"
set "target_line="
for /f "tokens=1* delims=:" %%a in ('findstr /n "^" "%sourceFile%"') do (
    if %%a equ 3 set "target_line=%%b"
)
for /f "tokens=3 delims=<>" %%c in ("%target_line%") do set "old_str=%%c"
echo 当前工程名称: %old_str%
set /p "userInput=请输入新的工程名称: "
set "tempFile=temp.txt"
set "oldString=%old_str%"
set "newString=%userInput%"
if exist "%tempFile%" del /Q "%tempFile%"
for /f "delims=" %%i in ('type "%sourceFile%"') do (
    set "line=%%i"
    set "modified=!line!"
    if defined modified (
        set "modified=!modified:%oldString%=%newString%!"
        echo !modified!>>"%tempFile%"
    ) else (
        echo %%i>>"%tempFile%"
    )
)
move /y "%tempFile%" "%sourceFile%" >nul 2>&1

set "sourceFile=.cproject"
if exist "%tempFile%" del /Q "%tempFile%"
for /f "delims=" %%i in ('type "%sourceFile%"') do (
    set "line=%%i"
    set "modified=!line!"
    if defined modified (
        set "modified=!modified:%oldString%=%newString%!"
        echo !modified!>>"%tempFile%"
    ) else (
        echo %%i>>"%tempFile%"
    )
)
move /y "%tempFile%" "%sourceFile%" >nul 2>&1
call 删除临时文件.bat
endlocal

