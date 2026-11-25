@echo off
REM STM32H7 编译脚本
REM 使用: build.bat [build|flash|clean]

cd /d "%~dp0.."

if "%~1"=="" (
    call :build
    goto :end
)

if "%~1"=="build" (
    call :build
    goto :end
)

if "%~1"=="flash" (
    call :flash
    goto :end
)

if "%~1"=="clean" (
    call :clean
    goto :end
)

call :help
goto :end

:build
echo === 编译项目 ===
py Tool\debug\stm32h7_tool.py build
goto :eof

:flash
echo === 编译+烧录 ===
py Tool\debug\stm32h7_tool.py flash
goto :eof

:clean
echo === 清理编译文件 ===
if exist "MDK-ARM\CtrBoard-H7_ALL\*.axf" del /q "MDK-ARM\CtrBoard-H7_ALL\*.axf"
if exist "MDK-ARM\CtrBoard-H7_ALL\*.hex" del /q "MDK-ARM\CtrBoard-H7_ALL\*.hex"
if exist "MDK-ARM\CtrBoard-H7_ALL\*.map" del /q "MDK-ARM\CtrBoard-H7_ALL\*.map"
echo 清理完成
goto :eof

:help
echo 用法: build.bat [build^|flash^|clean]
echo   build  - 编译项目
echo   flash  - 编译+烧录
echo   clean  - 清理编译文件
goto :eof

:end
