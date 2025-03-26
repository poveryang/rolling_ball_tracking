@echo off
setlocal enabledelayedexpansion

echo 开始构建项目...

:: 创建构建目录
if not exist build mkdir build
cd build

:: 配置 CMake
echo 配置 CMake...
cmake .. -G "Visual Studio 16 2019" -A x64

:: 编译
echo 开始编译...
cmake --build . --config Release

echo 构建完成！
echo 输出文件位置：
echo   可执行文件：%CD%\bin\Release\ball_tracker.dll
echo   库文件：%CD%\lib\Release\ball_tracker.lib

cd .. 