#!/bin/bash

# 设置错误时退出
set -e

echo "开始构建项目..."

# 清理历史构建
echo "清理历史构建..."
rm -rf build/*

# 创建构建目录
mkdir -p build
cd build

# 配置 CMake
echo "配置 CMake..."
cmake ..

# 编译
echo "开始编译..."
make -j$(sysctl -n hw.ncpu)

echo "构建完成！"
echo "输出文件位置："
echo "  库文件：$(pwd)/lib/libball_tracker.dylib" 
echo "  测试可执行文件：$(pwd)/test/camera_control_test"
echo "  测试可执行文件：$(pwd)/test/ball_detection_test"