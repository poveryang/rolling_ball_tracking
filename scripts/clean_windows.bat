@echo off
echo 清理构建文件...

:: 删除构建目录
if exist build rmdir /s /q build

echo 清理完成！ 