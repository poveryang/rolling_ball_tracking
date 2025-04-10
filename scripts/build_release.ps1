# 启动VS2022开发者PowerShell环境
$vsPath = "C:\Program Files\Microsoft Visual Studio\2022\Professional"
$vsDevShell = "$vsPath\Common7\Tools\Launch-VsDevShell.ps1"

if (Test-Path $vsDevShell) {
    Write-Host "正在初始化VS2022开发环境..."
    & $vsDevShell -Arch amd64 -HostArch amd64
} else {
    Write-Host "错误：找不到VS2022开发者PowerShell脚本"
    exit 1
}

# 切换到项目根目录（脚本所在目录的上一级）
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $projectRoot

# 创建并切换到构建目录
Write-Host "准备构建目录..."
if (!(Test-Path "build")) {
    New-Item -ItemType Directory -Path "build"
} else {
    Remove-Item -Recurse -Force "build\*"
}
Set-Location "build"

# 配置项目
Write-Host "配置项目..."
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

# 构建项目
Write-Host "构建Release版本..."
cmake --build . --config Release

# 安装项目
Write-Host "安装项目..."
cmake --install . --config Release

Write-Host "Release构建完成！"
Write-Host "生成的文件在 dist\release 目录下" 