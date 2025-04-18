cmake_minimum_required(VERSION 3.10)
project(RollingBallTracker VERSION 1.0.0)

# 清理之前的构建文件
if(EXISTS ${CMAKE_BINARY_DIR})
    file(REMOVE_RECURSE ${CMAKE_BINARY_DIR})
endif()

# 设置项目版本信息
set(BALL_TRACKER_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(BALL_TRACKER_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(BALL_TRACKER_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成编译命令数据库
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 启用测试
enable_testing()

# 设置安装前缀
if(NOT DEFINED INSTALL_DIR)
    set(INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dist")
endif()
set(CMAKE_INSTALL_PREFIX "${INSTALL_DIR}")

# Windows 平台特定配置
if(WIN32)
    # 设置运行时库链接方式
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    
    # 启用导出宏的生成
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    
    # 设置UTF-8编码
    add_compile_options(/utf-8)
else()
    message(FATAL_ERROR "This project only supports Windows platform")
endif()

# 查找OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 查找 OpenCV 包
set(OpenCV_DIR "C:/Program Files/opencv/build")
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV first.")
endif()

# 查找 nlohmann_json 包
set(nlohmann_json_DIR "C:/Users/yjunj/miniconda3/Library/share/cmake/nlohmann_json")
find_package(nlohmann_json REQUIRED)
if(NOT nlohmann_json_FOUND)
    message(FATAL_ERROR "nlohmann_json not found. Please install nlohmann_json first.")
endif()

# 添加华睿相机SDK
set(MV_CAM_SDK_DIR ${CMAKE_SOURCE_DIR}/third_party/mv_cam_sdk)
include_directories(${MV_CAM_SDK_DIR}/Include)
link_directories(${MV_CAM_SDK_DIR}/Lib/x64)

# 收集源文件
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

# 创建共享库
add_library(ball_tracker SHARED ${SRC_FILES})

# 设置版本信息
set_target_properties(ball_tracker PROPERTIES
    VERSION ${BALL_TRACKER_VERSION_MAJOR}.${BALL_TRACKER_VERSION_MINOR}.${BALL_TRACKER_VERSION_PATCH}
    SOVERSION ${BALL_TRACKER_VERSION_MAJOR}
    DEBUG_POSTFIX "d"  # Debug版本添加'd'后缀
)

# 设置包含目录
target_include_directories(ball_tracker 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(ball_tracker 
    PUBLIC 
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
        OpenMP::OpenMP_CXX
        MVSDKmd
)

# 设置输出目录
set_target_properties(ball_tracker PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# 添加huarui_grab和huarui_video
add_executable(huarui_grab test/huarui_grab.cpp)
target_link_libraries(huarui_grab
    ball_tracker
    ${OpenCV_LIBS}
    MVSDKmd
)

add_executable(huarui_video test/huarui_video.cpp)
target_link_libraries(huarui_video
    ball_tracker
    ${OpenCV_LIBS}
    MVSDKmd
)

# 安装目标
install(TARGETS ball_tracker huarui_grab huarui_video
    EXPORT ball_tracker-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# 安装头文件
install(FILES 
    include/ball_tracker_interface.h
    include/ball_tracker_common.h
    include/camera_control.h
    DESTINATION include
)

# 安装配置文件
install(DIRECTORY
    ${CMAKE_SOURCE_DIR}/config/
    DESTINATION config
)

# 创建并安装导出文件
install(EXPORT ball_tracker-targets
    FILE ball_tracker-targets.cmake
    NAMESPACE ball_tracker::
    DESTINATION lib/cmake/ball_tracker
)

# 创建并安装配置文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ball_tracker-config-version.cmake"
    VERSION ${BALL_TRACKER_VERSION_MAJOR}.${BALL_TRACKER_VERSION_MINOR}.${BALL_TRACKER_VERSION_PATCH}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ball_tracker-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ball_tracker-config.cmake"
    INSTALL_DESTINATION lib/cmake/ball_tracker
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ball_tracker-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ball_tracker-config-version.cmake"
    DESTINATION lib/cmake/ball_tracker
)

# 添加测试子目录
add_subdirectory(test)
