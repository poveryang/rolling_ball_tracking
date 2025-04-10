cmake_minimum_required(VERSION 3.10)
project(RollingBallTracker VERSION 1.0.0)

# 设置项目版本信息
set(BALL_TRACKER_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(BALL_TRACKER_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(BALL_TRACKER_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置基础安装路径
set(BASE_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dist")

# Windows 平台特定配置
if(WIN32)
    # 设置运行时库链接方式
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    
    # 启用导出宏的生成
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# 查找OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 设置 OpenCV 路径
if(WIN32)
    # Windows 平台下的 OpenCV 路径
    set(OpenCV_DIR "C:/opencv/build")
elseif(APPLE)
    # macOS 平台下的 OpenCV 路径（如果使用 Homebrew 安装）
    set(OpenCV_DIR "/opt/homebrew/share/cmake/opencv4")
endif()

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV first.")
endif()

# 查找 nlohmann_json 包
find_package(nlohmann_json REQUIRED)
if(NOT nlohmann_json_FOUND)
    message(FATAL_ERROR "nlohmann_json not found. Please install nlohmann_json first.")
endif()

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
)

# 设置输出目录
if(WIN32)
    set_target_properties(ball_tracker PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    )
else()
    set_target_properties(ball_tracker PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    )
endif()

# 安装规则
install(TARGETS ball_tracker
    EXPORT ball_tracker-targets
    RUNTIME DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/bin
    LIBRARY DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/lib
    ARCHIVE DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/lib
    INCLUDES DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/include
)

# 安装头文件
install(FILES 
    include/ball_tracker_interface.h
    include/ball_tracker_common.h
    DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/include
)

# 创建并安装导出文件
install(EXPORT ball_tracker-targets
    FILE ball_tracker-targets.cmake
    NAMESPACE ball_tracker::
    DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/lib/cmake/ball_tracker
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
    INSTALL_DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/lib/cmake/ball_tracker
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ball_tracker-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ball_tracker-config-version.cmake"
    DESTINATION ${BASE_INSTALL_DIR}/$<LOWER_CASE:$<CONFIG>>/lib/cmake/ball_tracker
)
