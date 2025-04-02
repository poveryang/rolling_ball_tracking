cmake_minimum_required(VERSION 3.10)
project(RollingBallTracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 设置 OpenCV 路径
if(WIN32)
    # Windows 平台下的 OpenCV 路径
    set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/windows_opencv/build")
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

# 设置包含目录
target_include_directories(ball_tracker 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
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
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
