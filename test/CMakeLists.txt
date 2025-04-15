# 查找Google Test包
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)

# 设置 GTest 包含路径
if(APPLE)
    include_directories(/opt/homebrew/Cellar/googletest/1.16.0/include)
    link_directories(/opt/homebrew/Cellar/googletest/1.16.0/lib)
endif()

# 添加测试可执行文件
add_executable(camera_control_test
    camera_control_test.cpp
)

add_executable(ball_detection_test
    ball_detection_test.cpp
)

add_executable(ball_tracking_test
    ball_tracking_test.cpp
)

# 设置包含目录
target_include_directories(camera_control_test PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${GTEST_INCLUDE_DIRS}
)
target_include_directories(ball_detection_test PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${GTEST_INCLUDE_DIRS}
)
target_include_directories(ball_tracking_test PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${GTEST_INCLUDE_DIRS}
)

# 链接测试所需的库
target_link_libraries(camera_control_test
    PRIVATE
    gtest
    gtest_main
    ${OpenCV_LIBS}
    ball_tracker
)

target_link_libraries(ball_detection_test
    PRIVATE
    gtest
    gtest_main
    ${OpenCV_LIBS}
    ball_tracker
)

target_link_libraries(ball_tracking_test
    PRIVATE
    gtest
    gtest_main
    ${OpenCV_LIBS}
    ball_tracker
)

# 确保输出目录存在
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_data)

# 复制测试数据
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_video.MOV
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test_data)

# 复制配置文件
file(COPY ${CMAKE_SOURCE_DIR}/config/balls_config.json
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test_data)

# 添加测试
add_test(NAME ball_tracking_test COMMAND ball_tracking_test) 