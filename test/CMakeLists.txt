# 设置Google Test路径
set(GTEST_DEBUG_ROOT "C:/Users/yjunj/gtest/debug")
set(GTEST_RELEASE_ROOT "C:/Users/yjunj/gtest/release")

# 定义测试可执行文件列表
set(TEST_EXECUTABLES
    camera_control_test
    ball_detection_test
    ball_tracking_test
)

# 为每个测试创建可执行文件
foreach(TEST_NAME ${TEST_EXECUTABLES})
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    
    # 设置包含目录
    target_include_directories(${TEST_NAME} PRIVATE 
        ${CMAKE_SOURCE_DIR}/include
        $<$<CONFIG:Debug>:${GTEST_DEBUG_ROOT}/include>
        $<$<CONFIG:Release>:${GTEST_RELEASE_ROOT}/include>
    )
    
    # 链接库
    target_link_libraries(${TEST_NAME} PRIVATE
        $<$<CONFIG:Debug>:${GTEST_DEBUG_ROOT}/lib/gtest.lib>
        $<$<CONFIG:Release>:${GTEST_RELEASE_ROOT}/lib/gtest.lib>
        $<$<CONFIG:Debug>:${GTEST_DEBUG_ROOT}/lib/gtest_main.lib>
        $<$<CONFIG:Release>:${GTEST_RELEASE_ROOT}/lib/gtest_main.lib>
        ${OpenCV_LIBS}
        ball_tracker
    )

    # 设置运行时库
    set_target_properties(${TEST_NAME} PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    )

    # 安装测试可执行文件
    install(TARGETS ${TEST_NAME}
        RUNTIME DESTINATION bin
    )
endforeach()

# 创建测试数据目录
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_data)

# 复制测试数据
file(COPY 
    ${CMAKE_CURRENT_SOURCE_DIR}/test_video.MOV
    ${CMAKE_SOURCE_DIR}/config/balls_config.json
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test_data
)

# 安装测试数据
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_data
    DESTINATION test
)

# 添加测试
add_test(NAME ball_tracking_test COMMAND ball_tracking_test)