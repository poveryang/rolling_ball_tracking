cmake_minimum_required(VERSION 3.30)
project(RollingBallTracker)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(ball_tracker SHARED src/ball_tracker_interface.cpp)
target_include_directories(ball_tracker PUBLIC include)
target_link_libraries(ball_tracker
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
)
