project(BehaviorPlanner)

cmake_minimum_required (VERSION 2.8.3)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package( OpenCV REQUIRED )

include_directories(
    include/
    ${GTEST_INCLUDE_DIRS}
)

set(sources 
	src/main.cpp
	src/road.cpp
	src/vehicle.cpp
)

add_executable(BehaviorPlanner
    ${sources}
)

target_link_libraries(BehaviorPlanner
    ${OpenCV_LIBS}
)
