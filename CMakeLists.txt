cmake_minimum_required(VERSION 3.10)
project(LaneDistanceDetector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Project: LaneDistanceDetector")
message(STATUS "Root Path: ${CMAKE_SOURCE_DIR}")

set(OPENCV_DIR ${CMAKE_SOURCE_DIR}/third_party/opencv470)
message(STATUS "OPENCV_DIR Path: ${OPENCV_DIR}")

include_directories(${OPENCV_DIR}/include)
link_directories(${OPENCV_DIR}/lib)

set(FMT_DIR ${CMAKE_SOURCE_DIR}/third_party/fmt)
message(STATUS "FMT_DIR Path: ${FMT_DIR}")

include_directories(${FMT_DIR}/include)
link_directories(${FMT_DIR}/lib)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(OPENCV_LIB opencv_world470d.lib)
    set(FMT_LIB fmtd.lib)
else ()
    set(OPENCV_LIB opencv_world470.lib)
    set(FMT_LIB fmt.lib)
endif ()


set(JSON_DIR ${CMAKE_SOURCE_DIR}/third_party/json373)
message(STATUS "JSON_DIR Path: ${JSON_DIR}")

include_directories(${JSON_DIR}/include)

# compile own file
file(GLOB SRC_LIST
        ${CMAKE_SOURCE_DIR}/src/*.cpp
)
add_executable(LaneDistanceDetector ${SRC_LIST})

target_compile_features(LaneDistanceDetector PRIVATE cxx_std_17)

target_link_libraries(LaneDistanceDetector ${OPENCV_LIB} ${FMT_LIB})
