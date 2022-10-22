cmake_minimum_required(VERSION 2.8)

project(DisplayImage)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_CXX_FLAGS "-Wall")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(demo main.cpp)
add_subdirectory(modules)
add_subdirectory(common)

target_include_directories(
    demo
    PUBLIC ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    rmath
)

target_link_libraries(
    demo
    ${OpenCV_LIBS}
    Eigen3::Eigen
    #${Eigen3_LIBS}
    rmath
)
