cmake_minimum_required(VERSION 3.10.0)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(radar-localization)

find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen REQUIRED)
find_package(FMT REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/test
)

set(SRC_LIST
    ${PROJECT_SOURCE_DIR}/src/features.cpp
    ${PROJECT_SOURCE_DIR}/src/registration.cpp
    ${PROJECT_SOURCE_DIR}/src/radar_utils.cpp
    ${PROJECT_SOURCE_DIR}/src/filter.cpp
)

add_executable(MAIN main.cpp ${SRC_LIST} )

target_link_libraries(MAIN
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    fmt::fmt
)