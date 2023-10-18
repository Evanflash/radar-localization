cmake_minimum_required(VERSION 3.10.0)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(radar-localization)

find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/filter
    ${PROJECT_SOURCE_DIR}/include/raw_data
    ${PROJECT_SOURCE_DIR}/include/display
    ${PROJECT_SOURCE_DIR}/include/utils
    ${PROJECT_SOURCE_DIR}/include/registration
)

set(SRC_FILTER
    ${PROJECT_SOURCE_DIR}/src/filter/base_filter.cpp
    ${PROJECT_SOURCE_DIR}/src/filter/k_strongest_filter.cpp
    ${PROJECT_SOURCE_DIR}/src/filter/cfar_filter.cpp
)

set(SRC_REGISTRATION
    ${PROJECT_SOURCE_DIR}/src/registration/base_registration.cpp
    ${PROJECT_SOURCE_DIR}/src/registration/icp_registration.cpp
    ${PROJECT_SOURCE_DIR}/src/registration/ndt_registration.cpp
    ${PROJECT_SOURCE_DIR}/src/registration/registration.cpp
)

set(SRC_LIST
    
    ${PROJECT_SOURCE_DIR}/src/raw_data/radar_data.cpp
    ${PROJECT_SOURCE_DIR}/src/display/show_point_cloud.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/utils_func.cpp
)

add_executable(MAIN main.cpp ${SRC_LIST} ${SRC_FILTER} ${SRC_REGISTRATION})

target_link_libraries(MAIN
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)