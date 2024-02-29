cmake_minimum_required(VERSION 3.10.0)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(radar-localization)

find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(
    ${GTSAM_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/test
)

set(SRC_LIST
    ${PROJECT_SOURCE_DIR}/src/radar_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/normal_feature.cpp
    ${PROJECT_SOURCE_DIR}/src/ceres_registration.cpp
    ${PROJECT_SOURCE_DIR}/src/radar_utils.cpp
    ${PROJECT_SOURCE_DIR}/src/filter.cpp
)

# set(SRC_LIST
#     ${PROJECT_SOURCE_DIR}/src/features.cpp
#     ${PROJECT_SOURCE_DIR}/src/registration.cpp
#     ${PROJECT_SOURCE_DIR}/src/radar_utils.cpp
#     ${PROJECT_SOURCE_DIR}/src/filter.cpp
# )
# set(SRC_LIST
#     ${PROJECT_SOURCE_DIR}/src/radar_sensor.cpp
#     ${PROJECT_SOURCE_DIR}/src/imu_sensor.cpp
#     ${PROJECT_SOURCE_DIR}/src/gps_sensor.cpp
#     ${PROJECT_SOURCE_DIR}/src/odometry_fusion.cpp
#     ${PROJECT_SOURCE_DIR}/src/radar_utils.cpp
# )

# add_executable(test test/test_odometry.cpp ${SRC_LIST} )
add_executable(test test/test_radar.cpp ${SRC_LIST})

target_link_libraries(test
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    gtsam
)