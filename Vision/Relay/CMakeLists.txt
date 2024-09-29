find_package(Eigen3)

find_package(realsense2 REQUIRED)

find_package(ur_rtde REQUIRED)

#--------   Include zmq dependence ----------------
find_package(cppzmq)
#--------Include OpenCV dependence --------------------
find_package(OpenCV)

#--------Include Depth dependence --------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(Depth_Lib REQUIRED IMPORTED_TARGET Depth)

include_directories(${PROJECT_SOURCE_DIR}/Detection)

add_executable(Relay main.cpp DataRelay.cpp)

target_link_libraries(Relay PRIVATE Detection
                                    PkgConfig::Depth_Lib
                                    realsense2::realsense2
                                    ur_rtde::rtde
                                    proto
                                    ${Protobuf_LIBRARIES}
                                    cppzmq
                                    Eigen3::Eigen
                                    ${OpenCV_LIBS}
                                )

target_include_directories(Relay PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(Relay PRIVATE RESOURCE_PATH_STR="${PROJECT_SOURCE_DIR}/resource/"
    RECORD_PATH_STR="${PROJECT_SOURCE_DIR}/record_ws/")
