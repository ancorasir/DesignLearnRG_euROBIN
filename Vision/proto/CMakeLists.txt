# Minimum CMake required
cmake_minimum_required(VERSION 3.14.4)

# Project
project(proto VERSION 0.0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# Find required protobuf package
find_package(Protobuf REQUIRED)


# Print version messages
if(Protobuf_FOUND)
    message(STATUS "Using Protocol Buffers ${Protobuf_VERSION}")
endif()

set(ProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
        CACHE INTERNAL "Path to generated protobuf files.")

# Get the filenames of all the proto files.
file(GLOB PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.proto)


set(MSG_SRC "")
# Generate header and source files for all the proto files
foreach(proto ${PROTO_FILES})
    # Get filename without extension
    get_filename_component(PROTO_NAME_WE ${proto} NAME_WE)
    get_filename_component(PROTO_NAME ${proto} NAME)
    message(STATUS "Generating Protobuf Code for ${PROTO_NAME}")
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto})
    LIST (APPEND MSG_SRC "${PROTO_SRCS}" "${PROTO_HDRS}")
endforeach()

message(STATUS "${MSG_SRC}")
add_library(proto SHARED ${MSG_SRC})
target_link_libraries(proto ${PROTOBUF_LIBRARIES})
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
