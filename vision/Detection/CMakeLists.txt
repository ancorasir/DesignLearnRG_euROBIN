find_package(OpenCV)
find_package(onnxruntime REQUIRED)


aux_source_directory(${CMAKE_CURRENT_LIST_DIR} PROJ_SRC)
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_library(Detection SHARED ${PROJ_SRC})

target_link_libraries(Detection  PUBLIC  ${OpenCV_LIBS}
                                   onnxruntime::onnxruntime)

target_include_directories(Detection PUBLIC ${OpenCV_INCLUDE_DIRS})
set_target_properties(Detection PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib
)
