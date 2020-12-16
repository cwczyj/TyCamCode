cmake_minimum_required(VERSION 2.8)

project(TYCam)

find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

set(COMMON_SOURCES common/MatViewer.cpp common/TYThread.cpp)

add_library(sample_common STATIC ${COMMON_SOURCES})
target_link_libraries(sample_common pthread)

add_executable(TYCam tycamCapture.cpp main.cpp)
add_dependencies(TYCam sample_common)
target_link_libraries(TYCam sample_common -ltycam ${OpenCV_LIBS})