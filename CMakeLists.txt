cmake_minimum_required(VERSION 3.21)
project(vgd_rm2023_vision_nn)


set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE RELEASE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
SET(BIN_NAME "run")


# 定义相机SDK工具包目录
set(HIKVISION_CAMERA_SDK_DIR "${PROJECT_SOURCE_DIR}/hikvision")
# OpenCV
find_package(OpenCV 4 REQUIRED core imgproc video dnn)

LINK_DIRECTORIES(/opt/MvCamCtrlSDK/lib/64)

include_directories(${OpenCV_DIRS})
find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)

# 指定相机库目录
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/hikvision/include)

FILE(GLOB_RECURSE INCLUDE_FILES_H "${PROJECT_SOURCE_DIR}/*.h")
FILE(GLOB_RECURSE INCLUDE_FILES_HPP "${PROJECT_SOURCE_DIR}/*.hpp")
FILE(GLOB_RECURSE SRC_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# add_subdirectory(hikvision/tool)
add_subdirectory(hikvision)

add_executable(${BIN_NAME} main.cpp
        ${INCLUDE_FILES_H}
        ${INCLUDE_FILES_HPP}
        ${SRC_FILES} src/inference.cpp)

target_link_libraries(${BIN_NAME} ${OpenCV_LIBS} openvino::runtime openvino::frontend::onnx)
# /opt/intel/openvino_2022.2.0.7713/runtime/lib/
TARGET_LINK_LIBRARIES(${BIN_NAME} hikvision)
