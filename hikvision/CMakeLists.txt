cmake_minimum_required(VERSION 3.4)
project(hikvision)

aux_source_directory(./inlcude HIK_INCLUDE)
aux_source_directory(./src HIK_SRC)
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUE_DIRS})

add_library(hikvision STATIC ${HIK_INCLUDE} ${HIK_SRC})

target_link_libraries(hikvision ${OpenCV_LIBS})
target_link_libraries(hikvision /opt/MvCamCtrlSDK/lib/64/libMvCameraControl.so)