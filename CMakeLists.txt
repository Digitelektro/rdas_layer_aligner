cmake_minimum_required(VERSION 3.10)

project(rdas_aligner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


find_package(OpenCV)
find_package(OpenCL)
find_package(PkgConfig REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

add_executable(rdas_aligner
    main.cpp
    parameterparser.cpp
    tps.cpp
    configstorage.cpp
    imageproc.cpp
    opencl.cpp
)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rdas_aligner
    ${OpenCV_LIBS}
    jsoncpp_lib  
)

if(OpenCL_FOUND)
    target_compile_definitions(rdas_aligner PUBLIC OPENCL_FOUND=1)
    include_directories(${OpenCL_INCLUDE_DIRS})
    target_link_libraries(rdas_aligner ${OpenCL_LIBRARIES})
endif()