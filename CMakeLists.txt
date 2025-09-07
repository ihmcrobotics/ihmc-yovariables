cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 20)

project(yo-variables CXX)

set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include/yo-variables" CACHE PATH "Installation directory for headers")
set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")

#find_package(Catch2 2 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)