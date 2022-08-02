cmake_minimum_required(VERSION 3.22)
project(icp)

option(WITH_CUDA "Compile CUDA ray cast variants" ON)

# from CMake 3.15 there is PREPEND
list(INSERT CMAKE_PREFIX_PATH 0 "${PROJECT_SOURCE_DIR}/libs/install")
message("-- CMAKE_PREFIX_PATH CMake variable = " ${CMAKE_PREFIX_PATH})
message("-- CMAKE_PREFIX_PATH env variable = " $ENV{CMAKE_PREFIX_PATH})

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    # TODO: Or do we need C++17?
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # TODO: consider enabling some of the flags
    # add_compile_options(-Wall -Wextra -Wpedantic -c -O3 -march=native -mtune=intel -mavx2 -mfma -flto -fopenmp -fPIC -fno-math-errno)
    # add_compile_options(-Wall -Wextra -Wpedantic -c -MMD -pipe)
endif ()

include_directories(.)

find_package(Eigen3 3.4.0 REQUIRED)
message("-- Eigen3 version: " ${EIGEN3_VERSION_STRING})

add_executable(icp_lvm
        icp_lvm_test.cpp)
target_link_libraries(icp_lvm Eigen3::Eigen)

#add_executable(icp
#        functor_test.cpp
#        icp.hpp
#        icp_2d.hpp
#        icp_2d_test.cpp
#        icp_lvm_test.cpp
#        knn.h
#        knn_test.cpp
#        solver_test.cpp)
