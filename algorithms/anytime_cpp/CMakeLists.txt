cmake_minimum_required(VERSION 3.5)
project(anytime_cpp)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -c -O3 -march=native -mtune=intel -std=gnu++17 -mavx2 -mfma -flto -fopenmp -fPIC -fno-math-errno -I/usr/local/include/eigen3/)
  # add_compile_options(-Wall -Wextra -Wpedantic -c -MMD -pipe)
endif ()

# find dependencies
find_package(Eigen3 REQUIRED)

# general demos

add_executable(hello_world
		src/hello_world.cpp)
target_link_libraries(hello_world Eigen3::Eigen)

add_executable(high_dim_system
		src/high_dim_system.cpp
		src/csv_utils.cpp)
target_link_libraries(high_dim_system Eigen3::Eigen)

add_executable(low_variance_resampler
		src/low_variance_resampler.cpp
		src/csv_utils.cpp)
target_link_libraries(low_variance_resampler Eigen3::Eigen)

# ICP

add_executable(functor_test
		src/icp/functor_test.cpp
		src/csv_utils.cpp)
target_link_libraries(functor_test Eigen3::Eigen)
target_include_directories(functor_test PRIVATE lib/LBFGSpp/include)

add_executable(icp_2d_test
		src/icp/icp_2d_test.cpp
		src/csv_utils.cpp)
target_link_libraries(icp_2d_test Eigen3::Eigen)
target_include_directories(icp_2d_test PRIVATE lib/LBFGSpp/include)

add_executable(knn_test
		src/icp/knn_test.cpp
		src/csv_utils.cpp)
target_link_libraries(knn_test Eigen3::Eigen)
target_include_directories(knn_test PRIVATE lib/LBFGSpp/include)

add_executable(solver_test
		src/icp/solver_test.cpp)
target_link_libraries(solver_test Eigen3::Eigen)
target_include_directories(solver_test PRIVATE lib/LBFGSpp/include)

# add_executable(range_libc_test
# 		range_libc_test.cpp)
# target_link_libraries(range_libc_test Eigen3::Eigen)
# TODO: rangelib

# include_directories(include)
