cmake_minimum_required(VERSION 3.5)
project(anytime_cpp)

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

# find dependencies

find_package(Eigen3 3.4.0 REQUIRED)
find_package(lbfgspp 0.2.0 REQUIRED)
message("-- Eigen3 version: " ${EIGEN3_VERSION_STRING})

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
target_link_libraries(functor_test lbfgspp)

add_executable(icp_2d_test
		src/icp/icp_2d_test.cpp
		src/csv_utils.cpp)
target_link_libraries(icp_2d_test Eigen3::Eigen)
target_link_libraries(icp_2d_test lbfgspp)

add_executable(knn_test
		src/icp/knn_test.cpp
		src/csv_utils.cpp)
target_link_libraries(knn_test Eigen3::Eigen)
target_link_libraries(knn_test lbfgspp)

add_executable(solver_test
		src/icp/solver_test.cpp)
target_link_libraries(solver_test Eigen3::Eigen)
target_link_libraries(solver_test lbfgspp)

# range_libc_test
if (WITH_CUDA)

	find_package(CUDA REQUIRED)

	cuda_add_executable(range_libc_test
			src/range_libc_test.cpp
			libs/src/range_lib/includes/kernels.cu
			libs/src/range_lib/vendor/lodepng/lodepng.cpp)

	target_compile_definitions(range_libc_test PRIVATE USE_CUDA=1)
	target_compile_definitions(range_libc_test PRIVATE CHUNK_SIZE=${CHUNK_SIZE})
	target_compile_definitions(range_libc_test PRIVATE NUM_THREADS=${CHUNK_THREADS})

else ()

	add_executable(range_libc_test
			src/range_libc_test.cpp
			libs/src/range_lib/includes/kernels.cu
			libs/src/range_lib/vendor/lodepng/lodepng.cpp)

	target_compile_definitions(range_libc_test PRIVATE USE_CUDA=0)

endif ()
target_include_directories(range_libc_test PRIVATE libs/src/range_lib/includes)
target_include_directories(range_libc_test PRIVATE libs/src/range_lib)
target_link_libraries(range_libc_test Eigen3::Eigen)
