cmake_minimum_required(VERSION 3.5)
project(particle_filter_cpp)

# Default to C99
if (NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# TODO: consider enabling some of the flags
	# add_compile_options(-Wall -Wextra -Wpedantic -c -O3 -march=native -mtune=intel -mavx2 -mfma -flto -fopenmp -fPIC -fno-math-errno)
	# add_compile_options(-Wall -Wextra -Wpedantic -c -MMD -pipe)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(message_filters REQUIRED)

if (BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif ()

include_directories(include)

add_executable(particle_filter src/particle_filter.cpp)
ament_target_dependencies(particle_filter rclcpp std_msgs sensor_msgs nav_msgs Eigen3)
target_include_directories(particle_filter PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>)

install(TARGETS particle_filter
		DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
		launch
		DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
		config
		DESTINATION share/${PROJECT_NAME})

ament_package()
