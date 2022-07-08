#include <iostream>
#include <Eigen/Core>

int main() {
	std::cout << "Hello World!" << std::endl;
	std::cout
		<< "Using Eigen3 "
		<< EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION
		<< std::endl;
	return 0;
}
