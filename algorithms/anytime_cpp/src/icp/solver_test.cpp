#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <functional>
#include <LBFGS.h>

#define PI 3.1415926535

using namespace Eigen;
using namespace std;


class Rosenbrock {
private:
	int n;
	float a, b, c;
public:
	Rosenbrock(int n_) : n(n_) {}

	float operator()(const VectorXf &x, VectorXf &grad) {
		float fx = 0.5 * ((x(0) - 2) * (x(0) - 2) + (x(1) - 4) * (x(1) - 4)) + 1;
		grad(0) = (x(0) - 2);
		grad(1) = (x(1) - 4);
		return fx;
	}
};

// class Function
// {
//     public:
//     float a, b, c;
//     Function (float a_, float b_, float c_)
//     {
//         a = a_;
//         b = b_;
//         c = c_;
//     }

//     float f(VectorXf x)
//     {
//         return 0.5 * ((x(0) - a)*(x(0) - a) + (x(1) - b)*(x(1) - b)) + c;
//     }
// };

// auto plus_one = [](const int value)
// {
//     return value + 1;
// };

float a, b, c;

float f(VectorXf x) {
	return 0.5 * ((x(0) - a) * (x(0) - a) + (x(1) - b) * (x(1) - b)) + c;
}

int main() {

	LBFGSpp::LBFGSParam<float> param;

	param.epsilon = 1e-6;
	param.max_iterations = 100;

	LBFGSpp::LBFGSSolver<float> solver(param);

	Rosenbrock fun(2);

	// Initial guess
	VectorXf x0 = VectorXf::Zero(2);
	// x will be overwritten to be the best point found
	float fx;
	int niter = solver.minimize(fun, x0, fx);

	std::cout << niter << " iterations" << std::endl;
	std::cout << "x = \n" << x0.transpose() << std::endl;
	std::cout << "f(x) = " << fx << std::endl;

	return 0;

}
