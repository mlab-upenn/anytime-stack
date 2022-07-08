#include <iostream>
#include <fstream>
#include <random>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <unsupported/Eigen/KroneckerProduct>
#include "csv_utils.hpp"

using namespace std;
using namespace Eigen;

#define PI 3.1415926535

class HighDimSystem {
public:
	int num_dims;
	MatrixXf A;
	MatrixXf B;

	HighDimSystem(const MatrixXf &A_, const MatrixXf &B_, int n_) {
		num_dims = n_;
		A = KroneckerProduct<MatrixXf, MatrixXf>(MatrixXf::Identity(num_dims, num_dims), A_);
		B = KroneckerProduct<MatrixXf, MatrixXf>(MatrixXf::Identity(num_dims, num_dims), B_);
	}

	VectorXf fs(const VectorXf &x) {
		return A * x;
	}

	MatrixXf gs(const VectorXf &x) {
		return B;
	}

	VectorXf f_sys(const VectorXf &x, const VectorXf &u) {
		return A * x + B * u;
	}
};

class Control {
public:
	VectorXf x_ref;
	MatrixXf K;

	Control() {
		x_ref = VectorXf::Zero(4);
		K = MatrixXf::Zero(2, 4);
		K << 1, 0, 1, 0,
			0, 1, 0, 1;
	}

	VectorXf ctrl(float t, const VectorXf &x) {
		x_ref << cos(2 * PI * t), sin(2 * PI * t), 0, 0;
		return K * (x_ref - x);
	}
};

class SolveODE {
public:
	float Ts;
	int N, Nh, num_dims;
	VectorXf x0, u0;
	MatrixXf A_, B_;

	SolveODE(float Ts_, int N_, VectorXf x0_, int num_dims_) {
		Ts = Ts_;
		N = N_;
		x0 = x0_;
		u0 = VectorXf::Zero(2);
		A_ = MatrixXf::Zero(4, 4);
		B_ = MatrixXf::Zero(4, 2);

		A_ << 0, 0, 1, 0,
			0, 0, 0, 1,
			0, 0, -1, 0,
			0, 0, 0, -1;

		B_ << 0, 0,
			0, 0,
			1, 0,
			0, 1;

		num_dims = num_dims_;
	}

	MatrixXf solve() {
		float t;
		HighDimSystem sys(A_, B_, num_dims);
		Control lqr;
		auto n = x0.rows();
		MatrixXf X = MatrixXf::Zero(n, N);
		MatrixXf K = MatrixXf::Zero(n, 4);
		VectorXf u(2);
		X.col(0) = x0;

		for (int k = 0; k < N - 1; k++) {
			t = k * Ts;
			u << 0, 0;
			K.col(0) = Ts * sys.f_sys(X.col(k), u);
			K.col(1) = Ts * sys.f_sys(X.col(k) + K.col(0) / 2, u);
			K.col(2) = Ts * sys.f_sys(X.col(k) + K.col(1) / 2, u);
			K.col(3) = Ts * sys.f_sys(X.col(k) + K.col(2), u);
			X.col(k + 1) = X.col(k) + (K.col(0) + 2 * K.col(1) + 2 * K.col(2) + K.col(3)) / 6;
		}

		return X;

	}

};

int main() {

	VectorXf x0(4);
	x0 << 1, 0, 0, 0;
	VectorXf u;
	Control lqr;
	u = lqr.ctrl(0, x0);
	MatrixXf A_ = MatrixXf::Zero(4, 4);
	MatrixXf B_ = MatrixXf::Zero(4, 2);

	A_ << 0, 0, 1, 0,
		0, 0, 0, 1,
		0, 0, -1, 0,
		0, 0, 0, -1;

	B_ << 0, 0,
		0, 0,
		1, 0,
		0, 1;
	HighDimSystem sys(A_, B_, 10);
	VectorXf x = sys.f_sys(x0, u);
	cout << x << endl;
	// SolveODE ode(1e-2, 10000, x0, 10);
	// MatrixXf X = ode.solve();
	// CSVData csv("/home/defcon1/Documents/Datasets/high_dim_state_data.csv", X);
	// csv.writeToCSVFile();

	return 0;
}
