#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <functional>
#include "knn.h"
// #include <Eigen/CXX11/Tensor>
#include <unsupported/Eigen/CXX11/Tensor>
#include <LBFGS.h>
#include "../csv_utils.hpp"

#define PI 3.1415926535

using namespace Eigen;
using namespace std;

typedef knncpp::Matrixi Matrixi;

class PointCloudError {
private:
	MatrixXf p, q;

	MatrixXf pose2tf(VectorXf x) {
		VectorXf t = x(seq(0, 1), 0);
		MatrixXf R = MatrixXf::Zero(2, 2);
		R << cos(x(2)), -sin(x(2)), sin(x(2)), cos(x(2));
		MatrixXf T = MatrixXf::Identity(3, 3);
		T.block(0, 0, 2, 2) = R;
		T.block(0, 2, 2, 1) = t;

		return T;
	}

	MatrixXf transformPointCloud(MatrixXf ptcloud, MatrixXf T) {
		VectorXf t = T.block(0, 2, 2, 1);
		MatrixXf R = T.block(0, 0, 2, 2);
		MatrixXf ptcloud_tf = R * ptcloud;
		ptcloud_tf.colwise() += t;

		return ptcloud_tf;
	}

public:
	PointCloudError(MatrixXf p_, MatrixXf q_) : p(p_), q(q_) {}

	float operator()(const VectorXf &x, VectorXf &grad) {
		MatrixXf T = pose2tf(x);
		MatrixXf p_tf = transformPointCloud(p, T);
		MatrixXf d = p_tf - q;
		MatrixXf p_rot = T.block(0, 0, 2, 2) * p;
		float err = d.squaredNorm();
		grad(0) = 2 * d(0, all).sum();
		grad(1) = 2 * d(1, all).sum();
		grad(2) = 2 * (d(0, all).array() * p_rot(0, all).array() + d(1, all).array() * p_rot(1, all).array()).sum();
		return err;
	}
};

MatrixXf pose2tf(VectorXf x) {
	VectorXf t = x(seq(0, 1), 0);
	MatrixXf R = MatrixXf::Zero(2, 2);
	R << cos(x(2)), -sin(x(2)), sin(x(2)), cos(x(2));
	MatrixXf T = MatrixXf::Identity(3, 3);
	T.block(0, 0, 2, 2) = R;
	T.block(0, 2, 2, 1) = t;

	return T;
}

MatrixXf transformPointCloud(MatrixXf ptcloud, MatrixXf T) {
	VectorXf t = T.block(0, 2, 2, 1);
	MatrixXf R = T.block(0, 0, 2, 2);
	MatrixXf ptcloud_tf = R * ptcloud;
	ptcloud_tf.colwise() += t;

	return ptcloud_tf;
}

int main() {
	CSVData csv("../data/scanData.csv", MatrixXf::Zero(1, 1));
	MatrixXf r = csv.readFromCSVFile().transpose();
	r = (r.array() < 50).select(r, 0.00);
	ArrayXf angle = ArrayXf::LinSpaced(1081, -3 * PI / 4, 3 * PI / 4);
	MatrixXf target = MatrixXf::Zero(2, 1081);
	MatrixXf source = MatrixXf::Zero(2, 1081);

	int target_idx = 0;
	int source_idx = 50;

	target << (r.col(target_idx).array() * cos(angle)).matrix().transpose(),
		(r.col(target_idx).array() * sin(angle)).matrix().transpose();

	source << (r.col(source_idx).array() * cos(angle)).matrix().transpose(),
		(r.col(source_idx).array() * sin(angle)).matrix().transpose();


	VectorXf x(3);
	x << 1, 2, PI / 4;

	MatrixXf T = pose2tf(x);

	MatrixXf target_tf = transformPointCloud(target, T);

	MatrixXf test = target_tf(0, all);

	cout << 0 << endl;

	CSVData sv("../data/testPointCloud.csv", target_tf);

	sv.writeToCSVFile();

	// cout << target_tf << endl;

	cout << T << endl;

	return 0;
}
