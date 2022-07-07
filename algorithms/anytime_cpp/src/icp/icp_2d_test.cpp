#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <functional>
#include "knn.h"
#include <LBFGS.h>

#define PI 3.1415926535

using namespace Eigen;
using namespace std;

typedef knncpp::Matrixi Matrixi;

const static Eigen::IOFormat CSVFormat(
	Eigen::StreamPrecision,
	Eigen::DontAlignCols,
	", ",
	"\n"
);

class CSVData {
public:
	MatrixXf data;
	string filename;

	CSVData(string filename_, MatrixXf data_) {
		filename = filename_;
		data = data_;
	}

	void writeToCSVFile() {
		ofstream file(filename.c_str());
		file << data.format(CSVFormat);
		file.close();
	}

	MatrixXf readFromCSVFile() {
		vector<float> matrixEntries;
		ifstream matrixDataFile(filename);
		string matrixRowString;
		string matrixEntry;
		int matrixRowNumber = 0;

		while (getline(matrixDataFile, matrixRowString)) {
			stringstream matrixRowStringStream(matrixRowString);
			while (getline(matrixRowStringStream, matrixEntry, ',')) {
				matrixEntries.push_back(stod(matrixEntry));
			}
			matrixRowNumber++;
		}

		return Map<Matrix<float, Dynamic, Dynamic, RowMajor>>(
			matrixEntries.data(),
			matrixRowNumber,
			matrixEntries.size() / matrixRowNumber
		);
	}
};


class ICPLoss {
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
		int n_pts = ptcloud.cols();
		int n_dims = ptcloud.rows();
		MatrixXf ptcloud_tf = MatrixXf::Constant(n_dims + 1, n_pts, 1.00);
		ptcloud_tf(seq(0, 1), all) = ptcloud;
		ptcloud_tf = T * ptcloud_tf;

		return ptcloud_tf(seq(0, 1), all);
	}

public:

	ICPLoss(MatrixXf p_, MatrixXf q_) : p(p_), q(q_) {}

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
	int n_pts = ptcloud.cols();
	int n_dims = ptcloud.rows();
	MatrixXf ptcloud_tf = MatrixXf::Constant(n_dims + 1, n_pts, 1.00);
	ptcloud_tf(seq(0, 1), all) = ptcloud;
	ptcloud_tf = T * ptcloud_tf;

	return ptcloud_tf(seq(0, 1), all);
}

int main() {

	CSVData csv("../data/scanData.csv", MatrixXf::Zero(1, 1));
	MatrixXf r = csv.readFromCSVFile().transpose();
	r = (r.array() < 50).select(r, 0.00);
	ArrayXf angle = ArrayXf::LinSpaced(1081, -3 * PI / 4, 3 * PI / 4);
	MatrixXf target = MatrixXf::Zero(2, 1081);
	MatrixXf source = MatrixXf::Zero(2, 1081);
	MatrixXf pc = MatrixXf::Zero(2, 1081);

	std::map<int, MatrixXf> point_clouds;

	int num_clouds = r.cols();

	for (int k = 0; k < num_clouds; k++) {
		pc << (r.col(k).array() * cos(angle)).matrix().transpose(),
			(r.col(k).array() * sin(angle)).matrix().transpose();

		point_clouds.insert({k, pc});
	}


	int target_idx = 0;
	int source_idx = 2;

	target = point_clouds[target_idx];
	source = point_clouds[source_idx];

	knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> kdtree(target);

	kdtree.setBucketSize(10);

	kdtree.setSorted(true);

	kdtree.setTakeRoot(true);

	kdtree.setMaxDistance(0.10);

	kdtree.setThreads(6);

	kdtree.build();

	Matrixi indices;
	MatrixXf distances;

	kdtree.query(source, 1, indices, distances);

	Map<Array<Index, Dynamic, Dynamic>> idx_long(indices.data(), indices.cols(), indices.rows());

	ArrayXi idx = idx_long.cast<int>();

	MatrixXf target_knn = target(all, idx);

	LBFGSpp::LBFGSParam<float> param;

	param.epsilon = 1e-1;
	param.max_iterations = 2;

	LBFGSpp::LBFGSSolver<float> solver(param);

	ICPLoss loss(source, target_knn);


	VectorXf x = VectorXf::Zero(3);
	VectorXf grad = VectorXf::Zero(3);

	float l;


	cout << loss(x, grad) << endl;

	int niter = solver.minimize(loss, x, l);

	std::cout << niter << " iterations" << std::endl;
	std::cout << "x = \n" << x.transpose() << std::endl;
	std::cout << "f(x) = " << l << std::endl;


	// VectorXf x(3);
	// x << 1, 2, PI/4;

	// MatrixXf T = pose2tf(x);

	// MatrixXf target_tf = transformPointCloud(target, T);

	// MatrixXf test = target_tf(0, all);

	// cout << 0 << endl;

	// CSVData sv("D:/Documents/Summer_22/UPenn_Research/ICP_CPP/testPointCloud.csv", target_tf);

	// sv.writeToCSVFile();

	// // cout << target_tf << endl;

	// cout << T << endl;

	return 0;

}
