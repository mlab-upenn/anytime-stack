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

int main() {

	cout << "This works so far" << endl;

	CSVData csv("../data/scanData.csv", MatrixXf::Zero(2, 2));
	MatrixXf r = csv.readFromCSVFile().transpose();
	ArrayXf angle = ArrayXf::LinSpaced(1081, -3 * PI / 4, 3 * PI / 4);
	// cout << r.col(0) << endl;
	MatrixXf target = MatrixXf::Zero(2, 1081);
	MatrixXf source = MatrixXf::Zero(2, 1081);

	int target_idx = 0;
	int source_idx = 50;

	target << (r.col(target_idx).array() * cos(angle)).matrix().transpose(),
		(r.col(target_idx).array() * sin(angle)).matrix().transpose();

	source << (r.col(source_idx).array() * cos(angle)).matrix().transpose(),
		(r.col(source_idx).array() * sin(angle)).matrix().transpose();

	// MatrixXf dataPoints(3, 9);
	// dataPoints << 1, 2, 3, 1, 2, 3, 1, 2, 3,
	//               2, 1, 0, 3, 2, 1, 0, 3, 4,
	//               3, 1, 3, 1, 3, 4, 4, 2, 1;

	cout << "This means that data has been loaded" << endl;

	knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> kdtree(target);

	kdtree.setBucketSize(10);

	kdtree.setSorted(true);

	kdtree.setTakeRoot(true);

	kdtree.setMaxDistance(0.01);

	kdtree.setThreads(6);

	kdtree.build();

	//cout << source(seq(0, 1), seq(0, 4)) << endl;

	// // MatrixXf queryPoints = source.col(0);
	// //MatrixXf queryPoints = source(seq(0, 1), seq(0, 100));
	MatrixXf queryPoints = source;
	// // queryPoints << 0, 1, 0;

	cout << "Some KNN setup stuff... " << endl;

	Matrixi indices;
	MatrixXf distances;

	kdtree.query(queryPoints, 1, indices, distances);

	Map<Array<Index, Dynamic, Dynamic>> idx_long(indices.data(), indices.cols(), indices.rows());

	ArrayXi idx = idx_long.cast<int>();

	MatrixXf dist = (target(seq(0, 1), idx) - source(seq(0, 1), all)).colwise().norm();

	// cout << distances - dist << endl;

	// std::cout
	//     << "Neighbor indices:" << std::endl
	//     << indices << std::endl
	//     << "Neighbor distances:" << std::endl
	//     << distances << std::endl;

	// cout << idx.rows() << endl;
	// MatrixXf target_processed = target(seq(0, 1), idx);
	// CSVData csv1("D:/Documents/Summer_22/UPenn_Research/ICP_CPP/idxData.csv", idx.matrix());
	// csv1.writeToCSVFile();
	// string filename = "D:/Documents/UPenn_Research/data.csv";
	// ofstream file(filename.c_str());
	// file << target_processed.format(CSVFormat);
	// file.close();

	return 0;
}
