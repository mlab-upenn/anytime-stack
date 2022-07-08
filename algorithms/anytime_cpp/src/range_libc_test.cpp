#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Core>
#include <RangeLib.h>
#include <gflags/gflags.h>

#define MAX_DISTANCE 500
#define THETA_DISC 108
#define MB (1024.0*1024.0)

// this trick is to avoid compiler sadness about the quotations for the BASEPATH define
#define Q(x) #x
#define QUOTE(x) Q(x)

// grid sample settings
#define GRID_STEP 10
#define GRID_RAYS 40
#define GRID_SAMPLES 1
#define RANDOM_SAMPLES 200000
// #define RANDOM_SAMPLES (CHUNK_SIZE*2)
// #define RANDOM_SAMPLES (CHUNK_SIZE*10)

using namespace ranges;
using namespace benchmark;
using namespace std;
using namespace Eigen;

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

VectorXi choice_idx(VectorXf w) {
	VectorXi w_rest = (w / w.minCoeff()).cast<int>();
	vector<int> weights(w_rest.data(), w_rest.data() + w_rest.size());
	int num_samples = w.size();
	static std::mt19937 generator{std::random_device{}()};
	static std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	VectorXi idx = (VectorXi{num_samples}.unaryExpr([&](auto x) { return distribution(generator); }));
	return idx;
}

int main() {
	OMap map = OMap(1, 1);
	map = OMap("/home/schmidd/Documents/CPP/maps/basement_hallways_5cm.png");
	int MAX_RANGE_PX = (int) (MAX_DISTANCE / 0.01);
	RayMarchingGPU rmgpu(map, MAX_RANGE_PX);
	return 0;
}
