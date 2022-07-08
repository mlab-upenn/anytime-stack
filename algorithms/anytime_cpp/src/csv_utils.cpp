#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include "csv_utils.hpp"

const Eigen::IOFormat CSVFormat(
	Eigen::StreamPrecision,
	Eigen::DontAlignCols,
	", ",
	"\n"
);

CSVData::CSVData(string filename_, MatrixXf data_) {
	filename = move(filename_);
	data = move(data_);
}

void CSVData::writeToCSVFile() const {
	ofstream file(filename.c_str());
	file << data.format(CSVFormat);
	file.close();
}

MatrixXf CSVData::readFromCSVFile() const {
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
