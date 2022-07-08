#ifndef CSV_UTILS_HPP_
#define CSV_UTILS_HPP_

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

extern const Eigen::IOFormat CSVFormat;

class CSVData {

public:

	string filename;
	MatrixXf data;

	CSVData(string filename_, MatrixXf data_);

	void writeToCSVFile() const;

	MatrixXf readFromCSVFile() const;

};

#endif // CSV_UTILS_HPP_
