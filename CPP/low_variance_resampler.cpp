#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <eigen3/Eigen/Core>

using namespace std;
using namespace Eigen;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

class CSVData
{
    public:
    MatrixXf data;
    string filename;

    CSVData(string filename_, MatrixXf data_)
    {
        filename = filename_;
        data = data_;
    }

    void writeToCSVfile()
    {
        ofstream file(filename.c_str());
        file << data.format(CSVFormat);
        file.close();
    }

    MatrixXf readFromCSVfile()
    {
        vector<float> matrixEntries;
        ifstream matrixDataFile(filename);
        string matrixRowString;
        string matrixEntry;
        int matrixRowNumber = 0;
    
        while (getline(matrixDataFile, matrixRowString))
        {
            stringstream matrixRowStringStream(matrixRowString);
            while (getline(matrixRowStringStream, matrixEntry, ','))
            {
                matrixEntries.push_back(stod(matrixEntry));
            }
            matrixRowNumber++;
        }
        
        return Map<Matrix<float, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
    }
};

VectorXi testRandomVector(VectorXf w)
{

    VectorXi w_rest = (w / w.minCoeff()).cast<int>();
    vector<int> weights(w_rest.data(), w_rest.data() + w_rest.size());
    int num_samples = w.size();
    static std::mt19937 generator{ std::random_device{}() };
    static std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    VectorXi idx = (VectorXi{ num_samples }.unaryExpr([&](auto x) { return distribution(generator); }));
    return idx;
}

int main()
{
    VectorXf v1 = VectorXf::Zero(4);
    v1 << 2, 4, 3, 1;
    VectorXf v_normed = v1 / v1.sum();
    MatrixXf M = MatrixXf::Zero(4, 5000);
    for (int i = 0; i < 5000; i++)
    {
        M.col(i) = testRandomVector(v_normed).cast<float>();
    }
    v1 = VectorXf::LinSpaced(4, 0, 1);
    MatrixXf V = MatrixXf::Zero(4, 4);
    V << v1, v1*2, v1*3, v1*4;
    // cout << V.col(3) << endl;
    cout << V(M.col(0), ArrayXf::LinSpaced(4, 0, 3)) << endl;
    cout << V(M.col(1), ArrayXf::LinSpaced(4, 0, 3)) << endl;
    cout << V(M.col(2), ArrayXf::LinSpaced(4, 0, 3)) << endl;
    // cout << v1(M.col(0)) << endl;
    // cout << v1(M.col(1)) << endl;
    // cout << idx << endl;
    // CSVData csv("D:/Documents/Summer_22/UPenn_Research/CPP/lowVariance.csv", M);
    // csv.writeToCSVfile();
}