#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <chrono>
#include <random>
#include <functional>
#include "knn.h"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#define PI 3.1415926535

using namespace Eigen;
using namespace std;

typedef knncpp::Matrixi Matrixi;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

// Generic functor template
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum
    {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const
    {
        return m_inputs;
    }

    int values() const
    {
        return m_values;
    }

};

// Read and write from CSV file (emulates ROS laserscan behavior)
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

// Utilities
MatrixXf pose2tf(VectorXf x)
{
    VectorXf t = x(seq(0, 1), 0);

    MatrixXf R = MatrixXf::Zero(2, 2);

    R << cos(x(2)), -sin(x(2)),
            sin(x(2)), cos(x(2));

    MatrixXf T = MatrixXf::Identity(3, 3);

    T.block(0, 0, 2, 2) = R;
    T.block(0, 2, 2, 1) = t;

    return T;
}

MatrixXf transformPointCloud(MatrixXf ptcloud, MatrixXf T)
{
    int n_pts = ptcloud.cols();

    int n_dims = ptcloud.rows();

    MatrixXf ptcloud_tf = MatrixXf::Constant(n_dims + 1, n_pts, 1.00);

    ptcloud_tf(seq(0, 1), all) = ptcloud;

    ptcloud_tf = T * ptcloud_tf;

    return ptcloud_tf(seq(0, 1), all);
}

struct ICPLoss : Functor<float>
{
    MatrixXf p, q;

    ICPLoss(void): Functor<float>(3, 3) {}

    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
        MatrixXf T = pose2tf(x);

        MatrixXf p_tf = transformPointCloud(p, T);

        MatrixXf d = p_tf - q;

        MatrixXf p_rot = T.block(0, 0, 2, 2) * p;

        fvec(0) = d(0, all).sum();
        fvec(1) = d(1, all).sum();
        fvec(2) = (d(0, all).array() * p_rot(0, all).array() + d(1, all).array() * p_rot(1, all).array()).sum();

        return 0;
    }

};

VectorXf solver(MatrixXf p, MatrixXf q)
{
    VectorXf x = VectorXf::Zero(3);

    ICPLoss loss_;

    loss_.p = p;
    loss_.q = q;

    Eigen::NumericalDiff<ICPLoss> gradient(loss_);

    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ICPLoss>, float> lvm(gradient);

    lvm.parameters.maxfev = 2000;

    lvm.parameters.xtol = 1e-12;

    int ret = lvm.minimize(x);

    return x;
}

MatrixXf runICP(MatrixXf p, MatrixXf q, int maxiters)
{
    VectorXf x = VectorXf::Zero(3);

    MatrixXf T = MatrixXf::Identity(3, 3);

    MatrixXf Tr = MatrixXf::Identity(3, 3);

    MatrixXf p_tf = transformPointCloud(p, T);

    knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> kdtree(q);

    kdtree.setBucketSize(10);

    kdtree.setSorted(true);

    kdtree.setTakeRoot(true);

    kdtree.setMaxDistance(0);

    kdtree.setThreads(6);

    kdtree.build();

    Matrixi indices;
    MatrixXf distances;

    for (int k = 0; k < maxiters; k++)
    {
        kdtree.query(p_tf, 1, indices, distances);

        Map<Array<long long, Dynamic, Dynamic>> idx_long(indices.data(), indices.cols(), indices.rows());

        ArrayXi idx = idx_long.cast <int> ();

        MatrixXf q_knn = q(all, idx);

        x = solver(p_tf, q_knn);

        p_tf = transformPointCloud(p_tf, pose2tf(x));

        Tr = pose2tf(x) * Tr;

    }

    return Tr;

}

int main()
{
    CSVData csv("scanData.csv", MatrixXf::Zero(1, 1));
    MatrixXf r = csv.readFromCSVfile().transpose();
    r = (r.array() < 50).select(r, 0.00);
    ArrayXf angle = ArrayXf::LinSpaced(1081, -3*PI/4, 3*PI/4);
    MatrixXf target = MatrixXf::Zero(2, 1081);
    MatrixXf source = MatrixXf::Zero(2, 1081);
    MatrixXf pc = MatrixXf::Zero(2, 1081);

    std::map<int, MatrixXf> point_clouds;

    int num_clouds = r.cols();

    for (int k = 0; k < num_clouds; k++)
    {
        pc << (r.col(k).array() * cos(angle)).matrix().transpose(),
                (r.col(k).array() * sin(angle)).matrix().transpose();

        point_clouds.insert({k, pc});
    }


    int target_idx = 70;
    int source_idx = 100;

    target = point_clouds[target_idx];
    source = point_clouds[source_idx];

    Matrix3f T = runICP(source, target, 50);

    cout << T << endl;

    CSVData tf("transformData.csv", T);

    tf.writeToCSVfile();

    return 0;
}