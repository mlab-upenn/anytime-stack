#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include "knn.h"


using namespace std;
using namespace Eigen;

typedef knncpp::Matrixi Matrixi;

# define PI 3.14159265358979323846

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
MatrixXf range2pc(ArrayXf r)
{
    ArrayXf angle = ArrayXf::LinSpaced(1081, -3*PI/4, 3*PI/4);
    MatrixXf pc = MatrixXf::Zero(2, 1081);
    pc << (r * cos(angle)).matrix().transpose(),
          (r * sin(angle)).matrix().transpose();
    
    return pc;
}

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

    ptcloud_tf(seq(0, 1), seq(0, n_pts-1)) = ptcloud;

    ptcloud_tf = T * ptcloud_tf;

    return ptcloud_tf(seq(0, 1), seq(0, n_pts-1));
}

struct ICPLoss : Functor<float>
{
    MatrixXf p, q;

    ICPLoss(void): Functor<float>(3, 3) {}

    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
        int n_pts = p.cols();

        MatrixXf T = pose2tf(x);

        MatrixXf p_tf = transformPointCloud(p, T);

        MatrixXf d = p_tf - q;

        MatrixXf p_rot = T.block(0, 0, 2, 2) * p;

        fvec(0) = d(0, seq(0, n_pts-1)).sum();
        fvec(1) = d(1, seq(0, n_pts-1)).sum();
        fvec(2) = (d(0, seq(0, n_pts-1)).array() * p_rot(0, seq(0, n_pts-1)).array() + d(1, seq(0, n_pts-1)).array() * p_rot(1, seq(0, n_pts-1)).array()).sum();

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

        Map<Array<long int, Dynamic, Dynamic>> idx_long(indices.data(), indices.cols(), indices.rows());

        ArrayXi idx = idx_long.cast <int> ();

        MatrixXf q_knn = q(seq(0, 1), idx);

        x = solver(p_tf, q_knn);

        p_tf = transformPointCloud(p_tf, pose2tf(x));

        Tr = pose2tf(x) * Tr;

        // Tr = Tr * pose2tf(x);

    }

    return Tr;

}

// class Laser : public rclcpp::Node
// {
// 	std::vector<float> axs, range;
// 	std::vector<int> btns;
// 	bool initial;
// 	MatrixXf p, q, Tr, T;
//     Matrix3f R_3;
// 	Quaternionf q_orientation;
	
// 	public:
	
// 	Laser() : Node("Laser_control")
// 	{
//         initial = true;
//         R_3 = Eigen::Matrix3f::Identity();
// 		laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ process_laser(msg); });
//         odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/icp/odom", 1);
// 	}

// 	private:
	
// 	void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr laser_in)
// 	{
// 		range = laser_in.get()->ranges;
// 		Map<ArrayXf> r(&range[0], range.size());
//         r = (r.array() < 50).select(r, 0.0);

//         if(initial)
//         {
//             q = range2pc(r);
//             T = Eigen::MatrixXf::Identity(3, 3);
//             Tr = Eigen::MatrixXf::Identity(3, 3);
//             initial = false;
//         }

//         p = range2pc(r);

//         T = runICP(p, q, 50);

//         Tr = Tr * T;

//         q = p;

//         R_3.block(0, 0, 2, 2) = Tr.block(0, 0, 2, 2);

//         q_orientation = R_3;

//         nav_msgs::msg::Odometry odom;
//         odom.header.frame_id = "odom";
//         odom.child_frame_id = "base_link";

//         // Position
//         odom.pose.pose.position.x = Tr(0, 2);
//         odom.pose.pose.position.y = Tr(1, 2);
//         odom.pose.pose.orientation.x = 0.0;
//         odom.pose.pose.orientation.y = 0.0;
//         odom.pose.pose.orientation.z = q_orientation.z();
//         odom.pose.pose.orientation.w = q_orientation.w();

//         odom_pub_->publish(odom);


// 	}
	
// 	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
// };

int main()
{
    // CSVData csv("scanData.csv", MatrixXf::Zero(1, 1));
    // MatrixXf r = csv.readFromCSVfile().transpose();
    // r = (r.array() < 50).select(r, 0.00);
    // ArrayXf angle = ArrayXf::LinSpaced(1081, -3*PI/4, 3*PI/4);
    // MatrixXf target = MatrixXf::Zero(2, 1081);
    // MatrixXf source = MatrixXf::Zero(2, 1081);
    // MatrixXf pc = MatrixXf::Zero(2, 1081);

    // std::map<int, MatrixXf> point_clouds;

    // int num_clouds = r.cols();

    // for (int k = 0; k < num_clouds; k++)
    // {
    //     pc << (r.col(k).array() * cos(angle)).matrix().transpose(),
    //             (r.col(k).array() * sin(angle)).matrix().transpose();

    //     point_clouds.insert({k, pc});
    // }


    // int target_idx = 70;
    // int source_idx = 100;

    // target = point_clouds[target_idx];
    // source = point_clouds[source_idx];

    CSVData csv1("source.csv", MatrixXf::Zero(1, 1));

    CSVData csv2("target.csv", MatrixXf::Zero(1, 1));

    MatrixXf target = csv2.readFromCSVfile();

    MatrixXf source = csv1.readFromCSVfile();

    MatrixXf T = runICP(source, target, 50);

    cout << T << endl;

    // Matrix3f T = runICP(source, target, 50);

    // cout << T << endl;

    // CSVData tf("transformData.csv", T);

    // tf.writeToCSVfile();

    return 0;
}