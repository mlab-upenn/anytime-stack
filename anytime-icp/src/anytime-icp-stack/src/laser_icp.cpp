#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "knn.h"


using namespace std;
using namespace Eigen;

typedef knncpp::Matrixi Matrixi;

# define PI 3.14159265358979323846

// Setup the shared memory file and obtain key and shared memory id (change the filepath to desired filepath)
// Will heap allocate 1024 bytes 

key_t key = ftok("/home/f1tenth3/shmfile", 65);
int shmid = shmget(key, 1024, 0666 | IPC_CREAT);

// Generic functor template that the ICP loss function will inherit from
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum
    {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };

    // This is needed since the LVM solver requires stack allocated matrices and so the sizes need to be known at compile time

    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

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

// Utilities

// Convert raw LiDAR ranges to point cloud

MatrixXf range2pc(ArrayXf r)
{
    ArrayXf angle = ArrayXf::LinSpaced(1081, -3*PI/4, 3*PI/4);
    MatrixXf pc = MatrixXf::Zero(2, 1081);
    pc << (r * cos(angle)).matrix().transpose(),
          (r * sin(angle)).matrix().transpose();
    
    return pc;
}

// Convert from a pose vector to a homogenous transformation matrix

MatrixXf pose2tf(VectorXf x)
{
    VectorXf t = x(seq(0, 1), 0);

    MatrixXf R = MatrixXf::Zero(2, 2);

    R << cos(x(2)), -sin(x(2)),
         sin(x(2)), +cos(x(2));

    MatrixXf T = MatrixXf::Identity(3, 3);

    T.block(0, 0, 2, 2) = R;
    T.block(0, 2, 2, 1) = t;

    return T;
}

// Apply homogenous transformation to the given point cloud

MatrixXf transformPointCloud(MatrixXf ptcloud, MatrixXf T)
{
    int n_pts = ptcloud.cols();

    int n_dims = ptcloud.rows();

    MatrixXf ptcloud_tf = MatrixXf::Constant(n_dims + 1, n_pts, 1.00);

    ptcloud_tf(seq(0, 1), seq(0, n_pts-1)) = ptcloud;

    ptcloud_tf = T * ptcloud_tf;

    return ptcloud_tf(seq(0, 1), seq(0, n_pts-1));
}

// ICP distance loss function (inherited from Functor class; as needed by the LVM solver interface)

struct ICPLoss : Functor<float>
{
    MatrixXf p, q;

    ICPLoss(void): Functor<float>(3, 3) {}

    // This function computes the gradient of the ICP loss function

    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
        int n_pts = p.cols();

        MatrixXf T = pose2tf(x);

        MatrixXf R_dUdt(2, 2);
        
        R_dUdt << -sin(x(2)), -cos(x(2)),
                   cos(x(2)), -sin(x(2));

        MatrixXf p_tf = transformPointCloud(p, T);

        MatrixXf d = p_tf - q;

        MatrixXf p_rot = R_dUdt * p;

        fvec(0) = d(0, seq(0, n_pts-1)).sum();
        fvec(1) = d(1, seq(0, n_pts-1)).sum();
        fvec(2) = (d(0, seq(0, n_pts-1)).array() * p_rot(0, seq(0, n_pts-1)).array() + d(1, seq(0, n_pts-1)).array() * p_rot(1, seq(0, n_pts-1)).array()).sum();

        return 0;
    }

};

// Wrapper function for Levenberg Marquardt nonlinear solver

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

    // Setup initial vectors and matrices

    VectorXf x = VectorXf::Zero(3);

    MatrixXf T = MatrixXf::Identity(3, 3);

    MatrixXf Tr = MatrixXf::Identity(3, 3);

    MatrixXf p_tf = transformPointCloud(p, T);

    // Setup kdtree for nearest neighbours computation with previous point cloud (q)

    knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> kdtree(q);

    kdtree.setBucketSize(10);

    kdtree.setSorted(true);

    kdtree.setTakeRoot(true);

    kdtree.setMaxDistance(0);

    kdtree.setThreads(0);

    kdtree.build();

    Matrixi indices;
    MatrixXf distances;

    // Get interrupt signal from shared memory

    int *interrupt = (int*) shmat(shmid, (void*) 0, 0);

    for (int k = 0; k < maxiters; k++)
    {

        // Pass the current number of iterations to the controller node via the shared memory pointer

        *(interrupt + 2) = k;

        // Obtain 1st nearest point indices and distances from previous point cloud to current point cloud 

        kdtree.query(p_tf, 1, indices, distances);

        // Type cast from Matrixi (indices matrix) to long int matrix using the Map function

        Map<Array<long int, Dynamic, Dynamic>> idx_long(indices.data(), indices.cols(), indices.rows());

        // Type cast from long int to int matrix

        ArrayXi idx = idx_long.cast <int> ();

        // Slice previous point cloud to get nearest correspondences

        MatrixXf q_knn = q(seq(0, 1), idx);

        // Obtain optimal pose estimate from Levenberg-Marquardt nonlinear solver

        x = solver(p_tf, q_knn);

        // Convert pose to homogenous transform and transform current point cloud

        p_tf = transformPointCloud(p_tf, pose2tf(x));

        // Update current transformation estimate

        Tr = pose2tf(x) * Tr;

        // Check to see if the current iteration count of the ICP has exceeded max desired iterations from the controller node

        if (*interrupt <= k)
        {
            *interrupt = 0;
            break;
        }

    }

    // Send handshaking signal from the ICP node to the controller node to signal that ICP is completed

    *(interrupt + 1) = 1;

    // Return homogenous transform matrix

    return Tr;

}

class Laser : public rclcpp::Node
{
	std::vector<float> axs, range;
	std::vector<int> btns;
	bool initial;
	MatrixXf p, q, Tr, T;
    Matrix3f R_3;
	Quaternionf q_orientation;
	int maxiters;
	
	public:
	
	Laser() : Node("Laser_control")
	{
        // Get max iterations parameter from external yaml file

        declare_parameter("max_iters", 40);
        maxiters = get_parameter("max_iters").as_int();

        // Set initial to true and create identity rotation matrix

        initial = true;
        R_3 = Eigen::Matrix3f::Identity();

        // Subscribe to LiDAR and create odometry publisher

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ process_laser(msg); });
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/icp/odom", 1);
	}

	private:
	
	void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr laser_in)
	{

        // Get ranges from LiDAR and convert to Eigen Matrix

        range = laser_in.get()->ranges;
        Map<ArrayXf> r(&range[0], range.size());
        r = (r.array() < 50).select(r, 0.0);

        // Set the first point cloud to the previous point cloud (q) : Initialization

        if(initial)
        {
            q = range2pc(r);
            T = Eigen::MatrixXf::Identity(3, 3);
            Tr = Eigen::MatrixXf::Identity(3, 3);
            initial = false;
        }

        // Convert LiDAR ranges to 2D point cloud

        p = range2pc(r);

        // Run ICP between successive point clouds and get homogenous transformation matrix

        T = runICP(q, p, maxiters);

        // Update current transform estimate

        Tr = Tr * T;

        // Set previous point cloud (q) to current point cloud (p)

        q = p;

        // Convert Homogenous Transformation to Quaternion

        R_3.block(0, 0, 2, 2) = Tr.block(0, 0, 2, 2);

        q_orientation = R_3;

        // Declare odometry message and set transform frames
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Position
        odom.pose.pose.position.x = Tr(0, 2);
        odom.pose.pose.position.y = Tr(1, 2);

        // Orientation
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = q_orientation.z();
        odom.pose.pose.orientation.w = q_orientation.w();

        odom_pub_->publish(odom);


	}
	
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Laser>());
  rclcpp::shutdown();
  return 0;
}
