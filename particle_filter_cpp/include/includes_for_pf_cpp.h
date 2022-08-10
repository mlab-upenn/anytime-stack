#include <Eigen/Core>
#include <iostream> 
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
// using namespace Eigen;


float quaternion2angle(tf2::Quaternion q)
{
    	tf2::Matrix3x3 m(q);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);    
 	return yaw;
}


Eigen::Matrix2f rotationMatrix(float theta){
	float c = cos(theta);
	float s = sin(theta);
    	Eigen::Matrix2f m;
    	m(0,0) = c;
        m(0,1) = -s;
    	m(1,0) = s;
    	m(1,1) = c;
    return m;
}

Eigen::MatrixXf map_to_world(Eigen::MatrixXf poses, float resolution, float x_origin, float y_origin, float angle)
{
 	float c = cos(angle);
	float s = sin(angle);
    Eigen::VectorXf x = poses.col(0);
	Eigen::VectorXf y = poses.col(1);
	int n = x.size();
	poses.col(0) = (c*x - s*y);
	poses.col(1) = s*x + c*y;
	poses.col(2) *= resolution;
	poses.col(0) += Eigen::MatrixXf::Constant(n, 1, x_origin);
	poses.col(1) += Eigen::MatrixXf::Constant(n, 1, y_origin);
	poses.col(2) += Eigen::MatrixXf::Constant(n, 1, angle);
	return poses;

}


