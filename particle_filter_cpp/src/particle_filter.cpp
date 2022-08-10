#include <cstdio>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <experimental/random>
#include <cmath>

#include <includes_for_pf_cpp.h>
#include <RangeLib.h>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace std;
using namespace Eigen;

float quaternion2angle(tf2::Quaternion q);
Eigen::Matrix2f rotationMatrix(float theta);
Eigen::MatrixXf map_to_world(Eigen::MatrixXf poses, float resolution, float x_origin, float y_origin, float angle);


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  cout << "hello world particle_filter_cpp package" << endl;
  return 0;
}
