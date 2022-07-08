#include <cstdio>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "RangeLib.h"

#define MAX_DISTANCE 500
#define THETA_DISC 108
#define MB (1024.0*1024.0)

#define Q(x) #x
#define QUOTE(x) Q(x)

#define GRID_STEP 10
#define GRID_RAYS 40
#define GRID_SAMPLES 1
#define RANDOM_SAMPLES 200000


using namespace ranges;
using namespace std;
using namespace Eigen;

using namespace std::chrono_literals;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

class ParticleFilter : public rclcpp::Node
{
  /*
    Add Class Variables 
  */
  OMap map = OMap(1,1);

  public:
  ParticleFilter() : Node("particle_filter")
  {
    // Map Service Client
    omap_client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");
    get_omap();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I've exited the map client");
  }

  private:

  void get_omap()
  {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("map_client");

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    while (!omap_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map service not available, waiting...");
    }

    using ServiceResponseFuture = rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture;

    auto response_received_callback = [this](ServiceResponseFuture future)
    {
        auto result = future.get();
    };

    auto future_result = omap_client_->async_send_request(request, response_received_callback);

    auto map_msg = future_result.get()->map;

    // float resolution = future_result.get()->map.info.resolution;

    // int w = future_result.get()->map.info.width;
    // int h = future_result.get()->map.info.height;

    map = OMap(100, 100);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lets see %d", (int)map.width);

    // int MAX_RANGE_PX = (int)(MAX_DISTANCE/resolution);

    // OMap map = OMap(w, h);

    // map = OMap(future_result.get()->map.data);

    // RayMarchingGPU rmgpu(map, MAX_RANGE_PX);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Okay! I got the map!");
        
  }

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr omap_client_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilter>());
  rclcpp::shutdown();
  return 0;
}
