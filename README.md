# anytime-stack

Anytime Autonomy Project

Anytime compute is a framework that can allow the controller to exploit the natural trade-off between the estimator's speed and accuracy during runtime by specifying a time of completion $\delta$ and error bound $\epsilon$, in the form of a contract specified by the controller based on its immediate control objectives.

**See Anytime_Compute_Study_README.pdf for details**

## Structure

* anytime-stack is a ROS 2 workspace, `src` contains ROS 2 packages as usual
* [anytime-icp-stack](./anytime-stack/src/anytime-icp-stack) directory contains the main implementation
	* [anytime-icp-stack/src/laser_icp_cpp](./anytime-stack/src/anytime-icp-stack/src/laser_icp.cpp) – Contains the main Anytime ICP Node with the shared memory interface
	* [anytime-icp-stack/src/anytime_stanley_controller_cpp](./anytime-stack/src/anytime-icp-stack/src/anytime_stanley_controller.cpp) – Contains the main Anytime Stanley Controller Node with the shared memory interface

## External Dependecies

* [ackermann_msgs] [https://index.ros.org/r/ackermann_msgs/#foxy].
* [urg_node] [https://index.ros.org/p/urg_node/#foxy]. This is the driver for Hokuyo LiDARs.
* [joy] [https://index.ros.org/p/joy/#foxy]. This is the driver for joysticks in ROS 2.
* [teleop_tools] [https://index.ros.org/p/teleop_tools/#foxy]. This is the package for teleop with joysticks in ROS 2.
* [vesc] GitHub - f1tenth/vesc at ros2. This is the driver for VESCs in ROS 2.
* [ackermann_mux] GitHub - f1tenth/ackermann_mux: Twist multiplexer. This is a package for multiplexing ackermann messages in ROS 2.
* [eigen3][https://eigen.tuxfamily.org/dox/GettingStarted.html]


## Nodes launched during startup
* joy
* joy_teleop
* ackermann_to_vesc_node
* vesc_to_odom_node
* vesc_driver_node
* urg_node
* ackermann_mux
* icp
* anytime_stanley

## Parameters and topics

### icp
* Parameters
    * max_iterations
* Publishes to:
    * icp/odom
* Subscribes to:
    * /scan
* Shared Memory Communication
    * anytime_stanley

### anytime_stanley
* Parameters
    * k_gain
    * speed
    * slope
    * intercept
    * steering_max
    * time_to_collision
* Publishes to:
    * /drive
* Subscribes to:
    * icp/odom
    * /joy
    * /scan
* Shared Memory Communication
    * icp

## Parameters and topics for dependencies

### vesc_driver
* Parameters:
    * duty_cycle_min, duty_cycle_max
    * current_min, current_max
    * brake_min, brake_max
    * speed_min, speed_max
    * position_min, position_max
    * servo_min, servo_max
* Publishes to:
    * sensors/core
    * sensors/servo_position_command
    * sensors/imu
    * sensors/imu/raw
* Subscribes to:
    * commands/motor/duty_cycle
    * commands/motor/current
    * commands/motor/brake
    * commands/motor/speed
    * commands/motor/position
    * commands/servo/position
### ackermann_to_vesc
* Parameters:
    * speed_to_erpm_gain
    * speed_to_erpm_offset
    * steering_angle_to_servo_gain
    * steering_angle_to_servo_offset
* Publishes to:
    * ackermann_cmd
* Subscribes to:
    * commands/motor/speed
    * commands/servo/position
### vesc_to_odom
* Parameters:
    * odom_frame
    * base_frame
    * use_servo_cmd_to_calc_angular_velocity
    * speed_to_erpm_gain
    * speed_to_erpm_offset
    * steering_angle_to_servo_gain
    * steering_angle_to_servo_offset
    * wheelbase
    * publish_tf
* Publishes to:
    * odom
* Subscribes to:
    * sensors/core
    * sensors/servo_position_command

## Development

**Requirements:**
* Ubuntu 20.04
* a working installation of ROS 2, see [Installing ROS 2 via Debian Packages][ros2-foxy-debian-pkgs]
* [vcstool](https://github.com/dirk-thomas/vcstool)
	* Test if you have it using: `vcs --version` (should print `vcs 0.3.0`)
	* You can install using `sudo apt install python3-vcstool` or `python3 -m pip install -U vcstool`


### Note about sourcing

**Note!    * Always use a separate terminal windows/tabs for building and running.

In a terminal window/tab where you are building the workspace, you must source **only    * the ROS 2
(i.e., `source /opt/ros/foxy.setup.bash`).

Then, in other terminal windows/tabs you can source the built workspace (`source install/setup.bash`) and run the
programs.

If you source workspace in the building terminal window/tab, then you will pollute the environment and the resulting
built workspace might not work correctly.


### Run when you pull the latest changes

Re-run `vcs` and `rosdep` when you pull the latest changes:
```bash
source /opt/ros/foxy/setup.bash
vcs import --input stack.repos
vcs pull --nested
rosdep install -i --from-paths src -y
```


<!-- links references -->

[ros2-foxy-debian-pkgs]: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
