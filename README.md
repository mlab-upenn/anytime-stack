# anytime-stack

Anytime Autonomy Project

## Structure

* the project root is a ROS 2 workspace, `src` contains ROS 2 packages as usual
* [algorithms](./algorithms) directory contains non-ROS code (algorithms implementations in Python and C++)
	* [algorithms/anytime_cpp](./algorithms/anytime_cpp) – CMake project with all C++ algorithms implementations
	* [algorithms/icp_py](./algorithms/icp_py) – Python algorithms implementations
	* [algorithms/pf_data_collection](./algorithms/pf_data_collection) – scripts for Particle Filter data collection


## Development

**Requirements:**
* Ubuntu 20.04
* a working installation of ROS 2, see [Installing ROS 2 via Debian Packages][ros2-galactic-debian-pkgs]
* [vcstool](https://github.com/dirk-thomas/vcstool)
	* Test if you have it using: `vcs --version` (should print `vcs 0.3.0`)
	* You can install using `sudo apt install python3-vcstool` or `python3 -m pip install -U vcstool`


### Note about sourcing

**Note!** Always use a separate terminal windows/tabs for building and running.

In a terminal window/tab where you are building the workspace, you must source **only** the ROS 2
(i.e., `source /opt/ros/galactic.setup.bash`).

Then, in other terminal windows/tabs you can source the built workspace (`source install/setup.bash`) and run the
programs.

If you source workspace in the building terminal window/tab, then you will pollute the environment and the resulting
built workspace might not work correctly.


### Run when you pull the latest changes

Re-run `vcs` and `rosdep` when you pull the latest changes:
```bash
source /opt/ros/galactic/setup.bash
vcs import --input stack.repos
vcs pull --nested
rosdep install -i --from-paths src -y
```


<!-- links references -->

[ros2-galactic-debian-pkgs]: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
