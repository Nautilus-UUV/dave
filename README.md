# DAVE

All documentation of Project DAVE can be found in the [Dave Documentation (https://field-robotics-lab.github.io/dave.doc/)](https://field-robotics-lab.github.io/dave.doc/).

## Installation

### Requirements
- ROS Noetic already setup
- `catkin_tools` installed to build packages

1. Clone this repo into your catkin workspace
2. Install the following additional dependencies to ros-noetic
	```
	DIST=noetic
	GAZ=gazebo11
	sudo apt install -y ${GAZ} lib${GAZ}-dev python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-gazebo-ros-control ros-${DIST}-gazebo-ros-pkgs ros-${DIST}-effort-controllers ros-${DIST}-geographic-info ros-${DIST}-hector-gazebo-plugins ros-${DIST}-image-view ros-${DIST}-joint-state-controller ros-${DIST}-joint-state-publisher ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-kdl-parser-py ros-${DIST}-key-teleop ros-${DIST}-move-base ros-${DIST}-moveit-commander ros-${DIST}-moveit-planners ros-${DIST}-moveit-simple-controller-manager ros-${DIST}-moveit-ros-visualization ros-${DIST}-pcl-ros ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-ros-base ros-${DIST}-ros-controllers ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins ros-${DIST}-rqt-robot-plugins ros-${DIST}-rviz ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-joy ros-${DIST}-teleop-twist-keyboard ros-${DIST}-tf2-geometry-msgs ros-${DIST}-tf2-tools ros-${DIST}-urdfdom-py ros-${DIST}-velodyne-gazebo-plugins ros-${DIST}-velodyne-simulator ros-${DIST}-xacro
	```
3. use vstools to install dependencies by running `vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .`
4. build packages by running `catkin build`
5. source workspace by running `source devel/setup.bash`

to use additional glider physics from glider_hybrid_whoi
1. clone glider_hybrid_whoi repo into your catkin workspace by running `git clone git@github.com:nautilus-uuv/glider_hybrid_whoi.git`
2. use vstools to install dependencies by running `vcs import --skip-existing --input glider_hybrid_whoi/extras/repos/glider_hybrid_whoi.repos .`
3. If the computer is using a discrete nvidia GPU. Install cuda-toolkit by running `sudo apt install nvidia-cuda-toolkit`.
4. build packages by running `catkin build`
5. Source workspace by running `source devel/setup.bash`
