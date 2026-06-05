# External BlueROV2 Direct-Thruster Path Controller

This note points to an external ROS 2 BlueROV2 path-following controller package for DAVE/Gazebo:

https://github.com/drwa92/bluerov2_reconciled_path_control_ros2

The package provides:

- go-to pose control;
- waypoint following;
- circle and spiral path following;
- generic trajectory following;
- stop and emergency-stop services;
- direct six-thruster BlueROV2 allocation;
- optional model-aided virtual-wrench reconciliation diagnostics.

The controller is designed to run with the DAVE BlueROV2 simulator in direct-control mode:

```bash
ros2 launch dave_demos dave_robot.launch.py \
  z:=-0.5 \
  namespace:=bluerov2 \
  world_name:=dave_ocean_waves \
  paused:=false \
  use_ardusub:=false \
  use_teleop:=false

ros2 launch bluerov2_path_control path_controller.launch.py \
  model_name:=bluerov2 \
  use_bridge:=true
The package is maintained externally by Waseem Akram at MARVIS LAB:

https://drwa92.github.io/marvis-lab/

Project webpage:

https://drwa92.github.io/bluerov2_reconciled_path_control_ros2/
