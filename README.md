# DAVE - Nautilus-UUV Fork

[![Publish a Docker image (AMD64; Common X86_64 Linux Machine)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml/badge.svg)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml)
[![Publish a Docker image (ARM64; Apple Silicon)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml/badge.svg?branch=ros2)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml)

This is the Nautilus-UUV fork of DAVE (Aquatic Robotic Simulator) containing custom configurations for the Glider Nautilus robot.

**Upstream DAVE Documentation**: [http://dave-ros2.notion.site](http://dave-ros2.notion.site)

## Installation

### Prerequisites

1. **Install DAVE** following the official installation tutorial:
   - [DAVE Native Installation Guide](https://dave-ros2.notion.site/Native-Local-Installation-Manual-7c6d7be83a4947d28ae3e3eb6b7de5ee)
   - Complete all steps through building the base DAVE workspace

### Installing Nautilus Custom Models

After completing the official DAVE installation:

```bash
# Navigate to the DAVE workspace
cd ~/dave_ws/src

# Remove the default DAVE repository
rm -rf dave

# Clone the Nautilus fork with the dev branch
git clone -b dev https://github.com/Nautilus-UUV/dave.git
```

Source your ros2 and correspondic gazebo versions:

```bash
# Source ros2 jazzy and gazebo harmonic
source /opt/ros/jazzy/setup.bash
source /opt/gazebo/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/opt/gazebo/install/lib/python
```



Build custom dave:

```bash
# Navigate back to workspace root
cd ~/dave_ws

# Remove any existing build artifacts
rm -rf build/ install/ log/

# Rebuild the workspace with Nautilus customizations
# NOTE: we skip the Hardware Abstraction Layer (HAL) of the actual control
colcon build --packages-skip nautilus_hal --symlink-install

# Source the workspace
source install/setup.bash
```

**Optional:**
Add to .bashrc_aliases to source dave by running `dave`:
```bash
alias jazzy='source /opt/ros/jazzy/setup.bash'
alias harmonic='source /opt/gazebo/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/opt/gazebo/install/lib/python'
alias dave='jazzy && harmonic && source ~/dave_ws/install/setup.bash'
```

> [!NOTE]
> That's right, we don't technically need to follow the DAVE installation since we are deleting it and rebuilding it. All that is needed is a working ros2 and corresponding gazebo version.


## Running the [nautilus-ros Repository](https://github.com/Nautilus-UUV/nautilus-ros) with the Digital Twin

```bash
cd ~/dave_ws/src
git clone -b dev ... # [INSERT ROS2 REPO]

cd ~/dave_ws
rm -rf build/ install/ log/

colcon build --symlink-install # this time not skipping the nautilus_hal

source install/setup.bash
```

### Testing Setup

Run:
```bash
ros2 launch nautilus_hal sawtooth_sim.launch.py \
    headless:=false \
    mission_autostart:=true \
    target_pressure_pa:=147150.0 \
    shallow_pressure_pa:=49050.0 \
    angle_rad:=0.6109 \
    n_oscillations:=3
```

You should see the glider going in a sawtooth motion between the deep
(`target_pressure_pa`) and shallow (`shallow_pressure_pa`) pressures, then
surfacing after `n_oscillations` dives. `shallow_pressure_pa:=0` climbs all the
way to the surface between dives. See the launch files in
`dave_ws/src/dave/hal/nautilus_hal/launch` for a complete list.

### Data Collection

```bash
ros2 launch nautilus_hal sawtooth_sim.launch.py \
    headless:=true \
    mission_autostart:=true \
    target_pressure_pa:=147150.0 \
    shallow_pressure_pa:=49050.0 \
    angle_rad:=0.6109 \
    n_oscillations:=3 \
    record:=true
```

**args:**
- `record:=true` to enable databag generation
- `run_id:={ID}` to give a predefined run_id that is concatenated together with the timestamp
- `bag_path:={PATH}` if you want to override the default data collection path
- `scenario:={PATH}` selects the scenario YAML driving gains, plant, bridge publish rates, and fault injection. Defaults to the installed `library/nominal.yaml` (perturbation-free, fault-injection off). To turn BCU fault injection back on (MTTF ~60 s), pick `baseline.yaml`:

  ```bash
  scenario:=$(ros2 pkg prefix py_pkg)/share/py_pkg/scenarios/library/baseline.yaml
  ```

  Same flag works for `trim_sim.launch.py`, `sawtooth_sim.launch.py`, `surface_sim.launch.py`, `bridge.launch.py`, and `control_stack.launch.py`.


### UI Connection

#### Mission Laptop

```bash
mosquitto -c ./mosquitto/mosquitto.conf -v
```

```bash
npm run dev
```

#### Pi

Start the HAL:
```bash
ros2 launch nautilus_hal bridge.launch.py
```

Start the simulation:
```bash
ros2 launch dave_demos dave_robot.launch.py \
    namespace:=glider_nautilus world_name:=dave_ocean_waves \
    z:=-5 roll:=3.141592653589793 yaw:=1.5707963267948966 \
    paused:=false headless:=false
```

Start control:
```bash
ros2 launch py_pkg control_stack.launch.py
```

## Branch Structure

Our fork uses a structured branching model:

```
polaris-ros2 (stable - tracks upstream + Nautilus customizations)
    └── dev (active development - merge feature branches here)
        └── github_username/feature_name
```

> [!NOTE]
> `dev` contains a [fork sync](.github/workflows/fork-sync.yml) workflow to automatically sync changes from upstream

## Contributing Workflow

### Create a Feature Branch

```bash
# Make sure you're on the dev branch
git checkout dev

# Pull the latest changes
git pull origin dev

# Create your feature branch
git checkout -b github_username/feature_name
```

### Make Your Changes

Edit files in the appropriate locations:

- **Robot models**: `models/dave_robot_models/description/glider_nautilus/`
- **Configurations**: `models/dave_robot_models/config/glider_nautilus/`
- **Meshes**: `models/dave_robot_models/meshes/glider_nautilus/`
- **Launch files**: Add to appropriate package directories

```
dave/
├── models/
│   └── dave_robot_models/
│       ├── description/
│       │   └── glider_nautilus/    # Robot SDF files
│       ├── config/
│       │   └── glider_nautilus/    # Launch configuration ros_gz_bridge
│       └── meshes/
│           └── glider_nautilus/    # 3D mesh files
├── README.md                       # This file
└── ...                             # Other DAVE packages
```

### Test Your Changes

```bash
# Rebuild the workspace
cd ~/dave_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Test your changes (launch files, simulations, etc.)
ros2 launch <your_test_commands>
```

