# SAFE LANDING PLANNER
https://github.com/PX4/PX4-Avoidance

ROS node for safe drone landing site detection and waypoint generation.

The *safe_landing_planner* classifies the terrain underneath the vehicle based on the mean and standard deviation of the z coordinate of pointcloud points. The pointcloud from a downwards facing sensor is binned into a 2D grid based on the xy point coordinates. For each bin, the mean and standard deviation of z coordinate of the points are calculated and they are used to locate flat areas where it is safe to land.



# Table of Contents
- [Getting Started](#getting-started)
  - [Installation](#installation)
    - [Installation for Ubuntu](#installation)
  - [Run the Avoidance Gazebo Simulation](#run-the-avoidance-gazebosimulation)
    - [Safe Landing Planner](#safe-landing-planner)
  - [Run on Hardware](#run-on-hardware)
    - [Prerequisite](#prerequisite)
    - [Local Planner](#local-planner)
    - [Global Planner](#global-planner)
- [Troubleshooting](#troubleshooting)
- [Advanced](#advanced)
  - [Message Flows](#message-flow)
    - [PX4 and local planner](#px4-and-local-planner)
    - [PX4 and global planner](#px4-and-global-planner)
- [Contributing](#contributing)

# Getting Started

## Installation

### Prerequisite

1. Ubuntu 18.04 (Bionic Beaver) 

1. ROS Melodic

1. MAVROS

1. Clone and build this repository in your catkin workspace.
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/AnujAgrawal7/avoidance.git
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```
   
1. PX4 source code (https://github.com/PX4/PX4-Autopilot)

## Run the Avoidance Gazebo Simulation

### Build and Run the Simulator

1. Build the Firmware once in order to generate SDF model files for Gazebo.
   This step will actually run a simulation (that you can immediately close).

   ```bash
   # This is necessary to prevent some Qt-related errors (feel free to try to omit it)
   export QT_X11_NO_MITSHM=1

   # Build and run simulation
   make px4_sitl_default gazebo
   
   # Quit the simulation (Ctrl+C)

   # Setup some more Gazebo-related environment variables (modify this line based on the location of the Firmware folder on your machine)
   . ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
   ```

1. Add the Firmware directory to ROS_PACKAGE_PATH so that ROS can start PX4:
   ```bash
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/PX4-Autopilot
   ```
1. Finally, set the GAZEBO_MODEL_PATH in your bashrc:
   ```bash
   echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds" >> ~/.bashrc
   ```

The last three steps, together with sourcing your catkin **setup.bash** (`source ~/catkin_ws/devel/setup.bash`) should be repeated each time a new terminal window is open.



### Safe Landing Planner

This section shows how to start the *safe_landing_planner* and use it to land safely in mission or auto land mode.
To run the node:
 Set the parameter `COM_OBS_AVOID` to 1 in QGroundControl.

```bash
roslaunch safe_landing_planner safe_landing_planner.launch
```


You will see an unarmed vehicle on the ground. Open [QGroundControl](http://qgroundcontrol.com/), either plan a mission with the last item of type *Land* or fly around the world in Position Control, click the *Land* button on the left side where you wish to land.
At the land position, the vehicle will start to descend towards the ground until it is at `loiter_height` from the ground/obstacle. Then it will start loitering to evaluate the ground underneeth.
If the ground is flat, the vehicle will continue landing. Otherwise it will evaluate the close by terrain in a squared spiral pattern until it finds a good enough ground to land on.

# Run on Hardware

## Prerequisite

### Camera

The planner require a 3D point cloud of type `sensor_msgs::PointCloud2`. Any camera that can provide such data is compatible.

The officially supported camera is Intel Realsense D435. We recommend using Firmware version 5.9.13.0. The instructions on how to update the Firmware of the camera can be found [here](https://www.intel.com/content/www/us/en/support/articles/000028171/emerging-technologies/intel-realsense-technology.html)

> **Tip:** Be careful when attaching the camera with a USB3 cable. USB3 might might interfere with GPS and other signals. If possible, always use USB2 cables.

Other tested camera models are: Intel Realsense D415 and R200, Occipital Structure Core.


### PX4 Autopilot

Parameters to set through QGC:
* `COM_OBS_AVOID` to Enabled
* `MAV_1_CONFIG`, `MAV_1_MODE`, `SER_TEL2_BAUD` to enable MAVLink on a serial port. For more information: [PX4 Dev Guide](http://dev.px4.io/en/companion_computer/pixhawk_companion.html#pixhawk-setup)

### Companion Computer

* OS: Ubuntu 18.04 OS 
* ROS Melodic: see [Installation](#installation)
* Other Required Components for Intel Realsense:
  - Librealsense (Realsense SDK). The installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
  - [Librealsense ROS wrappers](https://github.com/intel-ros/realsense.git)
* Other Required Components for Occipital Structure Core:
  - Download the [Structure SDK](https://structure.io/developers). The version tested with this package is `0.7.1`. Create the `build` directory and build the SDK
  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ```
  - Clone the [ROS wrapper](https://github.com/Auterion/struct_core_ros) in the `catkin_ws`
  - Copy the shared object `Libraries/Structure/Linux/x86_64/libStructure.so` from the SDK into `/usr/local/lib/`
  - Copy the headers from `Libraries/Structure/Headers/` in the SDK to the ROS wrapper include directory `~/catkin_ws/src/struct_core_ros/include`

Tested models:
- local planner: Intel NUC, Jetson TX2, Intel Atom x7-Z8750 (built-in on Intel Aero RTF drone)
- global planner: Odroid



## Local Planner

Once the catkin workspace has been built, to run the planner with a Realsense D435 or Occipital Structure Core camera you can generate the launch file using the script *generate_launchfile.sh*

1. `export CAMERA_CONFIGS="camera_namespace, camera_type, serial_n, tf_x, tf_y, tf_z, tf_yaw, tf_pitch, tf_roll"` where  `camera_type` is either `realsense` or `struct_core_ros`, `tf_*` represents the displacement between the camera and the flight controller. If more than one camera is present, list the different camera configuration separated by a semicolon. Within each camera configuration the parameters are separated by commas.
2. `export DEPTH_CAMERA_FRAME_RATE=frame_rate`. If this variable isn't set, the default frame rate will be taken.
3. `export VEHICLE_CONFIG=/path/to/params.yaml` where the yaml file contains the value of some parameters different from the defaults set in the cfg file. If this variable isn't set, the default parameters values will be used.

Changing the serial number and `DEPTH_CAMERA_FRAME_RATE` don't have any effect on the Structure Core.

For example:
```bash
export CAMERA_CONFIGS="camera_main,realsense,819612070807,0.3,0.32,-0.11,0,0,0"
export DEPTH_CAMERA_FRAME_RATE=30
export VEHICLE_CONFIG=~/catkin_ws/src/avoidance/local_planner/cfg/params_vehicle_1.yaml
./tools/generate_launchfile.sh
roslaunch local_planner avoidance.launch fcu_url:=/dev/ttyACM0:57600
```

where `fcu_url` representing the port connecting the companion computer to the flight controller.
The planner is running correctly if the rate of the processed point cloud is around 10-20 Hz. To check the rate run:

```bash
rostopic hz /local_pointcloud
```

If you would like to read debug statements on the console, please change `custom_rosconsole.conf` to
```bash
log4j.logger.ros.local_planner=DEBUG
```

## Safe Landing Planner

Once the catkin workspace has been built, to run the planner with a Realsense D435 and Occipital Structure Core, you can generate the launch file using the script *generate_launchfile.sh*. The script works the same as described in the section above for the Local Planner. For example:

```bash
export CAMERA_CONFIGS="camera_main,struct_core,819612070807,0.3,0.32,-0.11,0,0,0"
export VEHICLE_CONFIG_SLP=~/catkin_ws/src/avoidance/safe_landing_planner/cfg/slpn_structure_core.yaml
export VEHICLE_CONFIG_WPG=~/catkin_ws/src/avoidance/safe_landing_planner/cfg/wpgn_structure_core.yaml
./safe_landing_planner/tools/generate_launchfile.sh
roslaunch safe_landing_planner safe_landing_planner_launch.launch
```

In the `cfg/` folder there are camera specific configurations for the algorithm nodes. These parameters can be loaded by specifying the file in the `VEHICLE_CONFIG_SLP` and `VEHICLE_CONFIG_WPG` system variable for the safe_landing_planner_node and for the waypoint_generator_node respectively.

The size of the squared shape patch of terrain below the vehicle that is evaluated by the algorithm can be changed to suit different vehicle sizes with the WaypointGeneratorNode parameter `smoothing_land_cell`. The algorithm behavior will also be affected by the height at which the decision to land or not is taken (`loiter_height` parameter in WaypointGeneratorNode) and by the size of neighborhood filter smoothing (`smoothing_size` in LandingSiteDetectionNode).

For different cameras you might also need to tune the thresholds on the number of points in each bin, standard deviation and mean.

# Troubleshooting

### I see the drone position in rviz (shown as a red arrow), but the world around is empty
Check that some camera topics (including */camera/depth/points*) are published with the following command:

```bash
rostopic list | grep camera
```

If */camera/depth/points* is the only one listed, it may be a sign that gazebo is not actually publishing data from the simulated depth camera. Verify this claim by running:

```bash
rostopic echo /camera/depth/points
```

When everything runs correctly, the previous command should show a lot of unreadable data in the terminal. If you don't receive any message, it probably means that gazebo is not publishing the camera data.

Check that the clock is being published by Gazebo:

```bash
rostopic echo /clock
```

If it is not, you have a problem with Gazebo (Did it finish loading the world? Do you see the buildings and the drone in the Gazebo UI?). However, if it is publishing the clock, then it might be a problem with the depth camera plugin. Make sure the package `ros-kinetic-gazebo-ros-pkgs` is installed. If not, install it and rebuild the Firmware (with `$ make px4_sitl_default gazebo` as explained above).

### I see the drone and world in rviz, but the drone does not move when I set a new "2D Nav Goal"
Is the drone in OFFBOARD mode? Is it armed and flying?

```bash
# Set the drone to OFFBOARD mode
rosrun mavros mavsys mode -c OFFBOARD
# Arm
rosrun mavros mavsafety arm
```

### I see the drone and world in rviz, but the drone does not follow the path properly
Some tuning may be required in the file *"<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris"*.

### I see the drone and world in rviz, I am in OFFBOARD mode, but the planner is still not working
Some parameters that can be tuned in *rqt reconfigure*.
