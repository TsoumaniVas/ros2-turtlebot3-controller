# ROS2 TurtleBot3 Controller Project

This repository contains the code and results for a robotics assignment developed with ROS 2 and Python. The main goal is to control a TurtleBot3 robot and record its motion using ROS2 functionalities.

## Project Structure

* **robotdrive/controller.py**: Basic script that defines a node to move the robot in a predefined two-phase motion pattern (in-place rotation and straight motion with turn).
* **robotdrive/controller2.py**: Improved version with three phases of movement using cubic polynomials, with motion constraints and timing based on calculated trajectory segments.
* **robotdrive/controller3.py**: Implements a feedback control algorithm for smooth point-to-point robot motion using continuous odometry data.
* **setup.py**: Declares executable entry points for ROS2 (console\_scripts).
* **data/**: Contains output CSV files with motion data (exported from `.db3` files).
* **scripts/**: Contains data processing scripts such as `export_odom_to_csv.py` and `plot_rosbag2.py`.

## Execution Flow

1. **Package and Node Setup**

   * Create necessary directories and node files:

     ```bash
     mkdir -p ~/ros2_ws/src/robotcontrol/robotdrive
     touch ~/ros2_ws/src/robotcontrol/robotdrive/controller.py
     chmod +x ~/ros2_ws/src/robotcontrol/robotdrive/controller.py
     ```
   * Add executable in `setup.py`:

     ```python
     entry_points={
        'console_scripts': [
		    'controller = robotdrive.controller:main',
		    'controller2 = robotdrive.controller2:main',
		    'controller3 = robotdrive.controller3:main',
        ],
     }
     ```
   * Build the workspace:

     ```bash
     cd ~/ros2_ws
     colcon build
     source install/setup.bash
     ```

2. **Running the Controller**

   * Launch with:

     ```bash
     ros2 run robotcontrol controller
     ```

3. **Recording Odometry**

   * Record with rosbag2:

     ```bash
     ros2 bag record -o askisi1_bag /cmd_vel /odom
     ```

4. **Data Export and Plotting**

   * Convert `.db3` to `.csv` using `export_odom_to_csv.py`
   * Plot data using `plot_rosbag.py`

## Motion Phases Summary

### `controller.py`

* **Phase 1 (0-15s):** Robot rotates at 12°/s (≈0.21 rad/s).
* **Phase 2 (15–35s):** Robot moves forward at 0.15 m/s while rotating right (-8°/s).
* **End:** Robot stops and node shuts down.

### `controller2.py`

* Uses cubic polynomial profiles to ensure smooth trajectory segments.
* Divides motion into three phases:

  * Phase 1: Initial rotation.
  * Phase 2: Smooth linear motion from (0,0) to (xf,yf).
  * Phase 3: Final rotation to reach desired orientation θf.

### `controller3.py`

* Implements feedback control:

  * Continuously receives `/odom`.
  * Computes errors (position r, orientation θ, angles α and β).
  * Applies velocity control:

    ```
    v = kr * r
    ω = ka * α + kb * β
    ```

    (Constants: kr=0.4, ka=1, kb=-0.6)
  * Robot adjusts motion in real time without discrete motion phases.

## Graph Analysis

* Plots show linear and angular velocities across time.
* Additional plots visualize (x,y) robot position over time.
* Differences with expected plots are due to sampling gaps in rosbag2 recording.

---

For full documentation and step-by-step implementation, refer to the individual `.py` files and notebook records provided in this repository.
