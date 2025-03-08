# LAB1_WS – Mobile Robot Simulation

- [Lab 1.1](#lab-11---mobile-robot-kinematics)
- [Lab 1.2](#lab-12---path-tracking-controller)
- [Lab 1.3](#lab-13---state-estimator)

## Lab 1.1 - Mobile Robot Kinematics
## Prerequisites
Ensure you have ROS 2 Humble installed. If rosdep is not initialized, it will be set up automatically.


## Installation
Clone the repository and install dependencies:
```shell
git clone https://github.com/beamkeerati/LAB1_WS.git
cd LAB1_WS

sudo apt update
sudo apt install -y python3-rosdep 

if ! rosdep db > /dev/null 2>&1; then
    sudo rosdep init
fi
rosdep update
rosdep install -y -i --from-paths src

sudo apt install ros-humble-rqt*
sudo apt install ros-humble-tf*

colcon build
source install/setup.bash

```


## Running the Simulation
Launch the robot simulation:
```shell
ros2 launch limo_bringup limo_bringup_1.1.launch.py
```
## Notes
- Ensure your ROS 2 environment is sourced before running (source install/setup.bash).
- If dependencies are missing, rerun rosdep install.
- Try this command if there's an unexpected error with dependencies `sudo apt install -y ros-humble-rqt-common-plugins ros-humble-tf2-ros`

## Results

From this point, we will discuss the results observed from different trajectories plotted by subscribing to each topics and plot them into matplotlib.

Note 1: 
You might notice some unusual behavior in the ground truth values on the linear velocity graph. This is possibly due to errors in certain data topics. However, when compared to the PlotJuggler representation, the error values appear consistent.

Note2:
The straight line appearing before the graph's movement represents idle time before the actual movement is triggered.

---
Plotjuggler comparison
![Plotjuggler.png](/images/1.1/Ground_Truth_Error_Proof_Bi_Square_0.3_-0.4.png)

---

### Circular Trajectory

The yaw rate and single-track results are close to the ground truth, while the double-track result deviates significantly. This trend is also evident in the yaw graph and both velocity graphs.

---
Bicycle mode circular
![Bicycle_Circle.png](/images/1.1/Bi_Circle_0.3_-0.4.png)

---
Car mode circular
![Car_Circle.png](/images/1.1/Car_Circle_0.3_-0.4.png)

---

### Square Trajectory

With this trajectory, we can clearly see that yaw rate—as the name suggests—does not struggle with the Z-angular position at all. However, its magnitude is slightly smaller compared to the ground truth. The other two models exhibit the same error trend, but the single-track model manages to keep the error small, whereas the double-track model shows a significantly larger error. Nonetheless, both exhibit incremental angle errors.

In car mode, with no slip, the square trajectory is generated more accurately than in the bicycle model.

---
Bicycle mode square
![Bicycle_Square.png](/images/1.1/Bi_Square_0.3_-0.4.png)

---
Car mode square
![Car_Square.png](/images/1.1/Car_Square_0.3_-0.4.png)

---

### Wave Trajectory

This trajectory retains the same fundamental characteristics as the previous one; however, upon closer examination, subtle differences emerge between left and right turns. Specifically, the trajectory reveals a slight asymmetry in the turning behavior, indicating that left and right turns do not exhibit identical dynamics. This discrepancy could stem from factors such as mechanical imbalances, sensor inaccuracies, or variations in control execution. Understanding and addressing these differences is crucial for improving trajectory consistency and overall system performance.

---
Bicycle mode wave
![Bicycle_Wave.png](/images/1.1/Bi_Wave_0.3_-0.3.png)

---
Car mode wave
![Car_Wave.png](/images/1.1/Car_Wave_0.3_-0.3.png)

---

### Summary

This project implements a forward and inverse kinematics solution for a robotic vehicle using ROS 2. The forward kinematics (FK) node subscribes to joint state and IMU topics to compute the robot's odometry based on different kinematic models, such as double-track, single-track, and yaw-rate, and publishes the odometry data as `nav_msgs/Odometry`. The inverse kinematics (IK) node receives target linear and angular velocities and uses a control algorithm to compute the required wheel velocities, which are then sent to the vehicle's actuators. The FK model incorporates parameters such as wheelbase, track width, and yaw rate, while the IK model uses desired linear and angular velocity inputs to calculate the appropriate control outputs. Additionally, the system's state can be visualized in RViz, providing real-time feedback on the robot's position, velocity, and kinematic model. In simulation, the robot's behavior is modeled in Gazebo, allowing for testing of the kinematic models and actuator controls in a realistic virtual environment.

Despite the ideal nature of the implemented models, several errors persist in each measurement method. These include discrepancies between the simulated and actual robot motion, errors in odometry calculations due to sensor inaccuracies, and deviations in control response when transitioning between different kinematic models. Although the system works well under ideal conditions, further tuning and sensor calibration will be necessary to reduce these errors and improve the overall performance for real-world applications.

The discrepancies in trajectory between the kinematic models arise because each model relies on different assumptions about the robot's movement and the control system parameters. Specifically, the tuning of the K gain and other parameters in the xacro file can directly affect how the models perform in practice.

1. Tuning K gain: Each kinematic model (double-track, single-track, and yaw-rate) has different control dynamics. The K gain in your controller is critical for adjusting the robot's response to the desired velocity or trajectory. A low K gain might cause the robot to react too slowly, while a high K gain can lead to overshooting or instability. Therefore, tuning the K gain is necessary to ensure smooth transitions between desired and actual movement, especially when switching between different models.

2. xacro parameters: The xacro file contains parameters such as wheelbase, track width, and other mechanical properties that define the robot's geometry. Small variations in these parameters can lead to different behaviors in the kinematic models. For example, the wheelbase length directly impacts how the robot turns, and tuning this value is important for accurate motion representation. Similarly, the yaw rate model depends heavily on how the robot's sensors and actuators are configured in the simulation.

By carefully tuning these parameters in both the control algorithms and the xacro file, the discrepancies can be minimized ensuring that the models provide a more consistent trajectory across the different kinematic approaches. This process involves iteratively adjusting the values and testing them in both simulation (Gazebo) and real-world applications to achieve optimal performance.

---

## Lab 1.2 - Path Tracking Controller
different controllers can be selected as such.
```shell
ros2 launch limo_bringup limo_bringup_1.2_pid.launch.py
```
```shell
ros2 launch limo_bringup limo_bringup_1.2_pure_pursuit.launch.py
```
```shell
ros2 launch limo_bringup limo_bringup_1.2_stanley.launch.py
```

Three out of four controllers were selected and developed. Here are the detailed results for each controller.

### PID Method
![PID.png](/images/1.2/PID.png)

referring from [PID Control Method](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PID.html), there are essentially three parameters that can be configured and tuned to match the path trajectory. Although the PID method is stated to be more suitable for throttle control rather than lateral control, we implemented it for steering to the best of our ability. Our experiment demonstrated that, even if not optimal, PID can still be a feasible method on a simpler track.

each PID gains can be dynamically tuned as follow:

get the current K values
```shell
ros2 param get /pid_node Kp
ros2 param get /pid_node Ki
ros2 param get /pid_node Kd
```
To set each K values (replace the number with your own desired value)
```shell
ros2 param set /pid_node Kp 15.0
ros2 param set /pid_node Ki 2.0
ros2 param set /pid_node Kd 0.5
```

other miscellaneous parameters can be configured if needed.
```shell
ros2 param list /pid_node 
```

### Pure pursuit Method
![Pure_Pursuit.png](/images/1.2/Pure_Pursuit.png)

This is our most successful and ideal controller for lateral control so far. The results for the single-track model, yaw rate, and ground truth are very similar, while only the double-track model's path deviated.

There are no dynamically configurable parameters in this method. The LIMO robot consistently follows the path while maintaining a set distance to the path at all times.

### Stanley Method
![Stanley.png](/images/1.2/Stanley.png)


---

## Lab 1.3 - State Estimator