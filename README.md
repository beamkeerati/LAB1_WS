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

from [1.1 launch](/src/limo_bringup/launch/limo_bringup_1.1.launch.py) & [limo plot](/src/limo_results/scripts/limo_plot.py)

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

- [PID](#pid-method)
- [Pure Pursuit](#pure-pursuit-method)
- [Stanley](#stanley-method)

different controllers can be selected as such.

```shell
ros2 launch limo_bringup limo_bringup_1.2.launch.py controller:=pid
```

```shell
ros2 launch limo_bringup limo_bringup_1.2.launch.py controller:=pure_pursuit
```

```shell
ros2 launch limo_bringup limo_bringup_1.2.launch.py controller:=stanley
```

**or** calling directly

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

## PID Method
![PID.png](/images/1.2/PID.png)

Formula: 

$$
\delta(t) = K_p e(t) + K-i \int e(t)dt + K_d \frac{de(t)}{dt}
$$

where:

- $K_d, K_i, K_d$ are tuning parameters
- $e(t)$ is the cross track error

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

---

Method:

from [pid.py](/src/limo_controller/scripts/pid.py)

1. Define the Mode and PID Parameters

By defining the mode (car or bicycle) and the PID parameters (Kp, Ki, Kd) as configurable parameters for the controller. This allows for dynamic adjustments during runtime without having to modify the code.

```py
self.declare_parameter("mode", "car")
self.mode = self.get_parameter("mode").get_parameter_value().string_value

self.declare_parameter("Kp", 10.0)
self.declare_parameter("Ki", 1.0)
self.declare_parameter("Kd", 0.0)
self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
```

- `mode`: Defines the vehicle model used for path tracking. Can be set to "car" or "bicycle."
- `Kp`, `Ki`, `Kd`: PID controller gains for proportional, integral, and derivative terms. These are crucial for controlling the vehicle’s lateral error (cross-track error).

---

2. Set Constant Forward Velocity

Define the constant forward velocity at which the robot should travel while performing path tracking. A constant speed is typically used for simplicity and to isolate the lateral control problem.

```py
self.declare_parameter("target_speed", 2.0)
self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value
```

`target_speed`: The constant speed (in meters per second) that the robot should maintain while tracking the path.

---

3. Set Control Loop Period

The control period determines how often the PID control loop runs. It’s crucial to set this period to ensure the controller reacts quickly enough for smooth control.

```py
self.declare_parameter("control_period", 0.01)
self.control_period = self.get_parameter("control_period").get_parameter_value().double_value
```

`control_period`: The period (in seconds) between each control loop execution. A smaller value means more frequent updates, leading to more responsive control.

---

4. Compute Cross-Track Error

The core of the PID controller is to calculate the cross-track error, which is the perpendicular distance from the robot's current position to the closest point on the path. This error drives the PID correction.

```py
error, nearest_index = self.compute_cross_track_error()
```

- `error`: The cross-track error that the PID controller uses to adjust the robot's steering. Positive means the robot is to the right of the path, and negative means it's to the left.
- `nearest_index`: The index of the closest waypoint on the path, used to guide the vehicle toward the path.

---

5. PID Control Loop

```py
steering_correction = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative
```

- `steering_correction`: The calculated correction for the steering angle, which combines the proportional, integral, and derivative components based on the current error and past errors.
- `self.integral_error`: Accumulated error over time, helping to correct for biases.
- `derivative`: The rate of change of the error, used to predict the error's future behavior.

---

6. Limit Steering Command 

Limitation of the steering angle to avoid excessive corrections that might make the robot unstable.

```py
steering_correction = np.clip(steering_correction, -max_steering_rad, max_steering_rad)
```

`max_steering_rad`: The maximum allowable steering angle in radians. In this case, ±10 degrees (converted to radians).


---

## Pure pursuit Method
![Pure_Pursuit.png](/images/1.2/Pure_Pursuit.png)

Formula: 

$$
\delta(t) = \tan^{-1} \frac{2Lsin (\alpha(t))}{l_d}
$$

where:

- $L$ is the wheelbase
- $\alpha$ is the angle between the vehicle's heading and the look-ahead vector
- $l_d$ is the look-ahead distance

Our current controller for lateral control has proven to be the most successful and ideal approach so far. The results for the single-track model, yaw rate, and ground truth are nearly identical, while the double-track model shows some path deviation. This method does not include any dynamically configurable parameters, but the LIMO robot consistently follows the path while maintaining a constant distance from it.

However, there are a few limitations. At higher speeds, the controller can struggle, as the lookahead point increases and the robot may make less precise adjustments. Additionally, while the controller ensures the robot never overshoots, it tends to cut corners, which may lead to suboptimal path following in sharp turns.

---

Method:

from [pure_pursuit.py](/src/limo_controller/scripts/pure_pursuit.py)

1. Pure Pursuit State Variables

```py
self.currentPos = [0.0, 0.0]
self.currentHeading = 330
self.lastFoundIndex = 0
self.lookAheadDis = 0.8
self.linearVel = 100
```

- `currentPos`: Stores the vehicle's current position `[x, y]`.
- `currentHeading`: The current heading of the vehicle (in degrees).
- `lastFoundIndex`: Keeps track of the most recent path index used in the pure pursuit algorithm.
- `lookAheadDis`: Defines the look-ahead distance. The algorithm will search for a goal point that is `lookAheadDis` meters ahead along the path.
- `linearVel`: This is a scaling factor for velocity in simulation.

---

2. Helper Functions

```py
def sgn(self, x):
    return 1 if x >= 0 else -1

def pt_to_pt_distance(self, pt1, pt2):
    return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
```

- `sgn(x)` Returns 1 for positive numbers and -1 for negative numbers. This is useful in steering calculations.
- `pt_to_pt_distance(pt1, pt2)` computes the straight-line distance between two points (x1, y1) and (x2, y2).

3. Pure Pursuit Step

```py
def pure_pursuit_step(self):
    for i in range(self.lastFoundIndex, len(self.path)):
        if self.pt_to_pt_distance(self.currentPos, self.path[i]) >= self.lookAheadDis:
            self.lastFoundIndex = i
            break
```

- Finds the closest "goal point" that is at least `lookAheadDis` away from the current position.
- Iterates through `self.path` starting from `self.lastFoundIndex` (to avoid redundant checks).
- Once it finds a point at least `lookAheadDis` away, it updates `self.lastFoundIndex`.

---

5. Main Pure Pursuit Logic

```py
def pure_pursuit(self):
    if self.lastFoundIndex >= len(self.path) - 1:
        return
    
    goalPoint = self.path[self.lastFoundIndex]
    dx = goalPoint[0] - self.currentPos[0]
    dy = goalPoint[1] - self.currentPos[1]

    alpha = np.arctan2(dy, dx) - np.radians(self.currentHeading)
    Ld = self.pt_to_pt_distance(self.currentPos, goalPoint)

    if Ld > 0:
        turnVel = (2 * np.sin(alpha)) / Ld
    else:
        turnVel = 0

    return turnVel
```

- Finds the next goal point using `self.lastFoundIndex`.
- Computes:
`dx, dy`: Difference in `x` and `y` between current position and goal.
`alpha`: Angle error—the angle between the vehicle’s heading and the goal point direction.
`Ld` : Distance to the goal point.
- Computes turning velocity using

$$
turnVel = \frac{2sin(\alpha)}{L_d}
$$

- If `Ld` == 0 (goal already reached), sets `turnVel` = 0.

---

## Stanley Method
![Stanley.png](/images/1.2/Stanley.png)

Formula: 

$$
\delta(t) = \theta_e(t) + \tan^{-1} \left( \frac{k e_{fa}(t)}{v_x(t)} \right)
$$

where:

- $\theta_e(t)$ is the heading error
- $k$ is a gain parameter
- $e_{fa}$ is the cross-track error
- $v_x$ is the longitudinal velocity


This method exhibits the most overshooting among the controllers due to its nature of aggressively correcting heading angle errors. The Stanley controller works by minimizing both the cross-track error (the perpendicular distance to the path) and the heading error (the difference between the robot’s orientation and the desired path direction).

Because it directly adjusts the steering angle based on these errors, it can produce sharp corrections, especially when the robot is misaligned with the path. This often results in oscillations or overshooting, particularly at high speeds or when approaching the path from an angle. However, its strength lies in its ability to converge to the path robustly, making it effective for precise tracking when tuned properly.

One of its key limitations is that at very low speeds, the control gains can cause excessive steering adjustments, leading to unstable behavior. Conversely, at high speeds, the controller can struggle with aggressive corrections, making it sensitive to tuning and potentially causing instability.

---

Method:

from [stanley.py](/src/limo_controller/scripts/stanley.py)

1. Compute the front axle position using the vehicle's current state

The Stanley controller operates based on the front axle position, not the vehicle's center. This helps in accurately computing the cross-track error.

```py
fx = currentPos[0] + self.L * math.cos(yaw)
fy = currentPos[1] + self.L * math.sin(yaw)
```

- `currentPos[0]` and `currentPos[1]` represent the rear axle position in (x, y).
- `yaw` is the vehicle's heading angle (converted from quaternion).
- `self.L` is the wheelbase, the distance from the rear to the front axle.
- `fx` and `fy` compute the front axle's x, y position using basic trigonometry.

---

2. Find the closest path point using Euclidean distance

The vehicle needs to determine which point on the reference path it should follow. The closest point is the best target to minimize error.

```py
dx = [fx - icx for icx in cx]
dy = [fy - icy for icy in cy]
d = np.hypot(dx, dy)  
target_idx = int(np.argmin(d))
```

- `cx` and `cy` are arrays of x and y coordinates from the reference path.
- `dx` and `dy` are difference vectors between the front axle and all path points.
- `np.hypot(dx, dy)` computes the Euclidean distance to each point.
- `np.argmin(d)` finds the index of the closest path point.

---

3. Compute the cross-track error

The cross-track error (CTE) is the perpendicular distance between the front axle and the reference path. This is a key term in the Stanley control law.

```py
error_x = cx[target_idx] - fx
error_y = cy[target_idx] - fy
error_front_axle = error_x * (-math.sin(yaw)) + error_y * math.cos(yaw)
```

- `error_x` and `error_y` compute the difference between the front axle position and the closest path point.
- The cross-track error is projected perpendicularly onto the vehicle's local heading using

$$
CTE = error_x \cdot (-sin(yaw)) + error_y \cdot cos(yaw)
$$

- This ensures that positive error means the vehicle is to the right of the path, and negative error means it's to the left.

---

4. Compute the heading error

The vehicle's yaw may not be aligned with the reference path, so we need to compute the heading error and correct for it.

```py
theta_e = self.normalize_angle(cyaw[target_idx] - yaw)
```

- `cyaw[target_idx]` is the desired heading at the closest path point.
- `yaw`  is the vehicle's actual heading.
- `theta_e` is the heading error.
- The function `normalize_angle(angle)` ensures the error is in the range [-π, π], preventing wraparound issues.

---

5. Use the Stanley control law to calculate the steering angle

The Stanley formula combines the heading error and cross-track error to compute a steering command that minimizes both.

```py
theta_d = math.atan2(self.k * error_front_axle, speed)
delta = theta_e + theta_d
```

- The steering angle consists of:

$$
\delta = \theta_e + tan^{-1} (\frac{k \cdot CTE}{speed})
$$

- `theta_d` is the correction based on cross-track error, scaled by self.k (a tuning parameter).
- At higher speeds, `theta_d` has less influence to prevent oversteering.
- At lower speeds, `theta_d` has more influence to ensure tight cornering.
- The final `delta` steering angle is a sum of heading and cross-track corrections.

---

### Summary

In evaluating the PID, Stanley, and Pure Pursuit controllers for trajectory tracking, I analyzed their impact on key parameters: trajectory adherence, yaw heading stability, linear velocity consistency, and angular velocity responsiveness. The PID controller demonstrated smooth and stable performance but required careful tuning to balance responsiveness and overshoot. The Stanley method excelled in minimizing cross-track error, particularly at lower speeds, but exhibited aggressive corrections at higher speeds. Pure Pursuit, leveraging geometric path tracking, offered intuitive curvature-based steering but struggled with accuracy at sharp turns. Each method has trade-offs, with PID favoring stability, Stanley optimizing path adherence, and Pure Pursuit excelling in dynamic maneuvering. Controller selection depends on specific application requirements, balancing precision, responsiveness, and robustness.

---

## Lab 1.3 - State Estimator

- [Part 1](#lab-13-part-1)
- [Part 2](#lab-13-part-2)

choose one of the following controllers (pure pursuit recommended)

```shell
ros2 launch limo_bringup limo_bringup_1.3.launch.py controller:=pid
```

```shell
ros2 launch limo_bringup limo_bringup_1.3.launch.py controller:=pure_pursuit
```

```shell
ros2 launch limo_bringup limo_bringup_1.3.launch.py controller:=stanley
```
run the GPS emulator from [gps emulator](/src/limo_controller/scripts/gps_emulator.py)

```shell
ros2 run limo_controller gps_emulator.py
```

---

## Lab 1.3 part 1

from [EKF Localization](https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html) document, [gps emulator](/src/limo_controller/scripts/gps_emulator.py) & [gps plot](/src/limo_results/scripts/limo_plot_gps.py) 

![gps_emulator.png](/images/1.3/GPS_Emulator.png)

We can simulate GPS position by inducing noise to the ground truth path.

---

from [EKF](/src/limo_controller/scripts/ekf.py) the adjustable parameters are the Process Noise Covariance (matrix Q) and Measurement Noise Covariance (matrix R)

**Process Noise Covariance (Q)**
- Represents the uncertainty in the system's motion model.
- Accounts for errors from unmodeled dynamics, system imperfections, or external disturbances.

from this code;
```py
proc_noise = np.array(self.get_parameter('process_noise').value) ** 2  # square to variance
self.Q = np.diag(proc_noise)
```
the SD value got squared resulted in **variance** which then got converted into matrix.

example Q matrix:
```py
Q = |  1.0      0       0       0  |
    |  0      1.0       0       0  |
    |  0       0   (1°)^2    0  |
    |  0       0       0     1.0  |
```

- Large values → higher uncertainty in prediction.
- Small values → more confidence in the motion model.

---

**Measurement Noise Covariance (R)**

- Represents uncertainty in sensor measurements.
- Squaring ensures values represent variance.
- Each sensor has its own R matrix, defined separately as shown:

```python
double_noise = np.array(self.get_parameter('R_double_track').value) ** 2
single_noise = np.array(self.get_parameter('R_single_track').value) ** 2
yaw_noise    = float(self.get_parameter('R_yaw_rate').value) ** 2
gps_noise    = np.array(self.get_parameter('R_gps').value) ** 2
```

example R_Double matrix:

```py
R_double = | (10m)^2    0         0     |
           |    0   (10m)^2       0     |
           |    0       0    (500°)^2  |

R_single = same structure as R_double

R_yaw = | (1°)^2 |

R_gps = | (1m)^2    0   |
        |   0    (1m)^2 |
```

### Key Takeaways:
- Q (Process Noise): Controls how much the system trusts its motion model.
- R (Measurement Noise): Controls how much the system trusts sensor readings.
- Tuning: If the filter is too slow to update, increase Q or decrease R; if it's too noisy, decrease Q or increase R.

## Result

- Green line = Path
- Green arrow = Ground Truth
- Blue arrow = Yaw rate
- purple arrow = Single Track

![noise_ekf.png](/images/1.3/Noise_EKF.png)

With high Q value we can clearly see that the filtered odom became very noisy (black arrow)

![emulator_activated.png](/images/1.3/Emulator_activated.png)

When the GPS was activated, the EKF odometry adjusted to a more accurate position, as shown. This updated position can then serve as the new reference, replacing the Ground Truth, which is unavailable in real-world scenarios.

---

## Lab 1.3 part 2
