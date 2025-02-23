# LAB1_WS – Mobile Robot Simulation

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