## Description:
This repository contains mpc implementation of motion planning for omni-wheel drive robot, with focus on RoboCup MSL Competition.

### Input:
1. o1_data = {x, y, theta, enc0, enc1, enc2, enc3} : current state of o1
2. /o1/target_pos = {x, y, theta, vx, vy, omega, speed, nearest_point_idx, path_size} : next target state of o1

### Output:
1. /o1/cmd_vel = {vx, vy, omega} : next predicted velocities of o1

## Dependencies:
1. Eigen3:
```bash
sudo apt-get update
sudo apt-get install libeigen3-dev
```

2. Boost:
```bash
sudo apt-get install libboost-all-dev
```

3. Acado:
```bash
git clone https://github.com/acado/acado.git
cd acado
mkdir build
cd build
cmake ..
make
sudo make install
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```
You may follow acado/examples/getting_started for examples of Acado library

## Build and run the code:
```bash
cd model_predictive_control
colcon build
source install/local_setup.bash
ros2 run controller_package controller
```
