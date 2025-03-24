## Running the RB-Y1 Simulation with `prealpha_ros2`

This guide walks through launching the RB-Y1 robot simulation using Docker for the simulator and `prealpha_ros2` for the controller interface and visualization. It assumes you've already built the workspace and installed required dependencies.

--- 

### 0. Remove Gripper dependencies for simulations

first of all you need to comment out locally the gripper controllers, here:

https://github.com/HumanoidTeam/prealpha_ros2/blob/931c31ed646e0e6772e210408d99d82a848552d3/prealpha_description/control/real.ros2_control.xacro#L26-L40

---

### 1. Launch the Simulator

Navigate to the `rby1-sdk` directory inside `prealpha_ros2/third-party`:

```bash
cd third-party/rby1/rby1-sdk
xhost +local:docker
docker-compose -f docker-compose.sim.yaml up rby1-sim
```

> This command starts the MuJoCo-based simulator and binds to port `50051` on `0.0.0.0`.

---

### 2. Launch the ROS2 Controller

In a **new terminal**, activate the `pixi` shell in the root of the `prealpha_ros2` repository:

```bash
cd prealpha_ros2
pixi shell
ros2 launch prealpha_description controllers_bringup.launch.py hardware_type:=real robot_ip:=0.0.0.0:50051
```

---

### 3. Visualize the Robot in RViz

In another terminal (from `prealpha_ros2` root):

```bash
pixi run ros2 run rviz2 rviz2 -d $(pixi run ros2 pkg prefix --share prealpha_description)/rviz/config.rviz
```

---

### 4. Send a Test Command to the Robot

In a new terminal, you can send a sample joint position command to the active controller:

```bash
pixi run ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.2, -0.3, 0.1, 0.0, 0.0, 0.1, 0.0, 0.2, -0.3, 0.1, 0.0, 0.0, 0.1, 0.0]"
```

> This sends 14 joint positions, assuming 7 joints per arm (left and right).
