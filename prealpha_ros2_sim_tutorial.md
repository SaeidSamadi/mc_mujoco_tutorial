## Running the RB-Y1 Simulation with `prealpha_ros2`

This guide walks through launching the RB-Y1 robot simulation using Docker for the simulator and `prealpha_ros2` (via Pixi) for the controller interface and visualization. It assumes you've already built the workspace.

> **Important:** All commands below must be run from the **`prealpha_ros2` root directory**.

### 0. Remove Gripper dependencies for simulations

first of all you need to comment out locally the gripper controllers, here:

https://github.com/HumanoidTeam/prealpha_ros2/blob/931c31ed646e0e6772e210408d99d82a848552d3/prealpha_description/control/real.ros2_control.xacro#L26-L40

---

### 1. Start the Simulator (MuJoCo in Docker)

```bash
pixi run sim_up
```

This starts the MuJoCo-based simulator from `third-party/rby1/rby1-sdk` and opens the simulation GUI. The gRPC server listens on `0.0.0.0:50051`.

> If this is your first time, Docker may take a few minutes to pull the image.

---

### 2. Start the ROS2 Controller Interface

Open a new terminal in `prealpha_ros2` and run:

```bash
pixi run sim_control
```

This launches the `ros2_control` node and connects it to the simulator.

---

### 3. Launch RViz to Visualize the Robot

In a separate terminal:

```bash
pixi run sim_rviz
```

You’ll see the RB-Y1 robot rendered in RViz. Use the default config file for full visibility.

---

### 4. Send a Test Command to Move the Robot

In a new terminal:

```bash
pixi run sim_test
```

This publishes a joint position command to the robot’s arms via the `forward_position_controller`.
