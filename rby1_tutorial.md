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



with pixi shell


Awesome Saeid — now that everything works via Pixi, here’s your **updated, clean, and modernized tutorial** using the `pixi run -e sim` commands.

---

## ✅ Running the RB-Y1 Simulation with `prealpha_ros2`

This guide walks through launching the RB-Y1 robot simulation using Docker for the simulator and `prealpha_ros2` (via Pixi) for the controller interface and visualization. It assumes you've already built the workspace.

> ✅ **Important:** All commands below must be run from the **`prealpha_ros2` root directory**.

---

### 1. 🧠 Start the Simulator (MuJoCo in Docker)

```bash
pixi run -e sim sim_up
```

This starts the MuJoCo-based simulator from `third-party/rby1/rby1-sdk` and opens the simulation GUI. The gRPC server listens on `0.0.0.0:50051`.

> If this is your first time, Docker may take a few minutes to pull the image.

---

### 2. 🧠 Start the ROS2 Controller Interface

Open a new terminal in `prealpha_ros2` and run:

```bash
pixi run -e sim sim_control
```

This launches the `ros2_control` node and connects it to the simulator.

---

### 3. 🧠 Launch RViz to Visualize the Robot

In a separate terminal:

```bash
pixi run -e sim sim_rviz
```

You’ll see the RB-Y1 robot rendered in RViz. Use the default config file for full visibility.

---

### 4. 🧠 Send a Test Command to Move the Robot

In a new terminal:

```bash
pixi run -e sim sim_test
```

This publishes a joint position command to the robot’s arms via the `forward_position_controller`.

---

### 💡 Tips

- You can also enter the simulation environment shell for manual commands:

```bash
pixi shell -e sim
# then you can run: ros2 topic list, ros2 control list_controllers, etc.
```

- If RViz doesn’t show the full robot, double-check that the simulator and controllers are both running properly.

---

Let me know if you want this formatted as a `README.md` section too — it’s clean and ready to share with your team!
