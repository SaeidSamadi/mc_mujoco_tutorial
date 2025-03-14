# Adding New Robot to mc_rtc (Steps)

### It should have a description
- this one needs to be added to the catkin_data_ws
- catkin (colcon) build
### Another file is a created mc_robot
- This need to be built after description file
- it can be anywhere, just need to cm-m-smi it locally
- For not having errors, we need to source the `../install/setup.bash` in the place where the installed folder of description file exists. For instance:
```sh
source ~/devel/src/catkin_data_ws/src/install/setup.bash
```

# **Steps to Add a New Humanoid to `mc_rtc` and Simulate It**

## **1. Create the Robot Description Package**
- Create a ROS2 package for the robot:
  ```bash
  cd ~/ros2_ws/src
  ros2 pkg create my_robot_description
  ```
- Add **URDF/Xacro model** and required meshes (`.stl`, `.dae`).
- Define **surfaces** needed for `mc_rtc` (grippers, feet, etc.).
- Ensure the package is properly structured and build it.

## **2. Create the Robot Module**
- Create a new `mc_rtc` module:
  ```bash
  cd ~/mc_rtc_ws/src
  mkdir mc_my_robot
  ```
- Implement:
  - `MyRobot.cpp` (defines the robot module)
  - `MyRobot.h` (header)
  - `my_robot.json` (joint order, sensors, etc.)
- Modify `CMakeLists.txt` to compile the module.
- Build and install:
  ```bash
  colcon build --packages-select mc_my_robot
  ```

## **3. Integrate with a Simulator**
### **MuJoCo**
- Convert URDF to MuJoCo XML:
  ```bash
  python3 urdf2mujoco.py my_robot.urdf -o my_robot.xml
  ```
- Edit the XML to add actuators.
- Run MuJoCo with `mc_rtc`:
  ```bash
  ros2 launch mc_mujoco mujoco.launch.py model:=my_robot.xml
  ```

### **Gazebo (via ros2_control)**
- Modify URDF to include:
  ```xml
  <gazebo>
    <plugin name="ros2_control" filename="libgazebo_ros2_control.so"/>
  </gazebo>
  ```
- Start Gazebo:
  ```bash
  ros2 launch gazebo_ros gazebo.launch.py
  ```
- Run `mc_rtc`:
  ```bash
  ros2 run mc_rtc_ticker mc_rtc_ticker --ros-args -p robot:=MyRobot
  ```

## **4. Test and Debug**
- Check URDF:
  ```bash
  check_urdf my_robot.urdf
  ```
- Verify `mc_rtc` is running:
  ```bash
  ros2 run mc_rtc_rviz_panel mc_rtc_rviz_panel
  ```
- Debug missing topics:
  ```bash
  ros2 topic list | grep mc_rtc
  ```

This provides a clear, systematic workflow. Let me know if anything needs refinement. 🚀
