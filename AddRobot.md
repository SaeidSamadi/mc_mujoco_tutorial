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
