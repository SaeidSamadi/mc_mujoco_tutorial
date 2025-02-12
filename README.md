# mc\_mujoco\_tutorial

## **Instructions**

### **1. Install ********`mc_rtc`******** Using the Superbuild Script**

#### **GitHub Repository:**

[mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild)

#### **Installation Steps:**

Run the following command in the `mc_rtc` directory:

```bash
cmake -S mc-rtc-superbuild -B mc-rtc-superbuild/build -DSOURCE_DESTINATION=${HOME}/devel/src -DBUILD_DESTINATION=${HOME}/devel/build
```

Alternatively, for installation in `/usr/local/`, use:

```bash
cmake -S mc-rtc-superbuild -B mc-rtc-superbuild/build
```

Then, build with:

```bash
cmake --build mc-rtc-superbuild/build --config RelWithDebInfo
```

#### **Notes:**

- This will install in `/home/sasa/devel/build/mc_rtc/`.
- To install everything in `/usr/local/`, use the second command.
- If you are inside `mc-rtc-superbuild/build`, you can run the commands there directly.

---

### **2. Uninstall System-Wide ********`mc_rtc`******** (If Required)**

If an older version of `mc_rtc` is installed system-wide, remove it:

```bash
sudo rm -rf /usr/local/lib/libmc_rtc*
sudo rm -rf /usr/local/include/mc_rtc
sudo rm -rf /usr/local/share/mc_rtc
sudo rm -rf /usr/local/lib/cmake/mc_rtc
sudo rm -rf /usr/local/lib/python3.10/dist-packages/mc_rtc*
```

---

### \*\*3. Install \*\***`mc_mujoco`**

#### **GitHub Repository:**

[mc\_mujoco](https://github.com/rohanpsingh/mc_mujoco)

#### **Installation Steps:**

Run the following command in the `mc_mujoco` build directory:

```bash
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/home/sasa/Softwares/mujoco/mujoco-3.2.7 -DCMAKE_PREFIX_PATH=/home/sasa/devel/src/catkin_data_ws/install/share/mc_rtc_msgs/cmake
```

If you face issues, the following worked at a later attempt:

```bash
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DMUJOCO_ROOT_DIR=/home/sasa/Softwares/mujoco/mujoco-3.2.7 \
      -Dmc_rtc_DIR=/home/sasa/devel/install/share/mc_rtc/cmake ..
```

To debug if `mc_rtc` is installed correctly, run:

```bash
find ~/devel/install -name "mc_rtcConfig.cmake"
```

Expected output:

```
/home/sasa/devel/install/lib/cmake/mc_rtc/mc_rtcConfig.cmake
```

If the address is different, update the `CMAKE_PREFIX_PATH` accordingly.

---

## \*\*Adding Objects in \*\***`mc_mujoco`**

### **Steps to Add a New Object (e.g., Talos)**

#### **1. Clone the Object Model Files**

Clone the object model files (including `model.xml`) into the robot model directory:

```bash
cd ~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/
git clone https://github.com/some_repo/pal_talos.git
```

Expected directory:

```
~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/pal_talos/
```

#### **2. Create the ********`talos.in.yaml`******** File**

Navigate to the same directory as the XML model and create a `.yaml` config:

```bash
nvim ~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/pal_talos/talos.in.yaml
```

Paste the following content:

```yaml
xmlModelPath: "@MC_MUJOCO_SHARE@/mujoco_menagerie/pal_talos/talos.xml"
```

#### \*\*3. Register Talos in \*\***`mc_mujoco/robots/CMakeLists.txt`**

Modify `mc_mujoco/robots/CMakeLists.txt` to include Talos:

```bash
nvim ~/Softwares/mc_rtc/mc_mujoco/robots/CMakeLists.txt
```

Add Talos to the `setup_env_object` list:

```cmake
setup_env_object(mujoco_menagerie/pal_talos/talos robot)
```

Ensure the correct repository address is used.

#### \*\*4. Link the Talos Robot Model into \*\***`mc_rtc/robots/`**

Since `mc_rtc` also needs access to Talos, create a symbolic link:

```bash
ln -s ~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/pal_talos ~/Softwares/mc_rtc/robots/talos
```

Also, create a `CMakeLists.txt` file inside `robots/talos/`.

#### **5. Rebuild ********`mc_mujoco`******** to Include Talos**

Recompile `mc_mujoco` to apply the changes:

```bash
cd ~/Softwares/mc_rtc/mc_mujoco/build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
         -DMUJOCO_ROOT_DIR=/home/sasa/Softwares/mujoco/mujoco-3.2.7 \
         -Dmc_rtc_DIR=/home/sasa/devel/install/share/mc_rtc/cmake
make -j$(nproc)
make install
```

#### **6. Build a Configuration File for Talos**

Create a configuration file:

```bash
nvim ~/Softwares/mc_rtc/mc_mujoco/config/talos_config.yaml
```

Refer to an existing config file for structure.

#### \*\*7. Run Talos in \*\***`mc_mujoco`**

Finally, execute the following command to run Talos:

```bash
mc_mujoco --config ~/Softwares/mc_rtc/mc_mujoco/config/talos_config.yaml
```

✅ **Talos should now be successfully loaded in ********`mc_mujoco`********.** 🚀



