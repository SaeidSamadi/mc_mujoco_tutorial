# mc_rtc Installation

### **1. Install `mc_rtc` Using the Superbuild Script**

#### **GitHub Repository:**
[mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild)

Clone it to the mc_rtc folder (preference)

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
Also add the following line to `.bashrc`:

```bash
source ~/devel/src/catkin_data_ws/devel/setup.bash
```

#### **Notes:**
- This will install in `/home/sasa/devel/build/mc_rtc/`.
- To install everything in `/usr/local/`, use the second command.
- If you are inside `mc-rtc-superbuild/build`, you can run the commands there directly.

---

### **2. Uninstall System-Wide `mc_rtc` (If Required)**
If an older version of `mc_rtc` is installed system-wide, remove it:
```bash
sudo rm -rf /usr/local/lib/libmc_rtc*
sudo rm -rf /usr/local/include/mc_rtc
sudo rm -rf /usr/local/share/mc_rtc
sudo rm -rf /usr/local/lib/cmake/mc_rtc
sudo rm -rf /usr/local/lib/python3.10/dist-packages/mc_rtc*
```

In case of python-related erros like

```bash
ninja: build stopped: subcommand failed.
CMake Error at /home/noodles/Software/mc_rtc/mc-rtc-superbuild/cmake/scripts/cmake-with-prefix.cmake:22 (execute_process):
  execute_process failed command indexes:

    1: "Child return code: 1"



gmake[2]: *** [CMakeFiles/Eigen3ToPython.dir/build.make:89: prefix/Eigen3ToPython/src/Eigen3ToPython-stamp/Eigen3ToPython-build] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:2264: CMakeFiles/Eigen3ToPython.dir/all] Error 2
```

you can use env:

## ✅ Full Steps to Build `mc_rtc` with `mc_env`

---

### 🧱 1. Create and Activate Your Virtual Environment

```bash
python3 -m venv ~/mc_env
source ~/mc_env/bin/activate
```

---

### 📦 2. Install Required Python Packages

Inside the activated `mc_env`:

```bash
pip install --upgrade pip
pip install numpy scipy cython pythran
```

> ✅ This ensures consistent versions for Python bindings and Cython compilation.

---

### 3. Reinstall



---

# **Install MuJoCo 3.3.0**

### **🚀 Install MuJoCo 3.3.0 & Modify `mc_mujoco` Installation Steps**
Since the **latest MuJoCo release is 3.3.0**, we will install this version and update the installation steps for `mc_mujoco` accordingly.


### **🔹 Step 1: Create the MuJoCo Directory**
First, create the target directory for MuJoCo:
```bash
mkdir -p ${HOME}/.mujoco/mujoco_3.3.0
cd ${HOME}/.mujoco
```

---

### **🔹 Step 2: Download MuJoCo 3.3.0**
Download the latest release from DeepMind’s GitHub:
```bash
wget https://github.com/deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-linux-x86_64.tar.gz -O mujoco-3.3.0.tar.gz
```

---

### **🔹 Step 3: Extract MuJoCo**
Extract the archive to your MuJoCo directory:
```bash
tar -xvzf mujoco-3.3.0.tar.gz -C ${HOME}/.mujoco/mujoco_3.3.0 --strip-components=1
```
This ensures the extracted files go **directly into `mujoco_3.3.0`** without an extra folder.

---

### **🔹 Step 4: Set Up Environment Variables**
Add MuJoCo to your system paths by modifying your `~/.bashrc`:
```bash
echo 'export MUJOCO_PATH=${HOME}/.mujoco/mujoco_3.3.0' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=${MUJOCO_PATH}/bin:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PATH=${MUJOCO_PATH}/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

---

### **🔹 Step 5: Install Required Dependencies**
Ensure all required libraries are installed:
```bash
sudo apt update
sudo apt install libgl1 libosmesa6 libegl1 libglew-dev patchelf
```

---

### **🔹 Step 6: Verify Installation**
Check if MuJoCo is correctly installed:
```bash
ls ${MUJOCO_PATH}
```
You should see **directories like `bin/`, `include/`, `lib/`, `model/`**.

To test MuJoCo, try:
```bash
mujoco_gl
```
If a visualization window appears, **MuJoCo is installed successfully!** ✅

---

# **Install `mc_mujoco` with MuJoCo 3.3.0**

Now that MuJoCo is installed under **`${HOME}/.mujoco/mujoco_3.3.0`**, modify the `mc_mujoco` installation steps accordingly.

Or you can test the version and location first:

```bash
python3 -c "import mujoco; print(mujoco.__version__)"
python3 -c "import mujoco; print(mujoco.__file__)"
```

### **🔹 Step 1: Clone `mc_mujoco` Repository**
```bash
git clone --recursive git@github.com:rohanpsingh/mc_mujoco.git
cd mc_mujoco
```

---

### **🔹 Step 2: Create Build Directory**
```bash
mkdir build && cd build
```

---

### **🔹 Step 3: Run `CMake` with the Correct MuJoCo Path**
Modify the `MUJOCO_ROOT_DIR` to use **MuJoCo 3.3.0**:
```bash
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=${HOME}/.mujoco/mujoco_3.3.0
```

---

### **🔹 Step 4: Compile and Install**
```bash
make -j$(nproc)
make install
```

---

### **🔹 Step 5: Add Environment Variables for `mc_mujoco`**
Update your `~/.bashrc` to include `mc_mujoco` paths:
```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/.mujoco/mujoco_3.3.0/lib:${HOME}/.mujoco/mujoco_3.3.0/bin' >> ~/.bashrc
source ~/.bashrc
```

The `.bashrc` should look like:

```bash
# Set MuJoCo Path
export MUJOCO_PATH=${HOME}/.mujoco/mujoco_3.3.0
export LD_LIBRARY_PATH=${MUJOCO_PATH}/bin:${MUJOCO_PATH}/lib:$LD_LIBRARY_PATH
export PATH=${MUJOCO_PATH}/bin:$PATH

# Source ROS Noetic
source /opt/ros/noetic/setup.bash

# Source Catkin Workspace (ROS Packages)
source ~/devel/src/catkin_data_ws/devel/setup.bash

```

---

### **🔹 Step 6: Run `mc_mujoco`**
```bash
mc_mujoco
```

---

## **Adding Objects in `mc_mujoco`**

Here I try to add Unitree G1 as a test.

You're right! **`unitree_legged_sdk` does not contain the URDF** files directly. Instead, Unitree provides the URDF in a separate repository. Let’s switch to a **confirmed source** and get a robot that we can quickly run in `mc_mujoco`.  

---

## **🚀 Let's Use the Unitree Go1 URDF for mc_mujoco**
Since **Unitree Go1 (not G1)** has an openly available URDF, we will use that instead.

### **✅ Step 1: Clone the Correct Unitree Repository**
```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots_description/urdf
```

---

### **✅ Step 2: Check for URDF Files**
```bash
cd robots/
```
You should see robot description files.

Awesome! You have **Unitree G1** URDFs in different configurations. Let's pick one and convert it to **MuJoCo format** for `mc_mujoco`.

---

## **🚀 Step 1: Choose a Suitable URDF**
From your list, these look like good candidates:
- `g1_29dof.urdf` (29 degrees of freedom)
- `g1_23dof.urdf` (simpler version)

For now, let's use **`g1_29dof.urdf`** since it's likely the most detailed version.

---

## **✅ Step 2: Convert URDF to MuJoCo XML**
Run:
```bash
urdf2mjcf g1_29dof.urdf --output g1_mujoco.xml
```

If the conversion works without errors, you should get `g1_mujoco.xml`.

---

## **✅ Step 3: Integrate G1 into mc_mujoco**
1️⃣ **Copy the converted MuJoCo XML file to `mc_mujoco`'s robot folder**:
```bash
mkdir -p ~/.config/mc_mujoco
cp g1_mujoco.xml ~/.config/mc_mujoco/g1.xml
```

2️⃣ **Create a new mc_rtc robot configuration file**:
```bash
mkdir -p ~/.config/mc_rtc/robots/
nano ~/.config/mc_rtc/robots/g1.yaml
```

Inside `g1.yaml`, add:
```yaml
robot_module:
  name: "g1"
  urdf_path: "/home/noodles/robots/unitree_ros/robots/g1_description/g1_29dof.urdf"
  base_link: "base_link"
```
Modify the path if necessary.

---

## **✅ Step 4: Run G1 in mc_mujoco**
Launch `mc_mujoco` with Unitree G1:
```bash
mc_mujoco --robot g1
```

---

## **🔹 What If the URDF Conversion Fails?**
If `urdf2mjcf` gives errors (like missing meshes), try:
```bash
find ~/robots/unitree_ros/robots/g1_description/ -name "*.stl"
```
If the mesh files exist but are missing in the URDF, you may need to **update the URDF paths manually** before converting.

---

## **🌟 Summary**
✔ You have **Unitree G1 URDF**  
✔ Convert it to **MuJoCo XML**  
✔ Copy and configure for **mc_mujoco**  
✔ Run it in `mc_mujoco`  

🚀 **Let me know if you hit any issues during conversion!**

# In case of error in creating g1_mujoco.xml file:

That's because of How the STL Files Are Referenced in the URDF. Run this command to inspect the file paths inside your g1_29dof.urdf:

```bash
grep -i "meshes" ~/robots/unitree_ros/robots/g1_description/g1_29dof.urdf
```
You might see something like:

```xml
<mesh filename="meshes/pelvis_contour_link.STL"/>
```

or

```xml
<mesh filename="package://g1_description/meshes/pelvis_contour_link.STL"/>
```

If it uses relative paths like meshes/filename.STL, urdf2mjcf might not know where to look.

If that's the case, instead we should use absolute directories.

```xml
<mesh filename="/home/noodles/robots/unitree_ros/robots/g1_description/meshes/pelvis_contour_link.STL"/>
```



=======================

### **Steps to Add a New Object (e.g., Talos)**

#### **1. Clone the Object Model Files**
Clone the object model files (including `model.xml`) into the robot model directory:
```bash
cd ~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/
git clone https://github.com/mujoco_menagerie/pal_talos.git
```
Expected directory:
```
~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/pal_talos/
```

#### **2. Create the `talos.in.yaml` File**
Navigate to the same directory as the XML model and create a `.yaml` config:
```bash
nvim ~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/pal_talos/talos.in.yaml
```
Paste the following content:
```yaml
xmlModelPath: "@MC_MUJOCO_SHARE@/mujoco_menagerie/pal_talos/talos.xml"
```

#### **3. Register Talos in `mc_mujoco/robots/CMakeLists.txt`**
Modify `mc_mujoco/robots/CMakeLists.txt` to include Talos:
```bash
nvim ~/Softwares/mc_rtc/mc_mujoco/robots/CMakeLists.txt
```
Add Talos to the `setup_env_object` list:
```cmake
setup_env_object(talos robot)
```
Ensure the correct repository address is used.

#### **4. Link the Talos Robot Model into `mc_rtc/robots/`**
Since `mc_rtc` also needs access to Talos, create a symbolic link:
```bash
ln -s ~/Softwares/mc_rtc/mc_mujoco/robots/mujoco_menagerie/pal_talos ~/Softwares/mc_rtc/robots/talos
```

Also, create a `CMakeLists.txt` file inside `robots/talos/`.

#### **5. Rebuild `mc_mujoco` to Include Talos**
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

#### **7. Run Talos in `mc_mujoco`**
Finally, execute the following command to run Talos:
```bash
mc_mujoco --mc-config ~/Softwares/mc_rtc/mc_mujoco/config/talos_config.yaml
```

✅ **Talos should now be successfully loaded in `mc_mujoco`.** 🚀

