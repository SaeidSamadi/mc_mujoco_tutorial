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

---

### **🔹 Step 6: Run `mc_mujoco`**
```bash
mc_mujoco
```








### **3. Install `mc_mujoco`**

#### **GitHub Repository:**
[mc_mujoco](https://github.com/rohanpsingh/mc_mujoco)

#### **Installation Steps:**
Run the following command in the `mc_mujoco` build directory:
```bash
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=/home/sasa/Softwares/mujoco/mujoco-3.2.7 -Dmc_rtc_DIR=/home/sasa/devel/install/share/mc_rtc/cmake
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

## **Adding Objects in `mc_mujoco`**

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

