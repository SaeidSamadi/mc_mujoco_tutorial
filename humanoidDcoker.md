## 🚀 RB-Y1 Docker Simulator – Setup Tutorial (Working Version)

---

### 🧱 1. Install System Dependencies (Ubuntu 22.04)

```bash
sudo apt install -y build-essential git python3-pip pkg-config mesa-utils libglew-dev libglfw3 libglfw3-dev libsqlite3-dev qrencode libpng-dev libfreetype6-dev can-utils
```

> 🛠️ Optional (only if needed for GUI/GL errors):
```bash
sudo apt install -y libxcb-util-dev libxcb-util0-dev libx11-dev libx11-xcb-dev libfontenc-dev libice-dev libsm-dev libxau-dev libxaw7-dev libxcomposite-dev libxdamage-dev libxkbfile-dev libxmuu-dev libxres-dev libxtst-dev libxcb-glx0-dev libxcb-render0-dev libxcb-render-util0-dev libxcb-xkb-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-dri3-dev libxcb-cursor-dev libxcb-dri2-0-dev libxcb-present-dev libxcb-composite0-dev libxcb-ewmh-dev libxcb-res0-dev
```

---

### 🐍 2. Setup Python Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install cmake conan
conan profile detect --force
```

---

### 📦 3. Clone and Prepare HumanoidTeam rby1 Repo

```bash
git clone git@github.com:HumanoidTeam/rby1.git
cd rby1
git submodule set-url rby1-web git@github.com:HumanoidTeam/rby1-web.git
git submodule update --init --recursive
```

---

### 🏗️ 4. Build System Application

```bash
conan install . -s build_type=Release -b missing -of build
cmake --preset conan-release -D BUILD_EXAMPLES=ON
cmake --build --preset conan-release
```

✅ If successful, built binaries will be in:
```bash
./build/build/Release/_output/
```

---

### 📁 5. Prepare Files for Docker Build

```bash
mkdir -p output/app
cp -r ./build/build/Release/_output/* output/app
cp third-party/mujoco/lib/libmujoco.so.3.2.0 output/app
```

---

### 🐳 6. Build Docker Container

#### For x86:
```bash
docker build -t rby1 -f docker/sim.dockerfile .
```

#### For arm64 (Apple Silicon or Jetson):
```bash
docker build -t rby1 -f docker/sim.arm64.dockerfile .
```

---

### 🚀 7. Run the Docker Simulator

```bash
xhost +
docker run --rm -it \
 -e DISPLAY=${DISPLAY} \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -p 50051:50051 \
 --name=rby1-sim \
 rby1
```

✅ MuJoCo window should appear with RB-Y1 loaded.

---

### 🧪 8. Test Simulation with Rainbow SDK Example (Optional)

If you have the official Rainbow SDK (`~/rby1-sdk`) built:

```bash
cd ~/rby1-sdk/build/examples/cpp
./example_demo_motion 127.0.0.1:50051
```

✅ Robot should move inside MuJoCo simulator.

---

# Editing locally and adding to docker

## 🧪 Tutorial: How to See Your Local `rby1-sdk` Edits in Docker Simulation

### ✅ Use Case:
You're working in a feature branch (e.g., `add-robotiq-gripper`) and want to ensure that your **local changes** (like editing `model_act.xml`) appear in the **Dockerized simulation environment**.

---

### 🔁 Step-by-Step Workflow

#### **1. Make changes in your `rby1-sdk` submodule**
```bash
cd ~/Software/humanoidDockers/rby1/rby1-sdk
git checkout add-robotiq-gripper  # or create/switch to your working branch

# Make your edits, e.g.:
nano models/rby1a/mujoco/model_act.xml

git add .
git commit -m "Updated model_act.xml with gripper components"
git push
```

---

#### **2. Point the main `rby1` repo to the updated submodule commit**
```bash
cd ~/Software/humanoidDockers/rby1
git checkout add-robotiq-gripper  # Make sure you're on the same working branch

git add rby1-sdk
git commit -m "Update rby1-sdk submodule pointer with new changes"
git push
```

> 💡 This step ensures that the Docker build will include your **latest changes** in `rby1-sdk`.

---

#### **3. Rebuild the app and Docker image**
```bash
source venv/bin/activate

conan install . -s build_type=Release -b missing -of build
cmake --preset conan-release -D BUILD_EXAMPLES=ON
cmake --build --preset conan-release

mkdir -p output/app
cp -r ./build/build/Release/_output/* output/app
cp third-party/mujoco/lib/libmujoco.so.3.2.0 output/app

docker build -t rby1 -f docker/sim.dockerfile .
```

---

#### **4. Launch the Docker simulator**
```bash
xhost +
docker run --rm -it \
 -e DISPLAY=${DISPLAY} \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -p 50051:50051 \
 --name=rby1-sim \
 rby1
```

> 🟢 You should now **see your changes reflected** inside the simulator (e.g., updated models, new geometries, etc.).

---

### 🧠 Common Gotchas

- Don’t forget to `git add rby1-sdk && git commit` in the parent repo — **that’s what actually links your updated submodule state**.
- Always double-check your `git submodule status` to verify the correct commit is being used.
- If stuck, you can run:
  ```bash
  git submodule update --init --recursive
  ```
