## 🔧 General Strategy: Replacing Docker with Local MuJoCo

---

### ✅ Step 0: Understand What the Docker Container Was Doing

In your tutorial, Docker was used to:
- Run the RB-Y1 simulator (GUI via MuJoCo)
- Expose gRPC interface on port `50051`
- Provide `libmujoco.so`, `app_main`, and robot models inside the container

To bypass it, you just need to:
> ✔ Extract that simulator app  
> ✔ Run it locally on your system  
> ✔ Point your test app (e.g., `example_demo_motion`) to `localhost:50051`

---

### ✅ Step 1: Build Rainbow SDK (Same as Before)

No changes here — just follow your clean virtual environment approach:

```bash
python3 -m venv venv
source venv/bin/activate
pip install conan cmake
conan profile detect --force
```

Then:
```bash
git clone --recurse-submodules git@github.com:RainbowRobotics/rby1-sdk.git
cd rby1-sdk
conan install . -s build_type=Release -b missing -of build
cmake --preset conan-release -D BUILD_EXAMPLES=ON
cmake --build --preset conan-release
```

✅ This gives you the test apps like `example_demo_motion`.

---

### ✅ Step 2: Extract the Simulator from Docker

Instead of running the simulator in Docker, copy it out **once** and reuse it:

```bash
docker create --name rby1 rainbowroboticsofficial/rby1-sim
docker cp rby1:/root/exe/app ~/rby1-sim-app
docker rm rby1
```

---

### ✅ Step 3: Run the Simulator Locally (No Docker!)

```bash
cd ~/rby1-sim-app

# Ensure required libs are installed
sudo apt install -y libglfw3 libgl1-mesa-glx libxrandr2 libxinerama1 libxi6 libxcursor1

# Set environment vars
export LD_LIBRARY_PATH=$(pwd)
export MUJOCO_GL=glfw
export DISPLAY=:1  # or :0 depending on your setup

# Run the simulator
./app_main
```

Optional: create a `run_sim.sh` script to make it easy.

---

### ✅ Step 4: Run Your C++ Example App

Open a **second terminal** (with the simulator still running), and:

```bash
source venv/bin/activate  # if needed
cd ~/rby1-sdk/build/examples/cpp
./example_demo_motion 127.0.0.1:50051
```

✅ If the simulator is running, your robot will now perform motions as expected.

---

## 🧠 TL;DR: Replacing Docker With Local MuJoCo

| Step                      | Original (Docker)                                                | New (Local)                                                  |
|---------------------------|------------------------------------------------------------------|---------------------------------------------------------------|
| Run Simulator             | `docker run rainbowroboticsofficial/rby1-sim`                   | Extract + `./app_main` with `libmujoco.so` + env setup       |
| GUI Access                | Needs X11 & `xhost +`                                            | Native OpenGL via `libglfw`, much more stable                |
| Port Communication        | `-p 50051:50051`                                                  | No change (still use `127.0.0.1:50051`)                      |
| Motion Test App           | Run from `cpp` examples                                          | Same (point to local gRPC address)                          |
| Dependencies              | Inside container                                                 | Must be manually installed (OpenGL, GLFW)                   |


---

Comment:

My Observation:

The RB-Y1 simulator runs **smoothly inside Docker on Ubuntu 24.04**, but causes issues on **Ubuntu 22.04** due to **X11/GLFW compatibility problems**.

In Ubuntu 22.04, when trying to run the simulator using Docker with GUI support (`xhost +`, `-e DISPLAY`, etc.), the application fails to launch with the following error:

```
ERROR: Could not initialize GLFW
```

This is caused by **incompatibility between the container's OpenGL/GLFW requirements and the host's older X11 libraries**. Ubuntu 24.04 ships with newer versions of X11, Mesa, and Wayland support, which are **more compatible with GLFW-based GUI apps running in Docker**.

As a workaround for Ubuntu 22.04 users, instead of running the simulator inside Docker, you can:

1. **Extract the simulator from the Docker image** using:

   ```bash
   docker create --name temp_sim rainbowroboticsofficial/rby1-sim
   docker cp temp_sim:/root/exe/app ~/rby1-sim-app
   docker rm temp_sim
   ```

2. **Run the simulator natively (outside Docker)** on your host machine:

   ```bash
   cd ~/rby1-sim-app
   export LD_LIBRARY_PATH=$(pwd)
   export MUJOCO_GL=glfw
   ./app_main
   ```

This approach bypasses all the X11/GLFW compatibility issues and allows the simulator to run directly using your system's native graphics stack.

This workaround has been tested and confirmed working on Ubuntu 22.04.
