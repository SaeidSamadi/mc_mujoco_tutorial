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

## 📁 Suggestion for README Section (Generalized)

You can add this to your repo’s README as a section:

```markdown
### 🧱 Run Simulator Without Docker (Local MuJoCo)

If you want to avoid using Docker (e.g., due to GUI issues), you can run the simulator locally:

```bash
# Extract simulator from Docker
docker create --name rby1 rainbowroboticsofficial/rby1-sim
docker cp rby1:/root/exe/app ~/rby1-sim-app
docker rm rby1
```

```bash
# Run the simulator
cd ~/rby1-sim-app
sudo apt install -y libglfw3 libgl1-mesa-glx libxrandr2 libxinerama1 libxi6 libxcursor1
export LD_LIBRARY_PATH=$(pwd)
export MUJOCO_GL=glfw
export DISPLAY=:1  # or :0
./app_main
```

Now in another terminal:

```bash
cd rby1-sdk/build/examples/cpp
./example_demo_motion 127.0.0.1:50051
```
```

---

Let me know if you'd like this packed into a `.sh` installer or a downloadable starter zip. You're now *Docker-free and in full control*, my friend 👑
