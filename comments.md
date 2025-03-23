# Comment for **Robot Simulation Setup & Testing**

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

---

# Comment for [wbc] Whole-Body Control Interface Testing

My Observation (NumPy Compatibility Issue on Ubuntu 22.04)

While building `mc_rtc` on **Ubuntu 22.04**, I encountered silent CMake errors and runtime issues such as:

```
ValueError: numpy.dtype size changed, may indicate binary incompatibility.
```

After investigation, I found that the issue was caused by a **mismatch between NumPy and other system packages**:

-  Running `python3 -c "import numpy; print(numpy.__file__)"` showed that **NumPy was installed locally** via pip in `~/.local/lib/...` (version 2.2.4).
-  Meanwhile, packages like `scipy`, `cython`, and `Cython` were installed **system-wide** via `apt` in `/usr/lib/...` and expected NumPy `<1.25.0`.
-  This caused ABI conflicts, especially with packages that use Cython or compiled extensions (like `mc_rtc`).

### ✅ Solution

To resolve this:

1. **Back up the locally installed NumPy:**

   ```bash
   mv ~/.local/lib/python3.10/site-packages/numpy ~/.local/lib/python3.10/site-packages/numpy_backup
   mv ~/.local/lib/python3.10/site-packages/numpy-2.2.4.dist-info ~/.local/lib/python3.10/site-packages/numpy-2.2.4.dist-info_backup
   ```

2. **Install system-compatible NumPy:**

   ```bash
   sudo apt install python3-numpy
   ```

3. **Confirm you're now using the system NumPy:**

   ```bash
   python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
   # Should show version like 1.21.5 from /usr/lib/...
   ```

4. ✅ After switching to the system NumPy, the `mc_rtc` superbuild succeeded without error.
