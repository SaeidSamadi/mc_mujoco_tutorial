# ROS1 Installation

To install **ROS1 Noetic** (the latest and final ROS1 distribution) on **Ubuntu 20.04**, follow these steps:

---

## **Step 1: Set up the Sources**
First, ensure your system is up to date:
```bash
sudo apt update && sudo apt upgrade -y
```
Then, add the ROS package repository:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## **Step 2: Add the ROS GPG Key**
Ubuntu 20.04 uses **apt-key**, but since it is deprecated, we will use the proper method.

```bash
sudo apt install curl -y
curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | sudo tee /etc/apt/trusted.gpg.d/ros.asc
```

---

## **Step 3: Update Package Lists**
```bash
sudo apt update
```

Now, you should no longer see the `NO_PUBKEY` error, and you can proceed with installing ROS.

Try running:
```bash
sudo apt install ros-noetic-desktop-full -y
```
or the minimal version:
```bash
sudo apt install ros-noetic-ros-base -y
```
---

## **Step 4: Set Up Environment Variables**
Add ROS setup to your **bashrc**:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
If using **zsh**:
```bash
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

---

## **Step 5: Install Dependencies (rosdep, rosinstall, catkin, etc.)**
Install `rosdep` (used for managing dependencies):
```bash
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```
Install additional ROS tools:
```bash
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```

---

## **Step 6: Verify Installation**
Check if ROS is installed correctly:
```bash
roscore
```
If `roscore` runs successfully without errors, your ROS1 Noetic installation is complete.
