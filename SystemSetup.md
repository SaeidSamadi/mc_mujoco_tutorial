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

## In case of error:
#### **Remove Old ROS GPG Key**
```bash
sudo apt-key del F42ED6FBAB17C654
```

---

#### **Add the New ROS GPG Key**
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

---
---
---

# SSH setup

To add an **SSH key** to your local PC for **GitHub**, follow these steps:

---

## **Step 1: Check for Existing SSH Keys**
First, check if you already have an SSH key:
```bash
ls -al ~/.ssh
```
If you see files like `id_rsa` and `id_rsa.pub`, you already have a key. If not, generate one.

---

## **Step 2: Generate a New SSH Key (if needed)**
If you don't have a key or want a new one, run:
```bash
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```
Replace `"your_email@example.com"` with your GitHub email.  
When prompted, press **Enter** to save in the default location (`~/.ssh/id_rsa`).  
For the passphrase, you can leave it empty or set one.

---

## **Step 3: Start the SSH Agent and Add Your Key**
Run:
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa
```

---

## **Step 4: Copy Your SSH Key**
Copy the public key to your clipboard:
```bash
cat ~/.ssh/id_rsa.pub
```
Copy the output (the long text starting with `ssh-rsa ...`).

---

## **Step 5: Add the Key to GitHub**
1. Go to **GitHub** → **Settings** → **SSH and GPG keys**  
   ([Direct Link](https://github.com/settings/keys))
2. Click **New SSH Key**  
3. **Title**: Name it (e.g., "Ubuntu 20.04 PC")  
4. **Key**: Paste the copied key  
5. Click **Add SSH Key**

---

## **Step 6: Test the Connection**
Run:
```bash
ssh -T git@github.com
```
If everything is set up correctly, you should see:
```bash
Hi <your-github-username>! You've successfully authenticated, but GitHub does not provide shell access.
```

---

## **Step 7: Configure Git (Optional)**
Set your Git name and email:
```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

---

## **Step 8: Clone a Repo Using SSH**
Now you can clone repositories using SSH instead of HTTPS:
```bash
git clone git@github.com:your-username/repository-name.git
```
