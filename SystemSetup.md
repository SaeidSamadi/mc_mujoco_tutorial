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

---
---
---

# CMAKE Version Issue for the superbuild script

We **can** install a higher version of **CMake** on **Ubuntu 20.04**, but the **default version** available in Ubuntu 20.04 repositories is **CMake 3.16.3**, as you’ve seen. 

### **How to upgrade CMake to a higher version without removing it?**

**Install a Newer Version from the Official CMake Website** (this method does not replace the system-installed CMake):
   
   - **Step 1**: Download the latest version of **CMake** from the official website:

     ```bash
     wget https://cmake.org/files/v3.21/cmake-3.21.3-linux-x86_64.tar.gz
     ```

   - **Step 2**: Extract the downloaded file:

     ```bash
     tar -zxvf cmake-3.21.3-linux-x86_64.tar.gz
     ```

   - **Step 3**: Move the extracted files to `/opt` or any directory of your choice:

     ```bash
     sudo mv cmake-3.21.3-linux-x86_64 /opt/cmake-3.21.3
     ```

   - **Step 4**: Create a symbolic link to the new version of CMake:

     ```bash
     sudo ln -s /opt/cmake-3.21.3/bin/cmake /usr/local/bin/cmake
     ```

   - **Step 5**: Verify the installed version:

     ```bash
     cmake --version
     ```

     This will now use **CMake 3.21** or the version you installed. The default **system CMake** (3.16.3) will still remain intact for system use, and you will now have access to a newer version.
     **This will show after a system re-run! (Restart or sth)**

## In case if issue

The error message you're seeing indicates that the **CMake** installation might not have been completed correctly. This often happens when a symbolic link points to the wrong location or when the `CMAKE_ROOT` path isn't set correctly.

### **Fix the CMake Installation**

To resolve this issue, let's follow these steps:

### **Step 1: Remove the Previous Symbolic Link**
First, remove the existing symbolic link you created earlier, as it may be pointing to an incomplete or incorrect path.

```bash
sudo rm /usr/local/bin/cmake
```

### **Step 2: Create the Correct Symbolic Link**
Make sure you are pointing to the correct executable in your new **CMake** installation directory. After extracting **CMake** into `/opt/cmake-3.21.3`, create the symbolic link again but ensure that you are pointing to the right executable. The full path for `cmake` inside the folder should be `/opt/cmake-3.21.3/bin/cmake`.

```bash
sudo ln -s /opt/cmake-3.21.3/bin/cmake /usr/local/bin/cmake
```

### **Step 3: Verify CMake Version**
Once the symbolic link is fixed, verify the **CMake** version again:

```bash
cmake --version
```

It should now point to the new version (3.21 or higher).
