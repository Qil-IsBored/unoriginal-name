# FIRA Malaysia LocknDrop Challenge (ROS1 Noetic)

ROS1 package for Lock & Drop challenge development using COEX Clover simulation and real robot workflows.

---

# Requirements

## OS

* Ubuntu 20.04

## ROS

* ROS Noetic

## Recommended

* Python 3
* catkin workspace
* Git installed

---

# Workspace Setup

If you do not have a catkin workspace yet:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

---

# Clone the Repository

Go into your workspace src folder:

```bash
cd ~/catkin_ws/src
```

Clone the repository:

```bash
git clone https://github.com/HaziqYaacop/Fira_Malaysia_LocknDrop_Challenge.git
```

---

# Build the Package

Go back to workspace root:

```bash
cd ~/catkin_ws
```

Build workspace:

```bash
catkin_make
```

---

# Source the Workspace

After building:

```bash
source ~/catkin_ws/devel/setup.bash
```

Recommended permanent setup:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# If Package Is Not Detected

If `roscd` or `roslaunch` cannot detect the package:

Initialize rosdep:

```bash
sudo rosdep init
```

Update rosdep:

```bash
rosdep update
```

Then rebuild:

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

# Verify Package Detection

Check whether ROS can detect the package:

```bash
rospack find lockndrop
```

Test package navigation:

```bash
roscd lockndrop
```

---

# Running the Package

Example:

```bash
roslaunch lockndrop main.launch
```

Replace `main.launch` with the actual launch file name used in your setup.

---

# Recommended Development Workflow

## Always pull latest changes

```bash
git pull origin main
```

## Create your own feature branch

Example:

```bash
git checkout -b feature/your-name/your-feature
```

## Push your branch

```bash
git push origin feature/your-name/your-feature
```

Do not push directly into `main` branch.

---

# Common Issues

## Package not found

Solution:

```bash
source ~/catkin_ws/devel/setup.bash
```

---

## Python script cannot run

Make scripts executable:

```bash
chmod +x scripts/*.py
```

---

## catkin_make failed

Try clean rebuild:

```bash
cd ~/catkin_ws
rm -rf build devel
catkin_make
```

---

# Repository Structure

```text
lockndrop/
├── launch/
├── scripts/
├── src/
├── package.xml
├── CMakeLists.txt
└── README.md
```

---

# Notes

* This package is developed for ROS1 Noetic.
* Make sure your package is placed inside `~/catkin_ws/src`.
* Always rebuild and source after pulling new updates.

---

# Maintainer

Haziq Yaacop
