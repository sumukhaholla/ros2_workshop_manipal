## Please check if ROS2 is installed or sourced properly
Make sure that, you have followed ROS2_Installation_Guide.md file and ROS2 is up and running.
If not do source your ROS2 in the terminal by running this:
```
source /opt/ros/humble/setup.bash
```
Or put it under ~/.bashrc file
```
gedit ~/.bashrc
```
At last line put this statement
```
source /opt/ros/humble/setup.bash
```
Once done, close your terminal and try to restart your terminal

Thats it. You are good go for next steps 
## 📦 Getting This Project

All custom ROS 2 packages for this project are hosted on GitHub. Follow the steps below to clone the repository and automatically install every dependency the packages declare.

### Step 1 — Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2 — Clone the Repository

```bash
git clone https://github.com/sumukhaholla/ros2_workshop_manipal.git .
```

### Step 3 — Install All Package Dependencies (single command)

ROS 2 uses **`rosdep`** to automatically read every `package.xml` in your workspace and install the required system and ROS dependencies in one command.

First, initialise `rosdep` if you haven't already (only needed once per machine):

```bash
sudo rosdep init
rosdep update
```

Then, from the **workspace root**, run:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

| Flag | Meaning |
|---|---|
| `--from-paths src` | Scan the `src/` folder for packages |
| `--ignore-src` | Skip packages that are already present as source in the workspace |
| `-r` | Continue even if some packages can't be resolved (non-fatal) |
| `-y` | Answer *yes* automatically to all install prompts |

> ✅ This single command installs **all** ROS and system-level libraries your packages depend on, as declared in their `package.xml` files.

### Step 4 — Build the Workspace

```bash
cd ~/ros2_ws
colcon build
```

To build only a specific package:

```bash
colcon build --packages-select <package_name>
```

### Step 5 — Source the Workspace

After building, source the local workspace overlay so ROS 2 can find your packages:

```bash
source ~/ros2_ws/install/setup.bash
```

To make this permanent:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 🚀 Full Setup — One After Another

Here's the complete sequence from a fresh ROS 2 install to a ready workspace:

```bash
# 1. Create workspace and clone
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-username/your-repo.git

# 2. Install all dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build

# 4. Source
source ~/ros2_ws/install/setup.bash
```

---