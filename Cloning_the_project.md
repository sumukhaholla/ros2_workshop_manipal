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

First, install rosdep if not there:
```bash
sudo apt install python3-rosdep -y
```
Also, "colcon". If not installed:
```bash
sudo apt install python3-colcon-common-extensions -y
```

Then, initialise `rosdep` if you haven't already (only needed once per machine):

```bash
sudo rosdep init
rosdep update
```

Then, from the **workspace root**, run:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys="linkattacher_msgs warehouse_ros_mongo"
```

| Flag | Meaning |
|---|---|
| `--from-paths src` | Scan the `src/` folder for packages |
| `--ignore-src` | Skip packages that are already present as source in the workspace |
| `-r` | Continue even if some packages can't be resolved (non-fatal) |
| `-y` | Answer *yes* automatically to all install prompts |

> ✅ This single command installs **all** ROS and system-level libraries your packages depend on, as declared in their `package.xml` files.

Sometimes moveit-visuals-tools will not be installed, hence run following command:
```bash
sudo apt install ros-humble-moveit-visual-tools -y
```
Also change the path inside world files pointing from ros2_workshop_manipal to ros2_ws(or name of your workspace)
By running this command:
```bash
sed -i 's|/home/administrator/ros2_workshop_manipal/install/ros2_description/share/ros2_description/config/medi_assist_config.yaml|/home/sumukha/ros2_ws/install/ros2_description/share/ros2_description/config/medi_assist_config.yaml|g' ~/ros2_ws/src/ros2_description/worlds/medi_assistV02.world
```
Verify if its already done
```bash
grep "parameters" ~/ros2_ws/src/ros2_description/worlds/medi_assistV02.world
```

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