# ROS 2 Humble Hawksbill — Installation Guide

> **Platform:** Ubuntu 22.04 (Jammy Jellyfish)  
> **Distribution:** ROS 2 Humble Hawksbill (LTS — supported until May 2027)  
> **Official Docs:** https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

---

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is an open-source robotics middleware framework designed to build complex, distributed robot applications. Despite its name, it is not an operating system — it's a collection of software libraries, tools, and conventions that sit on top of a standard OS to help developers:

- Build **modular, reusable** robotic software components (called *nodes*)
- Enable **real-time, reliable communication** between processes using a publish/subscribe and request/response model
- Work across **multiple machines, languages, and platforms** seamlessly
- Use a rich ecosystem of tools like **RViz** (3D visualization), **rqt** (GUI tools), **ros2bag** (data recording), and more

### Key Concepts at a Glance

| Concept | Description |
|---|---|
| **Node** | A single executable that performs computation (e.g., sensor driver, controller) |
| **Topic** | Named bus for asynchronous publish/subscribe communication |
| **Service** | Synchronous request/response communication between nodes |
| **Action** | Long-running tasks with feedback and cancellation support |
| **Package** | The basic unit of software organization in ROS 2 |
| **Launch File** | Script to start multiple nodes and configure them together |
| **Workspace** | Directory where you build and source your ROS 2 packages |

### Why ROS 2 over ROS 1?

ROS 2 was built from the ground up to address limitations of ROS 1, bringing:

- ✅ **Real-time support** via DDS (Data Distribution Service) middleware
- ✅ **Security** with built-in SROS2 (encrypted, authenticated communication)
- ✅ **Multi-platform** support (Linux, Windows, macOS)
- ✅ **No single master node** — decentralized discovery
- ✅ **Production-ready** quality and lifecycle management for nodes

### About Humble Hawksbill

Humble Hawksbill is a **Long-Term Support (LTS)** release of ROS 2, targeting Ubuntu 22.04 Jammy Jellyfish. It is one of the most widely used and well-supported ROS 2 distributions for both research and production robotics.

---

## 📋 Prerequisites

Before you begin, ensure you have:

- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish) — 64-bit
- **Architecture:** `amd64` or `arm64`
- **Internet connection** for downloading packages
- **`sudo` privileges**

> ROS 2 Humble does **not** officially support Ubuntu 20.04 via deb packages. Use Ubuntu 22.04.

---

## 🛠️ Installation Steps

### Step 1 — Set Locale

ROS 2 requires a locale that supports UTF-8. Run the following to configure it:

```bash
# Check current locale
locale

# Install and generate UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

---

### Step 2 — Setup ROS 2 APT Sources

First, enable the **Ubuntu Universe repository**:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Next, install the **`ros2-apt-source`** package, which automatically configures the ROS 2 repository and manages signing keys:

```bash
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```

> The `ros2-apt-source` package will automatically receive key/repo updates when new versions are released — no manual re-configuration needed.

---

### Step 3 — Update & Upgrade System

Refresh the package cache and upgrade existing packages. This step is **critical** on a fresh Ubuntu 22.04 installation.

```bash
sudo apt update
sudo apt upgrade
```

> **Warning:** On a freshly installed Ubuntu 22.04 system, skipping `sudo apt upgrade` before installing ROS 2 can trigger the **removal of critical system packages** (like `systemd` and `udev`). Always upgrade first. See [ros2/ros2#1272](https://github.com/ros2/ros2/issues/1272) for details.

---

### Step 4 — Install ROS 2 Humble

Choose the installation variant that fits your use case:

#### Option A: Desktop Install *(Recommended)* <--If your system has got some space, please go for this installation, you get GUI and simulator
Includes ROS core libraries, RViz 2 visualization tool, demos, and tutorials.

```bash
sudo apt install ros-humble-desktop
```

#### Option B: ROS-Base Install *(Bare Bones)*
Includes only communication libraries, message packages, and CLI tools. No GUI tools.

```bash
sudo apt install ros-humble-ros-base
```

#### Option C: Development Tools *(Optional but Recommended)*
Installs compilers and build tools needed to build your own ROS 2 packages.

```bash
sudo apt install ros-dev-tools
```

> For most users, install **both** the Desktop variant and the dev tools:
> ```bash
> sudo apt install ros-humble-desktop ros-dev-tools
> ```

---

### Step 5 — Configure Environment

Source the ROS 2 setup script to make the `ros2` CLI and all installed packages available in your shell:

```bash
source /opt/ros/humble/setup.bash
```

#### Make it permanent (auto-source on every terminal)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> If you use **Zsh**, replace `.bashrc` with `.zshrc` and `setup.bash` with `setup.zsh`.
> If you use **sh**, use `setup.sh` instead.

---

## Verify Installation

Run the classic talker-listener demo to confirm ROS 2 is working correctly.
Please make sure that demo_nodes_cpp and demo_nodes_py is available. If not try running below installation

```bash
sudo apt install ros-humble-demo-nodes-cpp
```

```bash
sudo apt install ros-humble-demo-nodes-py
```

**Terminal 1 — C++ Talker:**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 — Python Listener:**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see output similar to:

```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [listener]: I heard: [Hello World: 1]
```

This confirms both the **C++ and Python client libraries** are installed and communicating correctly. 🎉

---
## Also do some changes in the ~/.bashrc file, so that different systems doesn't collide when running ROS2 packages

Openup bashrc file
```bash
gedit ~/.bashrc
```
At the end of file, paste this:
```bash
export ROS_DOMAIN_ID=0
```
Here ROS_DOMAIN_ID number can be anything, but keep it unique and random so that no other systems have same ID

## Usually ROS2 uses default fastDDS, if nothing is selected. yet sometimes you might get error telling that there is RMW implemntation. If so, follow below steps

In the terminal, run following command for installation:
```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
Then source your workspace
```bash
source /opt/ros/humble/setup.bash
```
Later run this command, to check what RMW implementation was selected:
```bash
echo $RMW_IMPLEMENTATION
```
If nothing prints, no need to panic that means, ROS2 uses fastDDS. Try to run example talker and listner packages (that is in verify installation)
If still Talker and listner is not yet running then, please select any one of the RMW implementation, taht is:
Option A: Using fastDDS
```bash
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
```
Option B: Using Cyclone DDS
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```
Later source your environment:
```bash
source /opt/ros/humble/setup.bash
```
***If there are any more difficulties or problems, please use chatGPT or claude***
---

## 🔧 Post-Installation Tips

### Check ROS 2 version
```bash
ros2 --version
```

### List available packages
```bash
ros2 pkg list
```

### Run `ros2doctor` to check your environment
```bash
ros2 doctor
```

### Auto-complete for `ros2` CLI
If you installed `ros-dev-tools`, tab-completion should work automatically after sourcing the setup file.

---

## Uninstalling ROS 2 Humble

To remove all ROS 2 Humble packages:

```bash
sudo apt remove '~nros-humble-*' && sudo apt autoremove
```

To also remove the ROS 2 repository configuration:

```bash
sudo apt remove ros2-apt-source
sudo apt update
sudo apt autoremove
sudo apt upgrade
```

---

## Next Steps & Resources

| Resource | Link |
|---|---|
| Official Tutorials (Beginner) | https://docs.ros.org/en/humble/Tutorials.html |
| Understanding Nodes | https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes |
| Creating a Workspace | https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace |
| Writing a Publisher/Subscriber (Python) | https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html |
| ROS 2 Concepts | https://docs.ros.org/en/humble/Concepts.html |
| Installation Troubleshooting | https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html |
| RMW Implementations | https://docs.ros.org/en/humble/Installation/RMW-Implementations.html |

---

> 📝 *This guide is based on the official ROS 2 Humble documentation. For the most up-to-date instructions, always refer to: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html*