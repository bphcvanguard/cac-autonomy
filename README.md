# CAC 2027 Autonomy Stack

The autonomy software for **Project Vanguard (BITS Pilani, Hyderabad)** in the **Caterpillar Autonomy Challenge 2027** (Shaastra, IIT Madras). A rover crosses an obstacle field, digs sand, carries it, and builds a berm. Nobody touches the controls.

**Stack:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic (`ros_gz`) · Nav2 · robot_localization · ros2_control · Python 3.12

> This repository is a **starting skeleton**. The packages are intentionally empty. Each pod builds its own package. What's fixed is *where* code lives and *how* we work together.

---

## What's in here

```
cac-autonomy/
├── src/                      ROS 2 packages (one colcon workspace)
├── docker/                   Dockerfile + compose.yaml: the shared dev environment
├── scripts/dev.sh            one command to start, enter, build and test the container
├── .github/
│   ├── workflows/ci.yml      builds and tests every pull request
│   ├── CODEOWNERS            who reviews what
│   ├── pull_request_template.md
│   └── ISSUE_TEMPLATE/       task and bug templates
├── .gitignore
└── .editorconfig
```

### Packages

| Package | Type | Purpose | Owner |
|---|---|---|---|
| `cac_interfaces` | ament_cmake (msgs) | Shared message types (`BucketCommand`, `BucketState`) | Integrator |
| `cac_description` | ament_cmake | Rover URDF/xacro, sensor mounting, frames | Platform + Perception |
| `cac_hardware` | ament_python | Motor and sensor drivers, wheel odometry, bucket, velocity mux, E-stop/battery | Platform |
| `cac_localization` | ament_python | IMU, landmark detection, EKF pose in the arena frame | Localization |
| `cac_perception` | ament_python | Point-cloud cropping, ground segmentation, rocks and craters | Perception |
| `cac_navigation` | ament_cmake | Nav2 config, costmap zone rules, approach behaviours, recoveries | Navigation |
| `cac_mission` | ament_python | Mission state machine and mission clock | Mission/Sim/Ops |
| `cac_sim` | ament_cmake | Gazebo Harmonic arena world + obstacle randomiser | Mission/Sim/Ops |
| `cac_bringup` | ament_cmake | Launch files and parameter files | Integrator |
| `cac_tools` | ament_python | Test harness, bag tools, dashboard layout | Mission/Sim/Ops |

Every Python package has a flake8 test, so `colcon test` checks code style from day one.

---

## Setup

Pick **one** option. Both give you the same tools.

### Option A: Native Ubuntu 24.04 (recommended if your laptop runs 24.04)

1. **Install ROS 2 Jazzy** (Desktop) by following the official guide:
   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
2. **Install the packages this stack uses:**
   ```bash
   sudo apt update && sudo apt install -y \
     python3-colcon-common-extensions python3-rosdep python3-pip \
     ros-jazzy-rmw-cyclonedds-cpp \
     ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
     ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
     ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization \
     ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
     ros-jazzy-pcl-ros ros-jazzy-sensor-msgs-py \
     ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard ros-jazzy-twist-mux \
     ros-jazzy-foxglove-bridge ros-jazzy-rosbag2-storage-mcap \
     ros-jazzy-ros-gz ros-jazzy-rviz2 \
     ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common ros-jazzy-ament-flake8 \
     ros-jazzy-rqt-common-plugins ros-jazzy-plotjuggler-ros \
     python3-opencv python3-numpy python3-scipy python3-yaml python3-pytest
   ```
3. **Initialise rosdep** (once per machine):
   ```bash
   sudo rosdep init   # skip if it says it's already initialised
   rosdep update
   ```
4. **Clone, install dependencies, build, test:**
   ```bash
   git clone https://github.com/bphcvanguard/cac-autonomy.git && cd cac-autonomy
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build --symlink-install
   source install/setup.bash
   colcon test && colcon test-result --verbose
   ```
5. Add `source /opt/ros/jazzy/setup.bash` to your `~/.bashrc` so every terminal has ROS.

### Option B: Docker (any Linux machine, including other Ubuntu versions)

**Prerequisites**
- Docker Engine + the Compose plugin: https://docs.docker.com/engine/install/ubuntu/
  Then run `sudo usermod -aG docker $USER` and log out and back in, so you can use Docker without `sudo`.
- *Optional, NVIDIA GPU only:* the NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). With it, Gazebo renders on your GPU. Without it, it uses your integrated graphics or the CPU (slower, same results).
- **No GPU, old GPU or old PC is fine.** See [Slow or old PC](#slow-or-old-pc) below.

**Use it**
```bash
git clone https://github.com/bphcvanguard/cac-autonomy.git && cd cac-autonomy
./scripts/dev.sh up      # first time builds the image: ~10–40 min, ~7 GB (keep 15 GB free)
./scripts/dev.sh info    # which graphics mode you got: post this on your onboarding issue
./scripts/dev.sh sh      # a shell inside the container, in /workspace (= your clone)
# inside the container:
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install && source install/setup.bash
```

| Command | What it does |
|---|---|
| `./scripts/dev.sh up` | Builds the image if needed and starts the container in the background |
| `./scripts/dev.sh sh` | Opens a shell inside it. Open as many as you like. |
| `./scripts/dev.sh build` | `colcon build` inside the container |
| `./scripts/dev.sh test` | Build + test, exactly what CI runs |
| `./scripts/dev.sh info` | Graphics mode, renderer, CPU cores, RAM, ROS version |
| `./scripts/dev.sh down` | Stops the container |

**How it works**
- Your clone is mounted at `/workspace`. You edit files on your laptop with any editor, and build and run inside the container. `build/`, `install/` and `log/` appear in your clone and are gitignored.
- The container runs as a user with **your** UID, so files it creates are owned by you.
- `dev.sh` picks the graphics mode automatically:

  | Your machine | Mode | Service | Renderer |
  |---|---|---|---|
  | NVIDIA GPU + Container Toolkit | `nvidia` | `dev` | Your NVIDIA GPU |
  | Intel/AMD graphics, or NVIDIA without the toolkit | `dri` | `dev-cpu` | Your integrated/AMD GPU via Mesa |
  | No `/dev/dri` (VirtualBox/VMware, very old or headless PC) | `none` | `dev-sw` | `llvmpipe`: software rendering on the CPU |

  Force a mode with `CAC_GPU=none ./scripts/dev.sh up` (or `nvidia` / `dri`). `up` switches cleanly if a different mode is already running.
- GUI apps (Gazebo, RViz, rqt) open on your desktop through X11. `dev.sh up` runs `xhost +local:` to allow this.

### Slow or old PC

Everything in this repo works on a CPU-only laptop. It was tested on **2 CPU cores, 4 GB RAM and no GPU** (software rendering):

| What | Result on 2 cores, no GPU |
|---|---|
| `colcon build` + `colcon test` | ✅ ~20 s |
| Gazebo + RViz windows | ✅ open and run |
| Simple world, with the Gazebo window | ✅ real time |
| GPU-LiDAR world, with the Gazebo window | ⚠️ ~0.35× real time (slow but correct) |
| GPU-LiDAR world, headless (no Gazebo window) | ✅ real time |

**Minimum:** any 64-bit x86 CPU with 2+ cores, 4 GB RAM (8 GB recommended), 15 GB free disk.

**If the sim is slow on your machine:**
1. **Run Gazebo without its window.** Rendering the 3D view is the expensive part. Run the server only, and look at the robot in RViz or Foxglove instead:
   ```bash
   gz sim -s -r --headless-rendering <world>.sdf    # -s = server only; sensors (LiDAR/camera) still work
   ```
2. **Old GPU and Gazebo crashes with `OpenGL 3.3 is not supported`:** your GPU is too old for Gazebo's default renderer. Either use the older renderer, `gz sim --render-engine ogre <world>.sdf`, or skip the GPU: `./scripts/dev.sh down && CAC_GPU=none ./scripts/dev.sh up`.
3. **Low RAM (≤ 8 GB) and `colcon build` freezes the PC:** build fewer packages at once: `colcon build --symlink-install --parallel-workers 1`, and for C++ packages `export MAKEFLAGS=-j2` first.
4. **Run `./scripts/dev.sh info`.** If it says `llvmpipe` but you do have an Intel/AMD GPU, check that `/dev/dri` exists on your laptop (`ls /dev/dri`). If it doesn't, your graphics driver isn't loaded.

**Worlds must not download models at startup.** Gazebo worlds that `<include>` models from `fuel.gazebosim.org` hang or fail on networks that block it (some do; ours did during testing). Put every model the world needs inside `cac_sim`.

**What's inside the image** (`docker/Dockerfile`, built on `ros:jazzy-ros-base-noble`)

| Group | Contents |
|---|---|
| Base | Ubuntu 24.04, ROS 2 Jazzy (ros-base), colcon, rosdep, Cyclone DDS (default RMW) |
| Robot model + transforms | xacro, robot_state_publisher, joint_state_publisher, tf2_ros, tf2_tools |
| Navigation + estimation | Nav2 (navigation2, nav2_bringup), robot_localization |
| Control | ros2_control, ros2_controllers (incl. diff_drive_controller) |
| Perception | pcl_ros, sensor_msgs_py, OpenCV 4.6 with ArUco (Ubuntu 24.04's version: use the classic API, `cv2.aruco.detectMarkers`, not `ArucoDetector`), NumPy, SciPy |
| Teleop | joy, teleop_twist_joy, teleop_twist_keyboard, twist_mux |
| Logging + dashboard | foxglove_bridge, rosbag2 MCAP storage |
| Simulation (sim/dev stages) | Gazebo Harmonic via ros_gz, RViz2, mesa-utils |
| Dev tools (dev stage) | gdb, pytest, ament lint (flake8 etc.), rqt, PlotJuggler, vim, nano, tmux |

The `robot` stage is the base layer only (no GUI, no Gazebo), for the rover's onboard computer.

---

## The arena in simulation

`cac_sim` builds the competition arena as a Gazebo world: 9 × 5 m of tilled sand, with boulders, craters, the central column and walls, plus a randomiser that makes new obstacle fields the way the judges would. After `colcon build`:

```bash
source install/setup.bash
export GZ_PARTITION=$USER-$(hostname)    # your own Gazebo "channel"; see src/cac_sim/README.md
gz sim -r $(ros2 pkg prefix cac_sim)/share/cac_sim/worlds/schematic/arena.sdf
ros2 run cac_sim generate_arena --layout random --seed 42 --count 10 --out ~/arenas
```

All geometry, and every assumption the rulebook leaves open, lives in `src/cac_sim/config/arena.yaml`. Details, flags, what the sensors can see, and the ground-truth rule are in [`src/cac_sim/README.md`](src/cac_sim/README.md).

## Verify your setup

Post a screenshot of these on your onboarding issue:

- [ ] `ros2 doctor` runs, and `echo $ROS_DISTRO` prints `jazzy`
- [ ] `colcon build` finishes all 10 packages
- [ ] `colcon test && colcon test-result --verbose` shows **0 errors, 0 failures**
- [ ] `ros2 interface show cac_interfaces/msg/BucketState` prints the message
- [ ] `./scripts/dev.sh info` output (Docker users), so we know who's on software rendering
- [ ] `gz sim shapes.sdf` opens a Gazebo window with shapes in it
- [ ] `rviz2` opens a window

---

## How we work in git (short version)

```
Issue → branch → commits → draft PR → CI green → review → squash-merge → demo → close
```

1. **Every piece of work is an issue**, with a measurable "done when…" line. Assign yourself before starting.
2. **Branch from fresh `main`:** `git switch main && git pull && git switch -c <pod>/<short-name>`
   Pod prefixes: `plt` `loc` `per` `nav` `mis` `int` `docs`. Example: `loc/imu-driver`.
3. **Commit small and often**, and push at least daily. Never commit bags, `build/`, `install/`, `log/` or passwords.
4. **Open a draft PR early** (`gh pr create --fill --draft`). Fill in the template, especially *how you tested it*.
5. **CI must be green** before review. Run `colcon build && colcon test` locally first.
6. **One approval**, then **squash-merge**. Delete the branch.
7. **Nobody pushes directly to `main`.**
8. An issue closes when its result is **demonstrated** (video, bag, numbers), not just merged.

Useful commands: `git status` · `git diff` · `git add -p` · `git fetch && git rebase origin/main` · `gh pr checkout <n>` · `git log --oneline --graph -20`

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2: command not found` | ROS isn't sourced in this shell | `source /opt/ros/jazzy/setup.bash`. In Docker, open shells with `./scripts/dev.sh sh` (it sources ROS automatically). |
| `Package 'cac_…' not found` after building | Workspace not sourced | `source install/setup.bash` from the repo root |
| Gazebo is very slow; `./scripts/dev.sh info` says `llvmpipe` | Software rendering (no GPU in the container) | NVIDIA: install the Container Toolkit, then `./scripts/dev.sh down && ./scripts/dev.sh up`. Intel/AMD: make sure `/dev/dri` exists on the host. Either way, see [Slow or old PC](#slow-or-old-pc). |
| `dev.sh up` fails: `error gathering device information … /dev/dri` | You forced `CAC_GPU=dri` on a machine without a GPU device | `CAC_GPU=none ./scripts/dev.sh up` |
| Gazebo: `OpenGL 3.3 is not supported` / `Failed to create dummy render window` | GPU too old, or the container can't open the GPU | `gz sim --render-engine ogre …`, or `./scripts/dev.sh down && CAC_GPU=none ./scripts/dev.sh up` |
| A Gazebo world hangs at start; log shows `fuel.gazebosim.org` | The world downloads models and the network blocks Fuel | Vendor the models into `cac_sim`; never `<include>` a Fuel URL |
| Gazebo shows someone else's world, or your robot moves on its own | Two Gazebo instances on the same network found each other | Give each one its own channel: `export GZ_PARTITION=$USER-$(hostname)` (add it to `~/.bashrc`) |
| `cannot open display` / no window appears | X11 access not allowed | Run `xhost +local:` on the host, and check `echo $DISPLAY` is set before `dev.sh up` |
| `Permission denied` in `build/` or `install/` | Those folders were created by root (e.g. an old container) | `sudo rm -rf build install log` on the host, then rebuild |
| `rosdep install` fails with "rosdep update" errors | rosdep cache missing | `rosdep update`, then retry |
| `rosdep init` / `rosdep update` print `handshake operation timed out` | Slow or filtered network reaching raw.githubusercontent.com | Just re-run it until it succeeds (it often takes 2–3 tries on campus Wi-Fi) |
| ROS install step: downloading `ros2-apt-source_*.deb` fails with `Connection reset by peer` | The network blocks `release-assets.githubusercontent.com` | Use the older official setup instead of the `.deb`:<br>`sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`<br>`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \| sudo tee /etc/apt/sources.list.d/ros2.list`<br>then `sudo apt update` and continue with the guide. Or use a phone hotspot for this one step. |
| `apt` or `./scripts/dev.sh up` fails with `Unable to connect` / `TLS handshake timeout` | Network dropped mid-download | Re-run the same command. Finished steps are cached, so it resumes. |
| Two machines can't see each other's topics | Different RMW or a firewall | Both use Cyclone DDS (`echo $RMW_IMPLEMENTATION`). Same `ROS_DOMAIN_ID`. Same network. |
| `colcon test` fails on flake8 | Style errors | Read the listed lines. Single quotes, max 99 characters per line, imports sorted. |

Stuck for more than 30 minutes? Post the **exact error** and **what you already tried** in the team channel.
