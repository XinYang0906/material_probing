# FR3 Probing 新系统运行指南（Fake Hardware + RViz）

本指南用于把 `probing/` 与 `docker/` 复制到一台新的 Ubuntu 电脑后，重新建立
FR3 fake-hardware、MoveIt、RViz 和四个 probing primitive 的演示环境。

## 0. 重要前提

- 本指南只针对 `use_fake_hardware:=true`，不连接真实 FR3。
- `docker/` 是 Docker 配置和官方 `franka_ros2` 依赖清单，不是完整的已编译
  ROS2 workspace。首次启动时需要网络下载依赖并编译。
- 当前 `docker/` **不包含**本机 `spatula_tip` 的 FR3 URDF/SRDF/MoveIt 源码改动。
  要保持 `tip_link=spatula_tip`，应一并传输已修改的 `~/franka_ros2/` 源码目录，
  或先在新系统重新应用 `docs/fr3_tool_tcp_plan.md` 说明的改动。
- `docker/patches/fake_hardware_position_controller.patch` 会在容器启动时自动
  配置 fake hardware 使用 position trajectory controller。它使 `/joint_states`
  和 RViz 实体机器人跟随轨迹；真实 FR3 仍使用原本的 effort controller。
- 若尚未迁移 TCP 改动，可临时使用官方 `fr3_hand_tcp`，但探针高度不再是 spatula
  尖端高度，不能把该配置用于真实接触。
- 新系统至少预留 25 GB 可用磁盘空间。Docker 构建、ROS2 编译和镜像缓存会占用
  较多空间；空间不足可能使容器以退出码 137 被系统结束。

## 1. 文件放置

假设从压缩包得到：

```text
~/fr3_transfer/
├── probing/
└── docker/
```

把 Docker 配置作为 FR3 workspace 根目录使用：

```bash
mv ~/fr3_transfer/docker ~/franka_ros2
```

保留 probing 在原位置，后续会复制进容器：

```text
~/fr3_transfer/probing
~/franka_ros2
```

若传输了当前电脑已修改过的完整 `~/franka_ros2/`，直接使用该目录，不要用一个
新的 `docker/` 覆盖它。

## 2. 宿主机要求

推荐系统为 Ubuntu 24.04。安装并确认以下软件：

- Docker Engine
- Docker Compose v2（命令为 `docker compose`）
- `xhost`（Ubuntu 软件包通常为 `x11-xserver-utils`）
- 可访问互联网（首次构建时下载 ROS2/FR3 依赖）

确认命令：

```bash
docker --version
docker compose version
command -v xhost
```

若执行 Docker 命令出现 permission denied，把当前用户加入 docker 组，然后重新
登录 Ubuntu：

```bash
sudo usermod -aG docker $USER
```

## 3. 首次构建 Docker 环境

在宿主机终端执行：

```bash
cd ~/franka_ros2
cp .env.example .env
```

编辑 `.env`。本项目曾使用下面的值以避开 Dockerfile 中的 GID 冲突：

```text
USER_UID=1001
USER_GID=1001
CONTAINER_NAME=franka_ros2
```

如果构建时仍出现 UID/GID already exists，选一个未被 Docker 基础镜像占用的数值，
并保持 `CONTAINER_NAME=franka_ros2`。

在宿主机允许容器显示 RViz：

```bash
xhost +local:docker
```

构建并启动容器：

```bash
cd ~/franka_ros2
docker compose build
docker compose up -d
docker exec -it franka_ros2 /bin/bash
```

进入容器后，首次编译 FR3 workspace：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /ros2_ws/install/setup.bash
```

如果 `src/` 的权限导致 `vcs import` 或 `colcon build` 失败，在容器内执行：

```bash
sudo chown -R user:user /ros2_ws/src
```

然后重新执行上面的 `colcon build`。

## 4. 复制 probing 到容器

回到宿主机终端执行：

```bash
docker cp ~/fr3_transfer/probing franka_ros2:/ros2_ws/probing
```

检查脚本能被容器看到：

```bash
docker exec -it franka_ros2 bash -lc 'ls /ros2_ws/probing/scripts/run_fr3_probe.py'
```

## 5. TCP 选择

### 推荐：已迁移 spatula_tip

确认已把本机修改过的 FR3 源码迁移过来并重新编译后，使用：

```text
MoveIt tip: spatula_tip
Probing tip: spatula_tip
```

验证：

```bash
ros2 run tf2_ros tf2_echo fr3_link0 spatula_tip
```

### 临时兼容：只有官方 franka_ros2

若还没有 `spatula_tip` 源码改动，Terminal 1 不传 `use_spatula_tip:=true`，
Terminal 2 的每条 primitive 命令加上：

```bash
--tip-link fr3_hand_tcp
```

这只能用于 fake-hardware 轨迹演示。

## 6. 两终端运行

只启动一次 Terminal 1。Terminal 1 运行期间，不要在其他终端再次启动
`moveit.launch.py`，否则会产生多个 `/move_action` server。

### Terminal 1：启动 FR3 MoveIt 和 RViz

宿主机打开第一个终端：

```bash
docker exec -it franka_ros2 /bin/bash
```

容器内执行（已迁移 spatula_tip 时）：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  use_spatula_tip:=true
```

如果只有官方 FR3 源码，使用：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

RViz 出现后，在另一个容器终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 node list
```

必须至少看到：

```text
fr3_arm_controller active
joint_state_broadcaster active
/controller_manager
/move_group
```

`/recognize_objects`、octomap 或 RViz 感知插件的 warning 是官方配置中的可选感知
组件提示，不影响本项目的 MoveIt pose trajectory。

### Terminal 2：运行 primitives

宿主机打开第二个终端并进入同一容器：

```bash
docker exec -it franka_ros2 /bin/bash
```

容器内先执行一次公共环境设置：

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export PYTHONPATH=/ros2_ws:$PYTHONPATH
```

注意不要写成 `PYTHONPATH=/ros2_ws python3 ...`；这会覆盖 ROS2 的 Python 路径并
导致 `ModuleNotFoundError: No module named 'rclpy'`。

以下命令默认使用 `spatula_tip`。如果采用临时兼容路线，请在每条命令末尾追加
`--tip-link fr3_hand_tcp`。

#### 1. micro_compression

```bash
python3 /ros2_ws/probing/scripts/run_fr3_probe.py \
  --primitive micro_compression \
  --compression-depth 0.005 \
  --hold-s 1.0
```

阶段：`APPROACH -> COMPRESS -> HOLD -> RETRACT`。

#### 2. horizontal_drag

```bash
python3 /ros2_ws/probing/scripts/run_fr3_probe.py \
  --primitive horizontal_drag \
  --insert-depth 0.005 \
  --drag-distance -0.030 \
  --hold-s 1.0
```

阶段：`APPROACH -> INSERT -> DRAG -> RETRACT`。

#### 3. lift_detach

```bash
python3 /ros2_ws/probing/scripts/run_fr3_probe.py \
  --primitive lift_detach \
  --compression-depth 0.005 \
  --lift-height 0.040 \
  --hold-s 1.0
```

阶段：`APPROACH -> COMPRESS -> HOLD -> LIFT`。

#### 4. inclined_insertion

```bash
python3 /ros2_ws/probing/scripts/run_fr3_probe.py \
  --primitive inclined_insertion \
  --insert-depth 0.005 \
  --tilt-deg 12.0 \
  --hold-s 1.0
```

阶段：`APPROACH -> TILT -> INSERT -> HOLD -> RETRACT -> UNTILT`。

## 7. 安全限制和结束方式

- `compression_depth` 与 `insert_depth` 的硬上限都是 0.03 m。
- `inclined_insertion` 的 `tilt_deg` 只允许 10 到 15 度。
- 上述命令只用于 fake hardware。它们是 MoveIt pose goal，不是接触力控制。
- 未完成真实 TCP 标定、工具碰撞/负载、F/T 反馈和接触中止逻辑前，禁止在真机运行
  四个接触类 primitive。

停止 MoveIt 时，在 Terminal 1 按 `Ctrl+C`。结束整个容器：

```bash
cd ~/franka_ros2
docker compose down
```

收回 X11 权限：

```bash
xhost -local:docker
```
