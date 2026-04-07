# `drink_action_server` Node

**Source:** `src/rammp/integration/drink_action_server.py`  
**Package:** `drink_actions_test`  
**ROS2 Node Name:** `drink_action_server`

## Overview

`drink_action_server` is the top-level ROS2 node that exposes the RAMMP robot's drink-assistance capabilities as a set of ROS2 action servers. Each action server wraps a **High-Level Action (HLA)** — a discrete skill the robot can perform (e.g., grab a cup, bring it to the user's mouth). A supervisor node or teleoperation interface calls these actions sequentially to run a full drinking-assistance episode.

The node runs both in **simulation-only** mode (PyBullet, no physical robot required) and **on-robot** mode, controlled by a launch argument.

---

## Architecture

```
DrinkActionServers (Node)
├── PerceptionInterface      — camera + perception pipeline
├── ArmInterfaceClient       — arm/gripper command interface (robot mode only)
├── RVizInterface            — plan visualization (robot mode only)
├── FeedingDeploymentPyBulletSimulator  — motion planning + sim state
└── HLAs (one instance per skill)
    ├── PickupAndOrderAction
    ├── GrabCupFromTableAction
    ├── BringCupToMouthAction
    ├── HomeCupAction
    └── PutCupBackToHolderAction
```

Each HLA holds references to the simulator, robot interface, perception interface, and RViz interface. The node dispatches incoming action goals to the corresponding HLA via `hla_name_to_hla[name].execute_action()`.

---

## Action Servers

All five action servers use the same action type: `drink_actions_test/action/DrinkAction`.

### `DrinkAction` Interface

```
# Goal
string request_id

---
# Result
bool success
string message

---
# Feedback
string state
```

`request_id` is a free-form string for the caller to tag the request (e.g. a UUID or step label). Feedback publishes a single `state` string at the start of execution. The result signals success/failure with a human-readable `message`.

### Registered Servers

| Action Name | Topic | HLA Class |
|---|---|---|
| Pickup and order | `/arm/drink/pickup_and_order` | `PickupAndOrderAction` |
| Grab cup from table | `/arm/drink/grab_cup_from_table` | `GrabCupFromTableAction` |
| Bring cup to mouth | `/arm/drink/bring_cup_to_mouth` | `BringCupToMouthAction` |
| Home cup | `/arm/drink/home_cup` | `HomeCupAction` |
| Put cup back to holder | `/arm/drink/put_cup_back_to_holder` | `PutCupBackToHolderAction` |

All servers are registered with a `MultiThreadedExecutor`, so they can receive goals concurrently (though HLA execution itself is not internally parallelized).

---

## High-Level Actions (HLAs)

HLAs are defined in `src/rammp/actions/` and all inherit from `BaseAction` (`src/rammp/actions/base.py`).

`BaseAction` provides shared motion primitives:

| Method | Description |
|---|---|
| `move_to_joint_positions(joints)` | Plans (via PyBullet) and optionally executes a joint-space move |
| `move_to_ee_pose(pose)` | Plans and optionally executes a Cartesian end-effector move |
| `open_gripper()` / `close_gripper()` | Opens or closes the gripper |
| `grasp_tool(tool)` / `ungrasp_tool(tool)` | Attaches/detaches a tool in sim and actuates the gripper |
| `execute_robot_command(cmd, plan, tool)` | Sends a command to `ArmInterfaceClient` and optionally visualizes the plan in RViz |

In **simulation-only** mode (`robot_interface is None`), `move_to_*` methods visualize the planned trajectory in PyBullet but send no hardware commands. In **robot** mode, they plan first, optionally pause for operator confirmation (unless `--no_waits`), then send the command to the arm.

---

## Simulation / Motion Planning

`FeedingDeploymentPyBulletSimulator` maintains a live PyBullet world that mirrors the scene description (robot, wheelchair, cup, table, walls). It is used for:

- **Collision-aware motion planning** — `plan_to_joint_positions` / `plan_to_ee_pose` return a trajectory of `FeedingDeploymentWorldState` snapshots.
- **Simulation-only execution** — `visualize_plan` steps through the trajectory in PyBullet.
- **State tracking** — the sim keeps track of what is grasped, object poses, etc.

The sim runs in the same process as the node. It is **not** exposed over any ROS2 topic or service.

---

## Scene Configuration

At startup the node loads a YAML scene config file that defines all robot joint waypoints and end-effector poses for the chosen scene. The file is resolved from inside the installed `rammp` package:

```
rammp/simulation/configs/<scene_config>.yaml
```

Available configs: `wheelchair`, `vention`.

The YAML format uses typed entries:

```yaml
initial_joints:
  type: joint_positions
  values: [0.0, 0.3, ...]

before_transfer_pose:
  type: ee_pose
  values: [x, y, z, qx, qy, qz, qw]
```

---

## Launch Arguments

| Argument | Type | Default | Description |
|---|---|---|---|
| `--scene_config` | `str` | `wheelchair` | Scene config name (no `.yaml` extension) |
| `--run_on_robot` | flag | `False` | Enable hardware interfaces (`ArmInterfaceClient`, `RVizInterface`). Without this flag the node runs in sim-only mode. |
| `--use_gui` | flag | `False` | Open the PyBullet GUI window |
| `--no_waits` | flag | `False` | Skip operator confirmation prompts before each robot command |

---

## Launch Files

### `minimal.launch.py`

Minimal setup for running on the real robot without RViz or the RealSense launch file:

```bash
ros2 launch drink_actions_test minimal.launch.py scene_config:=wheelchair
```

Starts:
- `static_transform_publisher` — `map` → `world`
- `drink_action_server` — with `--run_on_robot` enabled

> **Note:** The RealSense camera node must be started separately before launching this file. The node will block on startup waiting for camera data before warm-starting DECA head perception.

### `real.launch.py`

Full hardware launch:

```bash
ros2 launch drink_actions_test real.launch.py scene_config:=wheelchair
```

Starts:
- `joint_state_publisher` — merges `robot_joint_states` + `wrist_joint_states`
- Static TF publishers: `map`→`world`, `end_effector_link`→`camera_link`, `finger_tip`→`drinkbase`
- `realsense2_camera` — RealSense D4xx with depth alignment and pointcloud enabled
- `rviz2` — with `real.rviz` config
- `drink_action_server` — with `--run_on_robot` enabled
- A `sim/` namespaced `joint_state_publisher` for the PyBullet mirror

---

## Calling an Action (Example)

```bash
ros2 action send_goal /arm/drink/grab_cup_from_table \
  drink_actions_test/action/DrinkAction \
  "{request_id: 'step_1'}"
```

The node will log the received goal, publish one feedback message, execute the HLA, and return a result with `success: true` or `success: false` plus a message.

---

## Key Dependencies

| Package | Role |
|---|---|
| `rclpy` | ROS2 Python client library |
| `drink_actions_test` | Custom action definition (`DrinkAction`) |
| `rammp.interfaces.perception_interface` | Wraps RealSense camera data |
| `rammp.interfaces.rviz_interface` | Publishes markers / plan visualization to RViz |
| `rammp.control.robot_controller.arm_client` | Sends joint/Cartesian/gripper commands to the arm |
| `rammp.simulation.simulator` | PyBullet-based simulator and motion planner |
| `pybullet_helpers` | Geometry utilities and PyBullet wrappers |
