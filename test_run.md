# Feeding Demo Test Guide

This guide explains how to test the dummy Cornell feeding actions using two machines (or two terminals on the same machine).

## Overview

- **Server** (`drinking_node`): Runs dummy action servers that simulate cup manipulation. Runs on the Jetson or any ROS2 machine.
- **Client** (`demo_run.py`): Sends action goals to the server and prints feedback/results. Runs on a laptop or any ROS2 machine.

The actions exercised are:

| Action | Topic | Description |
|--------|-------|-------------|
| GrabCup | `/arm/drink/grab_cup_from_table` | Pick up the cup from table or wheelchair |
| BringCupToMouth | `/arm/drink/bring_cup_to_mouth` | Move the cup to the user's mouth |
| HomeCup | `/arm/drink/home_cup` | Return the cup to the home position |
| PutCupBack | `/arm/drink/put_cup_back_to_holder` | Place the cup back at table or wheelchair |

## Prerequisites

- ROS2 (Humble or later) installed on both machines
- Both machines on the same network (if running cross-machine)
- Same `ROS_DOMAIN_ID` on both machines (default is `0`)

## Step 1: Build the packages

Run this on **both** machines from the workspace root where `Demo-Software/` lives:

```bash
colcon build --packages-select cornell_feeding_interfaces cornell_feeding
source install/setup.bash
```

If you also want to run the client from the RAMMP side, make sure `cornell_feeding_interfaces` is built and sourced so Python can import it.

## Step 2: Start the server

On the **server machine** (e.g. Jetson):

```bash
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run cornell_feeding drinking_node
```

Or use the provided script:

```bash
./Demo-Software/scripts/start_drinking_server.sh
# With custom domain ID:
./Demo-Software/scripts/start_drinking_server.sh 42
```

You should see:

```
[INFO] [drinking_node]: Drinking Node starting up...
[INFO] [drinking_node]: Drinking Node ready.
```

## Step 3: Run the client

On the **client machine** (e.g. laptop):

```bash
source install/setup.bash
export ROS_DOMAIN_ID=0
python3 src/rammp/integration/demo_run.py
```

Or use the provided script:

```bash
./Demo-Software/scripts/start_feeding_client.sh
# With custom domain ID:
./Demo-Software/scripts/start_feeding_client.sh 42
```

### Client options

```
--source {table,wheelchair}          Where to grab the cup from (default: table)
--destination {table,wheelchair}     Where to put the cup back (default: table)
--outside_mouth_distance FLOAT       Distance to hold cup from mouth in meters (default: 0.05)
--timeout FLOAT                      Seconds to wait for action servers (default: 10.0)
```

Example:

```bash
python3 src/rammp/integration/demo_run.py --source wheelchair --destination wheelchair
```

## Step 4: Verify

The client should print the full sequence:

```
[INFO] Waiting for action server: /arm/drink/grab_cup_from_table
[INFO] Waiting for action server: /arm/drink/bring_cup_to_mouth
[INFO] Waiting for action server: /arm/drink/home_cup
[INFO] Waiting for action server: /arm/drink/put_cup_back_to_holder
[INFO] All action servers are available.
[INFO] Sending goal to GrabCup
[INFO] Goal accepted by GrabCup
[INFO] [GrabCup] Feedback: Moving to table
[INFO] [GrabCup] Feedback: Closing gripper on cup
[INFO] [GrabCup] Feedback: Lifting cup
[INFO] [GrabCup] Success: Cup grabbed from table
[INFO] Sending goal to BringCupToMouth
...
[INFO] Feeding demo completed successfully!
```

## Testing on a single machine

You can run both server and client on the same machine using two terminals:

```bash
# Terminal 1
source install/setup.bash
ros2 run cornell_feeding drinking_node

# Terminal 2
source install/setup.bash
python3 src/rammp/integration/demo_run.py
```

## Troubleshooting

- **"Action server not available"**: Make sure both machines use the same `ROS_DOMAIN_ID` and are on the same network. Check with `ros2 node list` and `ros2 action list` on the client machine.
- **Import errors for `cornell_feeding_interfaces`**: The interface package must be built (`colcon build --packages-select cornell_feeding_interfaces`) and sourced (`source install/setup.bash`) on the machine that fails.
- **Firewall issues**: ROS2 DDS uses multicast by default. If cross-machine discovery fails, check firewall rules or try setting `ROS_LOCALHOST_ONLY=0`.



## Run in sim
ros2 run cornell_feeding drinking_node --ros-args \
  -p run_on_robot:=false