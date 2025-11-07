# FRA502-LAB-6703
Peeradon Ruengkaew 6703 (Moke)

## Table of Contents

1. [Workspace Analysis](#workspace-analysis)
2. [System Architecture](#system-architecture)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Auto Mode Flow](#auto-mode-flow)


## Workspace Analysis

The 3R arm workspace is a spherical shell bounded by the minimum and maximum reach of links 2-3 and the wrist tool:

- Link lengths: `L2 = 0.12 m`, `L3 = 0.25 m`, `Tool = 0.28 m`
- Reach: `r_min = 0.10 m`, `r_max = L2 + L3 + Tool = 0.65 m`
- Base offset along +Z: `L1 = 0.20 m`

### How to verify

1. Build the workspace sampler node:
   ```bash
   colcon build --packages-select lab4
   source install/setup.bash
   ```
2. Launch the robot and bring up RViz with the provided launch file:
   ```bash
   ros2 launch lab4 lab4.launch.py
   ```
3. Start the random workspace node. It publishes sampled poses within the spherical shell and validates them through the IK solver before sending them to `/target`:
   ```bash
   ros2 run lab4 random_pos
   ```
4. Inspect `/target` in RViz or via:
   ```bash
   ros2 topic echo /target
   ```
   Every published pose should satisfy `r_min < sqrt(x^2 + y^2 + (z-L1)^2) < r_max`. This double-checks both the analytical workspace and the numerical IK validation done inside `random_pos.py`.
5. Optionally log samples and verify in Python:
   ```bash
   ros2 topic echo /target --csv > /tmp/targets.csv
   python3 scripts/check_workspace.py /tmp/targets.csv
   ```
   where `scripts/check_workspace.py` can be a short script that recomputes the radius for each sample and reports any violations.


## System Architecture


## Installation


## Usage


## Auto Mode Flow

Auto Mode uses the `/random_target` service (`controller_interfaces/srv/RandomTarget`) to both request the next pose and report whether the previous pose was achieved within 10 seconds:

1. `controller_bt.py` loads the behavior tree, run the node:
   ```bash
   ros2 run lab4 controller_bt
   ```
2. Start the random target server (`random_pos.py`) as shown above.
3. Switch to Auto Mode:
   ```bash
   ros2 service call /set_mode controller_interfaces/srv/SetMode "{mode: 'AM'}"
   ```
4. The AM behavior tracks each target with velocity control, checks the Jacobian condition number, and enforces a 10 s travel deadline.
5. After reaching a goal, the controller reports `target_reached=true` along with the achieved pose back to `/random_target`, triggering the next sample immediately. If the timeout is exceeded, the controller reports `target_reached=false`, logs the error, and requests a fresh goal so the sequence can continue without manual intervention.

You can monitor the handshake with:
```bash
ros2 topic echo /joint_states
ros2 service type /random_target
ros2 service call /random_target controller_interfaces/srv/RandomTarget "{request_new_target: true, target_reached: false}"
```
The log stream should clearly show whether each request finished under 10 seconds, satisfying the lab requirement.