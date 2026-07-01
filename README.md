# UAV_P2

## Overview

This repository contains a ROS 2 mission node for controlling a swarm of Crazyflie drones in the ICUAS/Gazebo simulation using Crazyswarm2’s high‑level services (`Takeoff`, `GoTo`, `Land`). The code assigns ArUco markers to specific drones, plans collision‑aware paths between them, and can be reused for any of the five simulated Crazyflies by changing only a drone ID argument.

The implementation is split into two Python modules:

- `mission_config.py` –> defines all marker positions, per‑drone assignments, obstacle pillars and helper functions (e.g. yaw alignment to face markers)
- `drone_mission.py` –> ROS 2 node that subscribes to the drone pose and runs the mission (takeoff, navigation, marker inspection, landing) for a selected drone

## Design and evolution

The project started with a very simple single‑drone script: take off, send one `GoTo` command toward a marker, then land. This matched the Crazyswarm2 service API but produced unstable behaviour in the ICUAS simulation for long horizontal moves right after takeoff. To improve stability, the motion was later divided into smaller segments at a fixed cruise altitude, so the drone followed a “stepped” path instead of a single large jump.

Once this approach worked for one drone, the architecture was generalized. All markers and their assigned drones were moved into `mission_config.py`, and the mission node was written so it only depends on a `drone_id`. Internally, the node maps `drone_id` to the correct namespace (`cf_1`-`cf_5`) and calls the same mission logic. This means the same code can fly **any** of the five drones; you can eventually loop over the drone IDs and run one mission per drone. 

```bash
# Example usage (one drone):
ros2 run icuas_competition drone_mission.py 0   # runs mission for cf_1
ros2 run icuas_competition drone_mission.py 3   # runs mission for cf_4
```

## Clock and simulation issues

During development we discovered a **time synchronisation problem** between the tmux‑launched ROS 2 nodes and the Gazebo simulation clock. The mission node uses `use_sim_time=True`, but in practice the `/clock` from Gazebo and the timing assumptions inside the mission code were not always aligned. 

Symptoms:

- The drone would climb and fly segments correctly at first, but when it approached the estimated target position, the timing mismatch caused the controller to stop thrust at the wrong moment.
- As a result, the drone often **shut down its propellers and fell** shortly before reaching some ArUco markers, even though the mission logic itself was still running.

These behaviours exposed that time handling in the current implementation is fragile. The code now uses more `spin_once`‑based waiting instead of pure `time.sleep`, and all nodes set `use_sim_time`, but **the clock issue is not fully resolved**: under some conditions, the simulated Crazyflie can still become unstable and crash near the end of a leg. 

In other words, the mission logic and APF planner work conceptually, but the interaction between ROS 2 timers, Gazebo’s `/clock` and the Crazyflie backend still needs further refinement to guarantee robust behaviour in all scenarios. 

## Path planning and trajectory choices

Several trajectory strategies were tested:

- **Direct straight‑line paths** from spawn point to marker: simple but led to aggressive motion and combined with the timing issues above, frequent crashes.
- **“L”‑shaped paths** (first along X, then along Y): easier to predict geometrically but they increased flight time and battery usage too much for the competition constraints, so they were discarded.
- **Adaptive APF paths**: the current implementation uses an Artificial Potential Field planner (attraction to the marker, repulsion from pillars) to generate intermediate waypoints and blend altitude from a cruise level to the marker height.

This APF‑based planner significantly improved obstacle avoidance and path smoothness compared to the early versions. However, it still has **edge cases**:

- Near some markers, especially when the drone is already close and repulsion and attraction are nearly balanced, the path can become oscillatory.
- In combination with the unresolved clock/timing issues, these oscillations sometimes lead to the drone losing stability and falling just before fully reaching the ArUco position.

So the current code represents a **working proof‑of‑concept** for multi‑drone APF navigation in the ICUAS simulation, but not a fully robust controller. Users should expect occasional instability near targets and should treat this repository as a baseline to improve, not a finished production solution.

## Target configurations and tuning

Once the marker positions and drone assignments were available from the competition environment, different target ordering and APF parameter configurations were tried:

- Ordering targets by ID vs. ordering them by **proximity** to the current position.
- Different attractive and repulsive gains (`k_att`, `k_rep`), safe distances, and maximum repulsion caps.
- Different cruise altitudes and arrival radii around each marker.

The final configuration orders markers by proximity (nearest‑neighbour heuristic), uses a cruise altitude that balances safety and path length, and applies APF parameters tuned to avoid excessive oscillation near pillars while still deflecting the drone away from obstacles. This tuning was empirical: several runs were evaluated until the drone could visit all its assigned markers without collisions or premature landing.

## Reusability and swarm support

Despite these limitations, the codebase is architected for multi‑drone use:

- `mission_config.py` defines which drone should visit which markers, plus the obstacle map and the yaw required to face each marker.
- `drone_mission.py` implements one generic mission node that works for any drone ID; internally, it maps `drone_id` to the corresponding `/cf_X` namespace and runs the same mission logic.
- A future extension can start one mission node per drone (e.g. via a launch file or a loop over the drone list), allowing all five Crazyflies to execute their assigned routes concurrently.

The goal of this design is **code reuse and clarity**: even though there are known issues with timing and final approach stability, the structure is ready for further tuning and experimentation on multi‑drone missions.
