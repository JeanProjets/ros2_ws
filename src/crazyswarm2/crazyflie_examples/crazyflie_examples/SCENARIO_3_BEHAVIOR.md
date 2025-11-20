# Hackathon Scenario 3: Mobile Target Search and Neutralize

## Overview

Scenario 3 is an advanced multi-drone coordination mission where four drones work together to locate and neutralize a **moving target** drone in a 10m x 6m x 8m cage environment.

## Drone Roles

| Drone ID | Initial Role | Description |
|----------|-------------|-------------|
| **cf1** | N_1 (Neutral 1) | Safety zone patrol, becomes Leader or Follower |
| **cf2** | N_2 (Neutral 2) | Safety zone patrol, becomes Leader or Follower |
| **cf3** | P (Patrol) | Area scout, performs kamikaze attack |
| **cf4** | C (Target) | Hostile drone, moves autonomously |

## Environment Setup

### Cage Dimensions
- **X-axis**: 10.0 meters
- **Y-axis**: 6.0 meters
- **Z-axis**: 8.0 meters
- **Origin**: (0, 0, 0) at bottom front-left corner

### Safety Zone
- **Size**: 3m x 6m area at X = 0-3m
- **Purpose**: "Home base" - unlikely to contain hostile target
- **Color**: Blue (in diagrams)

### Hostile Zone
- **Size**: 7m x 6m area at X = 3-10m
- **Purpose**: Area where target operates
- **Target Patrol Center**: (6.5, 3.0, 5.0)

## Starting Positions

### Ground Level (Z = 0)
```
cf1 (N_1): (2.5, 2.5, 0.0)
cf2 (N_2): (2.5, 3.5, 0.0)
cf3 (P):   (3.0, 5.0, 0.0)
cf4 (C):   (6.5, 3.0, 0.0) → moves to 5.0m height
```

### Flight Altitudes
- **N_1, N_2, P**: 4.0 meters (optimal for horizontal camera detection)
- **Target (C)**: 5.0 meters (slightly higher)

## Mission Phases

---

## Phase 1: Initialization (10 seconds)

### Objective
Initialize all drones and establish operational positions.

### Behavior

#### Drones N_1, N_2, P (cf1, cf2, cf3)
1. **Takeoff**: Ascend from ground to 4.0m altitude
2. **Duration**: 3.0 seconds
3. **Final positions**:
   - cf1: (2.5, 2.5, 4.0)
   - cf2: (2.5, 3.5, 4.0)
   - cf3: (3.0, 5.0, 4.0)

#### Target Drone C (cf4)
1. **Takeoff**: Ascend to 5.0m altitude at center position
2. **Duration**: 3.0 seconds
3. **Final position**: (6.5, 3.0, 5.0)
4. **Autonomous patrol activated**: Begins moving in square pattern

### Mobile Target Pattern
The target drone begins continuous autonomous movement:

```
Waypoint Path (3m x 3m square):
1. (8.0, 4.5, 5.0)  ← Top-right
2. (8.0, 1.5, 5.0)  ← Bottom-right
3. (5.0, 1.5, 5.0)  ← Bottom-left
4. (5.0, 4.5, 5.0)  ← Top-left
5. → Repeat from waypoint 1
```

- **Movement duration**: 3.0 seconds per waypoint
- **Wait time**: 3.5 seconds total per waypoint (includes movement)
- **Pattern**: Continuous clockwise square
- **Speed**: Approximately 1.0 m/s

### Phase Duration
- **Minimum**: 10 seconds
- **Includes**: Takeoff + stabilization + patrol initialization

---

## Phase 2: Patrol (Max 2 minutes)

### Objective
Search the entire cage for the mobile target using coordinated patrol patterns.

### N_1 and N_2: Safety Zone Verification

#### N_1 (cf1) Patrol Pattern
```
Route: Safety zone sweep
1. (2.5, 2.5, 4.0) [start]
2. (1.5, 1.5, 4.0)
3. (2.5, 1.5, 4.0)
4. (2.5, 4.5, 4.0)
5. (1.5, 4.5, 4.0)
6. (2.5, 2.5, 4.0) [return]
```

#### N_2 (cf2) Patrol Pattern
```
Route: Safety zone sweep (offset pattern)
1. (2.5, 3.5, 4.0) [start]
2. (1.5, 2.0, 4.0)
3. (2.5, 2.0, 4.0)
4. (2.5, 5.0, 4.0)
5. (1.5, 5.0, 4.0)
6. (2.5, 3.5, 4.0) [return]
```

- **Movement duration**: 2.0 seconds per waypoint
- **Total time**: ~2.5 seconds per waypoint (with battery drain simulation)
- **Purpose**: Verify safety zone is clear (unlikely target location)

### P (cf3): Hostile Zone Scan

#### Patrol (P) Extended Search Pattern
```
Route: Comprehensive hostile zone coverage
1. (3.0, 5.0, 4.0) [start]
2. (4.0, 5.0, 4.0)
3. (6.0, 5.0, 4.0)
4. (8.0, 5.0, 4.0)
5. (8.0, 3.0, 4.0) ← High probability detection zone
6. (8.0, 1.0, 4.0)
7. (6.0, 1.0, 4.0)
8. (4.0, 3.0, 4.0)
```

- **Movement duration**: 3.0 seconds per waypoint
- **Total time**: 3.5 seconds per waypoint
- **Detection checks**: Performed at each waypoint

### Target Detection System

#### Detection Method
- **Type**: Proximity-based (simulates camera detection)
- **Range**: 1.5 meters radius
- **Frequency**: Checked after each waypoint movement
- **Target tracking**: Real-time position updates from mobile target

#### Detection Logic
```python
if distance(drone_position, target_position) < 1.5m:
    TARGET_DETECTED = True
    Record target position
    Broadcast to all drones
    Proceed to Phase 3
```

#### Simultaneous Checks
All three drones (N_1, N_2, P) continuously check for target:
- **N_1**: Checks during safety zone patrol
- **N_2**: Checks during safety zone patrol
- **P**: Checks during hostile zone scan

**First detection wins** → Proceeds to Phase 3

### Phase Exit Conditions

#### Success Path
- **Condition**: Any drone detects target within 1.5m
- **Next Phase**: Phase 3 - Detection & Role Assignment
- **Typical time**: ~1 minute (target usually detected by P)

#### Failure Path
- **Condition**: P completes all waypoints without detection
- **Next Phase**: Phase 3b - Fallback Scan
- **Typical time**: ~2 minutes (full patrol completed)

---

## Phase 3: Detection & Role Assignment (7 seconds)

### Triggered When
Target detected during patrol phase by any drone.

### Step 1: Target Position Broadcast (1 second)

The detecting drone shares target location:
```
Detector → All drones: Target at (X, Y, Z)
```

All drones receive:
- Target position coordinates
- Detection confirmation flag
- Current timestamp

### Step 2: Battery Assessment (1 second)

Both neutral drones report battery levels:
```
cf1 (N_1): Battery = ~3.9V (simulated drain)
cf2 (N_2): Battery = ~3.85V (simulated drain)
```

Battery simulation:
- **Starting voltage**: 4.0V ± 0.1V (random variation)
- **Drain rate**: 0.01V per second
- **Lower battery** → consumed more energy during patrol

### Step 3: Role Assignment (1 second)

```python
if cf1_battery > cf2_battery:
    LEADER = cf1 (N_1)
    FOLLOWER = cf2 (N_2)
else:
    LEADER = cf2 (N_2)
    FOLLOWER = cf1 (N_1)
```

**Rationale**: Drone with highest battery becomes Leader (needs more energy for engagement)

### Roles After Assignment

| Role | Drone | Responsibility |
|------|-------|----------------|
| **LEADER** | Higher battery N | Front position, primary jammer |
| **FOLLOWER** | Lower battery N | Support position, secondary jammer |
| **PATROL** | cf3 | Kamikaze attack drone |
| **TARGET** | cf4 | Hostile (continues moving) |

### Phase Duration
- **Total**: 7 seconds
- **Buffer time**: Remaining time used for coordination and preparation

---

## Phase 3b: Fallback Scan (30 seconds)

### Triggered When
Patrol phase completes without target detection.

### Step 1: Reposition to Scan Line (3.5 seconds)

All three drones move to X = 3m boundary line:

```
Formation: Vertical line at safety zone boundary
cf1 (N_1): (3.0, 1.5, 4.0)  ← Bottom
cf2 (N_2): (3.0, 3.0, 4.0)  ← Middle
cf3 (P):   (3.0, 4.5, 4.0)  ← Top
```

- **Movement duration**: 3.0 seconds
- **Total time**: 3.5 seconds (with positioning)
- **Purpose**: Create observation line facing hostile zone

### Step 2: Scanning Phase (20 seconds)

All drones hover at scan positions and continuously check for target:

```
Scan Loop (every 0.5 seconds for 20 seconds):
  For each drone (cf1, cf2, cf3):
    Get current target position
    Calculate distance to target
    If distance < 2.25m (extended range):
      TARGET DETECTED
      → Proceed to Phase 3 (Role Assignment)
      Break loop
```

**Extended detection range**: 1.5m × 1.5 = 2.25m (increased for fallback)

### Outcomes

#### Success: Target Detected
- **Action**: Broadcast target position
- **Next**: Phase 3 - Detection & Role Assignment
- **Then**: Phase 4 - Dynamic Engagement

#### Failure: Target Not Detected
- **Status**: Mission Failed
- **Action**: Log failure, return to base
- **Next**: Phase 5 - Mission Complete (FAILED status)

---

## Phase 4: Dynamic Engagement (Variable duration)

### Triggered When
Leader and Follower roles assigned, target position known.

### Target State During Engagement
**IMPORTANT**: Target continues moving in its patrol pattern throughout engagement!

---

### Step 1: Initial Approach (4.5 seconds)

#### Calculate Engagement Positions

```python
target_pos = mobile_target.get_current_position()  # Real-time position

# Leader positions in front of target
leader_engage_pos = target_pos + [-1.5, 0, 0]

# Follower maintains offset from Leader
follower_offset = [-0.5, -0.5, -0.5]
follower_engage_pos = leader_engage_pos + follower_offset
```

Example calculation:
```
Target at: (7.0, 3.0, 5.0)
Leader →   (5.5, 3.0, 5.0)  [1.5m in front]
Follower → (5.0, 2.5, 4.5)  [0.5m behind Leader in all axes]
```

#### Execute Approach
```
LEADER.goTo(leader_engage_pos, duration=4.0s)
FOLLOWER.goTo(follower_engage_pos, duration=4.0s)
Wait 4.5 seconds (movement + stabilization)
```

---

### Step 2: Dynamic Jamming Phase (20 seconds)

This is the **critical dynamic tracking** phase.

#### Tracking Loop (1-second intervals)

```python
For i = 1 to 20 seconds (20 iterations):
    # Get target's CURRENT position (it's moving!)
    target_pos = mobile_target.get_current_position()

    # Recalculate engagement positions
    leader_engage_pos = target_pos + [-1.5, 0, 0]
    follower_engage_pos = leader_engage_pos + [-0.5, -0.5, -0.5]

    # Send movement commands
    LEADER.goTo(leader_engage_pos, duration=1.0s)
    FOLLOWER.goTo(follower_engage_pos, duration=1.0s)

    # Wait for next update cycle
    sleep(1.0 second)

    # Update internal positions and battery
    LEADER.update_position(leader_engage_pos)
    FOLLOWER.update_position(follower_engage_pos)
    LEADER.simulate_battery_drain(1.0s)
    FOLLOWER.simulate_battery_drain(1.0s)
```

#### Visual Example

Target moving clockwise in square pattern:

```
Second 0:
Target:   (8.0, 4.5, 5.0)
Leader:   (6.5, 4.5, 5.0) [tracking]
Follower: (6.0, 4.0, 4.5)

Second 3 (target moved):
Target:   (8.0, 3.0, 5.0) ← moved down
Leader:   (6.5, 3.0, 5.0) ← followed
Follower: (6.0, 2.5, 4.5) ← followed

Second 6 (target moved):
Target:   (8.0, 1.5, 5.0) ← moved down
Leader:   (6.5, 1.5, 5.0) ← followed
Follower: (6.0, 1.0, 4.5) ← followed
```

#### Formation Maintenance
- **Leader**: Maintains 1.5m distance in front (lower X) of target
- **Follower**: Maintains 0.5m offset behind Leader in X, Y, Z
- **Update rate**: 1 Hz (every second)
- **Total duration**: 20 seconds of continuous tracking

#### Jamming Simulation
During this phase, Leader and Follower simulate jamming target communications:
- Visual representation: Drones maintain close formation
- No actual RF jamming implemented (simulated mission)

---

### Step 3: Kamikaze Attack with Interception (5 seconds)

The Patrol drone performs a high-speed intercept of the **still-moving** target.

#### Rapid Tracking Loop (0.5-second intervals)

```python
For i = 1 to 10 iterations (5 seconds total):
    # Get target's CURRENT position
    target_pos = mobile_target.get_current_position()

    # Calculate intercept position (0.5m above target)
    kamikaze_pos = target_pos + [0, 0, 0.5]

    # Execute rapid movement
    PATROL.goTo(kamikaze_pos, duration=0.5s)

    # Wait brief interval
    sleep(0.5 seconds)

    # Update tracking
    PATROL.update_position(kamikaze_pos)
    PATROL.simulate_battery_drain(0.5s)
```

#### Intercept Behavior

```
High-frequency position updates (2 Hz):

T+0.0s: Target at (7.5, 2.0, 5.0) → PATROL moves to (7.5, 2.0, 5.5)
T+0.5s: Target at (7.3, 2.0, 5.0) → PATROL moves to (7.3, 2.0, 5.5)
T+1.0s: Target at (7.1, 2.0, 5.0) → PATROL moves to (7.1, 2.0, 5.5)
T+1.5s: Target at (6.9, 2.0, 5.0) → PATROL moves to (6.9, 2.0, 5.5)
...
T+5.0s: IMPACT - Target neutralized
```

#### Neutralization
- **Method**: Patrol drone positions 0.5m directly above target
- **Safety**: Simulated impact (drones stop before collision in real test)
- **Result**: Target mobility disabled, mobile patrol stopped

---

### Step 4: Mission Success

```
mobile_target.stop_patrol()  ← Stop autonomous movement
Log: "BOOM! TARGET NEUTRALIZED!"
Proceed to Phase 5
```

### Phase Metrics

| Metric | Value |
|--------|-------|
| Total engagement time | ~29.5 seconds |
| Initial approach | 4.5s |
| Jamming with tracking | 20s |
| Kamikaze intercept | 5s |
| Position updates (jamming) | 20 updates @ 1 Hz |
| Position updates (attack) | 10 updates @ 2 Hz |

---

## Phase 5: Mission Complete

### Objective
Safely land all drones and report mission status.

### Procedure

#### Step 1: Stop Mobile Target (if still running)
```python
if mobile_target:
    mobile_target.stop_patrol()
```

#### Step 2: Land All Drones
```
All drones: Land to 0.06m height
Duration: 3.0 seconds
Wait: 4.0 seconds for stabilization
```

#### Step 3: Mission Summary Report

```
=============================================================
MISSION SUMMARY
=============================================================
cf1:
  Role: LEADER (or FOLLOWER)
  Final Battery: 3.72V
  Target Detected: True

cf2:
  Role: FOLLOWER (or LEADER)
  Final Battery: 3.65V
  Target Detected: True

cf3:
  Role: PATROL
  Final Battery: 3.58V
  Target Detected: True

cf4:
  Role: TARGET
  Final Battery: 3.50V
  Target Detected: False
=============================================================
HACKATHON SCENARIO 3: SUCCESS (or FAILED)
=============================================================
```

### Mission Status

#### SUCCESS Conditions
- Target detected during patrol OR fallback scan
- Leader and Follower assigned successfully
- Dynamic engagement completed
- Kamikaze intercept executed
- All drones landed safely

#### FAILED Conditions
- Target not found after fallback scan (20s timeout)
- Mission status = FAILED
- Drones return to base and land

---

## Communication & Coordination

### ROS2 Topics

#### Drone Status Publishing (10 Hz)
Each drone publishes to: `/drone_status/{drone_name}`

```yaml
Message: DroneStatus
  header:
    stamp: current_time
    frame_id: "world"
  drone_name: "cf1" | "cf2" | "cf3" | "cf4"
  position:
    x: float
    y: float
    z: float
  battery_level: float (volts)
  target_detected: bool
  target_position:
    x: float
    y: float
    z: float
  role: "NEUTRAL" | "LEADER" | "FOLLOWER" | "PATROL" | "TARGET"
```

### Inter-Drone Information Sharing

#### Battery Levels
- Published continuously during mission
- Used for Leader/Follower selection
- Simulated drain: 0.01V per second

#### Target Position
- Updated when detected
- Broadcast to all drones
- Real-time updates during engagement (from mobile target)

#### Detection Status
- Binary flag: detected or not
- Shared across all drones
- Triggers phase transitions

---

## Key Differences from Scenario 1

| Aspect | Scenario 1 | Scenario 3 |
|--------|-----------|-----------|
| **Target Mobility** | Static at (7.5, 3.0, 5.0) | Dynamic 3m square patrol |
| **Detection Difficulty** | Easy (target stationary) | Challenging (target moving) |
| **Engagement Strategy** | Static positioning | Dynamic tracking required |
| **Fallback Behavior** | None | 3m line scanning |
| **Tracking Frequency** | N/A | 1 Hz jamming, 2 Hz attack |
| **Patrol Waypoints** | 4 waypoints (P) | 8 waypoints (P) for coverage |
| **Mission Complexity** | Baseline | Advanced |
| **Typical Duration** | ~3 minutes | ~3-4 minutes |

---

## Technical Implementation Details

### Mobile Target Control

```python
class MobileTarget:
    - Autonomous square patrol pattern
    - Runs in separate daemon thread
    - Uses time.sleep() (not timeHelper.sleep())
    - Waypoint navigation: 3.0s per segment
    - Continuous operation during entire mission
    - Thread-safe position queries
```

### Proximity Detection

```python
def detect_target(drone_position, target_position, threshold=1.5):
    distance = euclidean_distance(drone_position, target_position)
    return distance < threshold
```

### Dynamic Position Updates

```python
# Jamming phase (1 Hz)
for i in range(20):
    target_pos = mobile_target.get_current_position()
    calculate_engagement_positions(target_pos)
    send_movement_commands()
    sleep(1.0)

# Attack phase (2 Hz)
for i in range(10):
    target_pos = mobile_target.get_current_position()
    calculate_intercept_position(target_pos)
    send_movement_commands()
    sleep(0.5)
```

---

## Safety Features

### Emergency Landing
Triggered on any exception:
```python
try:
    run_mission()
except Exception as e:
    mobile_target.stop_patrol()
    emergency_land_all_drones()
```

### Thread Safety
- Mobile target runs in isolated thread
- No shared ROS executor usage
- Clean shutdown on mission end

### Boundary Checking
All waypoints respect cage dimensions:
- X: 0 to 10m
- Y: 0 to 6m
- Z: 0 to 8m

---

## Expected Timeline

```
T+0:00  Phase 1: Initialization begins
T+0:10  Phase 2: Patrol begins (target moving)
T+1:10  Phase 3: Target detected (typical)
T+1:17  Phase 4: Engagement begins
T+1:47  Phase 5: Kamikaze attack complete
T+1:52  Mission complete, landing

Total mission time: ~2 minutes
```

### Fallback Timeline (if detection fails)
```
T+0:00  Phase 1: Initialization begins
T+0:10  Phase 2: Patrol begins
T+2:10  Phase 3b: Fallback scan (patrol complete, no detection)
T+2:40  Mission FAILED or target detected during fallback
```

---

## Success Metrics

### Detection Performance
- **Primary detection zone**: X = 6-8m, Y = 1-5m (P's patrol area)
- **Detection probability**: ~90% during patrol phase
- **Fallback detection probability**: ~50% (depends on target position)

### Battery Consumption
- **Starting**: 4.0V ± 0.1V
- **After patrol**: ~3.8V
- **After engagement**: ~3.5-3.7V
- **Minimum safe level**: 3.5V

### Timing Accuracy
- **Initialization**: 10s ± 1s
- **Patrol**: 60-120s (variable based on detection)
- **Engagement**: ~30s
- **Total mission**: 2-3 minutes

---

## Troubleshooting

### Target Not Detected
**Cause**: Target position outside drone patrol paths
**Solution**: Fallback scan provides second opportunity

### Threading Errors
**Issue**: "generator already executing"
**Fix**: Use `time.sleep()` in background threads, not `timeHelper.sleep()`

### Position Tracking Lag
**Issue**: Drones lag behind moving target
**Solution**: Increase update frequency or reduce target speed

---

## Running the Scenario

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select crazyflie_examples

# Source the workspace
source install/setup.bash

# Run scenario 3
ros2 run crazyflie_examples hackathon_scenario_3
```

### Expected Console Output

```
[INFO] Hackathon Scenario 3 Coordinator initialized
[INFO] ============================================================
[INFO] STARTING HACKATHON SCENARIO 3 - MOBILE TARGET
[INFO] ============================================================
[INFO] PHASE 1: INITIALIZATION (10s)
[INFO] Starting mobile target patrol pattern...
[INFO] Initialization complete in 10.2s
[INFO] PHASE 2: PATROL (max 2 min)
[INFO] N_1 and N_2: Checking safety zone...
[INFO] P: Scanning area for mobile target...
[INFO] P moving to waypoint 1/8: [4.0, 5.0, 4.0]
...
[INFO] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[INFO] TARGET DETECTED BY PATROL DRONE!
[INFO] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[INFO] PHASE 3: DETECTION & ROLE ASSIGNMENT (7s)
[INFO] N_1 battery: 3.92V
[INFO] N_2 battery: 3.87V
[INFO] LEADER: cf1 (Battery: 3.92V)
[INFO] FOLLOWER: cf2 (Battery: 3.87V)
[INFO] PHASE 4: DYNAMIC ENGAGEMENT WITH MOVING TARGET
[INFO] Leader and Follower approaching moving target...
[INFO] ============================================================
[INFO] JAMMING TARGET - TRACKING MOVEMENT (20s)...
[INFO] ============================================================
[INFO] ============================================================
[INFO] PATROL DRONE: INITIATING KAMIKAZE ATTACK ON MOVING TARGET!
[INFO] ============================================================
[INFO] ************************************************************
[INFO] BOOM! TARGET NEUTRALIZED!
[INFO] ************************************************************
[INFO] MISSION COMPLETE - LANDING ALL DRONES
[INFO] ============================================================
[INFO] HACKATHON SCENARIO 3: SUCCESS
[INFO] ============================================================
```

---

## Conclusion

Scenario 3 demonstrates advanced autonomous drone coordination with:
- **Dynamic target tracking** during engagement
- **Autonomous mobile target** behavior
- **Fallback scanning** for robustness
- **Real-time position updates** for precision interception
- **Multi-phase coordination** between 4 drones

This scenario tests capabilities in:
- Computer vision (simulated proximity detection)
- Path planning (patrol patterns)
- Coordination (role assignment)
- Dynamic tracking (moving target engagement)
- Fault tolerance (fallback scanning)
