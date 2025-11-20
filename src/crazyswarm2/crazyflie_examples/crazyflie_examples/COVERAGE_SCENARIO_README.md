# Coverage Scenario - 3 Drones

This script implements a coverage scenario with 3 drones in a defined area.

## Scenario Description

- **Area dimensions**: x=10m, y=6m, z=8m
- **Safety zone**: x=[0,3], y=[0,6], z=0 (ground level)
- **Coverage area**: x=[3,10], y=[0,6], z=4m (constant altitude)
- **Time limit**: Maximum 2 minutes for coverage flight
- **Camera angle**: 0° (horizontal) for all drones

## Phases

1. **Initialization Phase**: All 3 drones are placed randomly in the safety zone at ground level (z=0)
2. **Coverage Phase**: One drone (drone 0) takes off and flies a lawnmower pattern at z=4m to cover the entire area x=[3,10], y=[0,6] within 2 minutes
3. **Return Phase**: The coverage drone returns to its original position in the safety zone and lands

## Prerequisites

1. **3 Drones Configured**: Ensure your `crazyflies.yaml` file has at least 3 drones enabled
2. **Motion Capture System**: The drones need external positioning (mocap) to track their positions
3. **Crazyflie Server Running**: Start the crazyflie_server node before running this script

## Configuration

### Example `crazyflies.yaml` Configuration

```yaml
fileversion: 3

robots:
  cf1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E701
    initial_position: [0.0, 0.0, 0.0]
    type: cf21
    reference_frame: "world"

  cf2:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E702
    initial_position: [1.0, 0.0, 0.0]
    type: cf21
    reference_frame: "world"

  cf3:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E703
    initial_position: [2.0, 0.0, 0.0]
    type: cf21
    reference_frame: "world"

robot_types:
  cf21:
    motion_capture:
      tracking: "librigidbodytracker"
      marker: default_single_marker
      dynamics: default
    big_quad: false
    battery:
      voltage_warning: 3.8
      voltage_critical: 3.7

all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10
      status:
        frequency: 1
  firmware_params:
    commander:
      enHighLevel: 1
    stabilizer:
      estimator: 2
      controller: 2
    locSrv:
      extPosStdDev: 1e-3
      extQuatStdDev: 0.5e-1
  reference_frame: "world"
```

## Usage

1. **Build the Package** (if you just added the script):
   ```bash
   cd /path/to/ros2_ws
   colcon build --packages-select crazyflie_examples
   source install/setup.bash
   ```

2. **Start the Crazyflie Server** (using the launch file):
   
   **Option A: With Hardware (Crazyradio PA dongle required)**
   ```bash
   ros2 launch crazyflie launch.py teleop:=False gui:=False mocap:=False
   ```
   
   **Option B: Simulation Mode (No hardware required - recommended for testing)**
   
   **Without visualization:**
   ```bash
   ros2 launch crazyflie launch.py backend:=sim teleop:=False gui:=False mocap:=False
   ```
   
   **With RViz visualization (recommended):**
   ```bash
   ros2 launch crazyflie launch.py backend:=sim teleop:=False gui:=False mocap:=False rviz:=True
   ```
   
   **With GUI (alternative visualization):**
   ```bash
   ros2 launch crazyflie launch.py backend:=sim teleop:=False gui:=True mocap:=False
   ```
   
   **Note**: 
   - The server needs to be launched with the launch file (not `ros2 run`) 
     so it receives the `crazyflies.yaml` configuration as parameters.
   - If you get "No Crazyradio dongle found!" error, use `backend:=sim` for simulation mode.
   - Simulation mode allows you to test the scenario without physical drones.

3. **Run the Coverage Scenario**:
   ```bash
   ros2 run crazyflie_examples coverage_scenario
   ```

   Or directly:
   ```bash
   python3 /path/to/crazyflie_examples/crazyflie_examples/coverage_scenario.py
   ```

## Parameters

You can modify the following parameters in the script:

- `COVERAGE_SPACING`: Grid spacing for coverage pattern (default: 1.0 m)
- `COVERAGE_SPEED`: Base speed for goTo commands (default: 0.5 m/s)
- `COVERAGE_Z`: Altitude for coverage flight (default: 4.0 m)
- `COVERAGE_TIME_LIMIT`: Maximum time for coverage (default: 120.0 s)

## Coverage Pattern

The script uses a lawnmower pattern to ensure complete coverage of the area:
- Waypoints are generated in a grid pattern with specified spacing
- The pattern alternates direction (left-to-right, then right-to-left) for efficiency
- Intermediate waypoints are added for better coverage
- Speed is automatically adjusted if needed to complete within the time limit

## Safety Notes

- Ensure the flight area is clear of obstacles
- Verify that all drones are properly calibrated and have sufficient battery
- The safety zone (x=[0,3]) is where drones start and return to
- The coverage drone flies at a constant altitude of 4m to avoid obstacles
- All drones have cameras fixed at 0° (horizontal) as specified

## Troubleshooting

- **"Need at least 3 drones"**: Check your `crazyflies.yaml` and ensure at least 3 drones are enabled
- **Connection issues**: Verify that the crazyflie_server is running and can connect to all drones
- **Positioning errors**: Ensure motion capture system is properly configured and tracking all drones
- **Time limit exceeded**: The script will attempt to complete coverage but may skip waypoints if time is running out

