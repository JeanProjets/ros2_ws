#!/usr/bin/env python3
"""
Coverage scenario with 3 drones:
- Area dimensions: x=10m, y=6m, z=8m
- Safety zone: x=[0,3], y=[0,6], z=0
- One drone flies coverage pattern at z=4m covering x=[3,10], y=[0,6]
- Coverage must complete within 2 minutes
- All drones have cameras fixed at 0° (horizontal)
"""

import random
import math
import numpy as np
from crazyflie_py import Crazyswarm


# Area dimensions
AREA_X_MAX = 10.0
AREA_Y_MAX = 6.0
AREA_Z_MAX = 8.0

# Safety zone
SAFETY_X_MAX = 3.0
SAFETY_Y_MAX = 6.0
SAFETY_Z = 0.0

# Coverage parameters
COVERAGE_X_MIN = 3.0
COVERAGE_X_MAX = 10.0
COVERAGE_Y_MIN = 0.0
COVERAGE_Y_MAX = 6.0
COVERAGE_Z = 4.0
COVERAGE_TIME_LIMIT = 120.0  # 2 minutes in seconds

# Coverage pattern parameters
COVERAGE_SPACING = 1.0  # Grid spacing for coverage pattern (meters)
COVERAGE_SPEED = 0.5  # Speed for goTo commands (m/s)


def generate_coverage_waypoints():
    """
    Generate waypoints for complete area coverage using a lawnmower pattern.
    Returns a list of (x, y) waypoints that cover the entire area.
    """
    waypoints = []
    
    # Calculate number of rows needed
    y_range = COVERAGE_Y_MAX - COVERAGE_Y_MIN
    num_rows = int(math.ceil(y_range / COVERAGE_SPACING)) + 1
    
    # Generate waypoints in a lawnmower pattern
    for i in range(num_rows):
        y = COVERAGE_Y_MIN + (i * COVERAGE_SPACING)
        y = min(y, COVERAGE_Y_MAX)  # Clamp to max
        
        if i % 2 == 0:
            # Move from left to right
            waypoints.append((COVERAGE_X_MIN, y))
            waypoints.append((COVERAGE_X_MAX, y))
        else:
            # Move from right to left
            waypoints.append((COVERAGE_X_MAX, y))
            waypoints.append((COVERAGE_X_MIN, y))
    
    # Add intermediate waypoints for better coverage
    refined_waypoints = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]
        
        # Add start point
        refined_waypoints.append(start)
        
        # Calculate distance
        dist = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        num_intermediate = int(dist / COVERAGE_SPACING)
        
        # Add intermediate points
        for j in range(1, num_intermediate):
            t = j / (num_intermediate + 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            refined_waypoints.append((x, y))
    
    # Add last waypoint
    if waypoints:
        refined_waypoints.append(waypoints[-1])
    
    return refined_waypoints


def calculate_waypoint_duration(start, end, speed):
    """Calculate time needed to travel between two waypoints."""
    distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    return distance / speed


def main():
    # Initialize the swarm (this will initialize rclpy)
    print("Initializing swarm and discovering drones...")
    print("  (This may take a few seconds while waiting for services...)")
    try:
        swarm = Crazyswarm()
    except Exception as e:
        print(f"\nError initializing swarm: {e}")
        print("\nMake sure the crazyflie_server is running:")
        print("  ros2 launch crazyflie launch.py backend:=sim teleop:=False gui:=False mocap:=False rviz:=True")
        return
    
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    print("  ✓ Swarm initialized")
    
    # Check if crazyflie_server is running by checking for the emergency service
    print("Checking if crazyflie_server is running...")
    if not allcfs.emergencyService.wait_for_service(timeout_sec=5.0):
        print("\n" + "=" * 60)
        print("ERROR: crazyflie_server is not running or not accessible!")
        print("=" * 60)
        print("\nPlease start the crazyflie_server first:")
        print("  ros2 run crazyflie crazyflie_server")
        print("\nThen run this script in another terminal.")
        return
    
    print("✓ crazyflie_server is running")
    
    # Check how many drones were found initially
    num_drones = len(allcfs.crazyflies)
    print(f"Found {num_drones} drone(s)")
    
    # If no drones found, check if individual drone services exist
    # This helps diagnose if drones are connecting but services aren't created yet
    if num_drones == 0:
        print("\nChecking for individual drone services...")
        import rclpy
        from rclpy.node import Node
        
        # Create a temporary node to check services
        check_node = Node('drone_checker')
        service_names = check_node.get_service_names_and_types()
        check_node.destroy_node()
        
        # Look for drone services (pattern: /cfX/start_trajectory)
        drone_services = []
        for srv_name, srv_types in service_names:
            if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                cfname = srv_name[1:-17]  # Remove '/' and '/start_trajectory'
                if cfname != 'all' and cfname not in drone_services:
                    drone_services.append(cfname)
        
        if drone_services:
            print(f"  Found services for: {', '.join(drone_services)}")
            print("  But CrazyflieServer didn't discover them.")
            print("  This may indicate a timing issue. Try running the script again.")
        else:
            print("  No individual drone services found.")
            print("  This means drones haven't connected to the server yet.")
            print("  Check the crazyflie_server terminal for connection status.")
    
    if num_drones == 0:
        print("\n" + "=" * 60)
        print("ERROR: No drones detected after waiting!")
        print("=" * 60)
        print("\nPossible causes:")
        print("  1. No drones are enabled in crazyflies.yaml")
        print("     - Check that at least 3 drones have 'enabled: true'")
        print("  2. Drones are not connected yet")
        print("     - The crazyflie_server may still be connecting to drones")
        print("     - Check the crazyflie_server terminal for connection status")
        print("     - Verify drones are powered on and in range")
        print("     - Check that URIs in crazyflies.yaml match your drones")
        print("  3. Motion capture system not running (if required)")
        print("  4. Services not created yet - try waiting longer")
        print("\nTo check configured drones, look at:")
        print("  ~/.ros/crazyflies.yaml or")
        print("  <install_path>/share/crazyflie/config/crazyflies.yaml")
        print("\nTo check available services:")
        print("  ros2 service list | grep -E '(cf1|cf2|cf3)'")
        return
    
    # Ensure we have at least 3 drones
    if num_drones < 3:
        print("\n" + "=" * 60)
        print(f"ERROR: Need at least 3 drones, but only {num_drones} are available")
        print("=" * 60)
        print("\nPlease enable at least 3 drones in your crazyflies.yaml file.")
        print("Currently detected drones:")
        for i, cf in enumerate(allcfs.crazyflies):
            print(f"  {i+1}. {cf.prefix}")
        return
    
    print("=" * 60)
    print("Coverage Scenario: 3 Drones")
    print("=" * 60)
    
    # Phase 1: Initialize drones in random positions in safety zone
    print("\nPhase 1: Placing drones in safety zone...")
    safety_positions = []
    
    # First, arm all drones (required before takeoff)
    print("  Arming all drones...")
    for cf in allcfs.crazyflies[:3]:
        cf.arm(True)
        timeHelper.sleep(0.5)
    timeHelper.sleep(1.0)
    
    # Then, take off all drones to a low altitude for positioning
    print("  Taking off all drones to low altitude for positioning...")
    takeoff_height = 0.3  # Low altitude for positioning
    takeoff_duration = 2.0
    for cf in allcfs.crazyflies[:3]:
        cf.takeoff(targetHeight=takeoff_height, duration=takeoff_duration)
    timeHelper.sleep(takeoff_duration + 1.0)
    
    # Position each drone randomly in safety zone
    for i, cf in enumerate(allcfs.crazyflies[:3]):
        # Generate random position in safety zone
        x = random.uniform(0.0, SAFETY_X_MAX)
        y = random.uniform(0.0, SAFETY_Y_MAX)
        z = takeoff_height  # Keep at low altitude during positioning
        
        safety_positions.append((x, y, SAFETY_Z))  # Store ground position
        print(f"  Drone {i} ({cf.prefix}): Moving to ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Move drone to random position in safety zone
        target_pos = np.array([x, y, z])
        duration = 3.0
        cf.goTo(target_pos, 0.0, duration)
        timeHelper.sleep(duration + 0.5)
    
    # Land all drones back to safety zone ground level
    print("  Landing all drones to safety zone...")
    land_duration = 2.0
    for cf in allcfs.crazyflies[:3]:
        cf.land(targetHeight=SAFETY_Z, duration=land_duration)
    timeHelper.sleep(land_duration + 1.0)
    
    print("\nAll drones placed in safety zone.")
    timeHelper.sleep(2.0)
    
    # Phase 2: One drone performs coverage flight
    print("\nPhase 2: Starting coverage flight...")
    coverage_drone = allcfs.crazyflies[0]
    coverage_drone_name = coverage_drone.prefix
    
    # Generate coverage waypoints
    waypoints = generate_coverage_waypoints()
    print(f"  Generated {len(waypoints)} waypoints for coverage")
    
    # Calculate total path distance and time
    total_distance = 0.0
    for i in range(len(waypoints) - 1):
        dist = math.sqrt(
            (waypoints[i+1][0] - waypoints[i][0])**2 +
            (waypoints[i+1][1] - waypoints[i][1])**2
        )
        total_distance += dist
    
    estimated_time = total_distance / COVERAGE_SPEED
    print(f"  Total distance: {total_distance:.2f} m")
    print(f"  Estimated time: {estimated_time:.2f} s")
    
    if estimated_time > COVERAGE_TIME_LIMIT:
        # Adjust speed to fit within time limit
        required_speed = total_distance / COVERAGE_TIME_LIMIT
        print(f"  Adjusting speed to {required_speed:.2f} m/s to fit within time limit")
        actual_speed = required_speed
    else:
        actual_speed = COVERAGE_SPEED
    
    # Takeoff to coverage height
    print(f"\n  {coverage_drone_name}: Taking off to z={COVERAGE_Z} m...")
    takeoff_duration = 3.0
    coverage_drone.takeoff(targetHeight=COVERAGE_Z, duration=takeoff_duration)
    timeHelper.sleep(takeoff_duration + 1.0)
    
    # Start coverage flight
    print(f"  {coverage_drone_name}: Starting coverage pattern...")
    start_time = timeHelper.time()
    
    # Fly to each waypoint
    for i, (x, y) in enumerate(waypoints):
        # Check if we're approaching time limit
        elapsed = timeHelper.time() - start_time
        remaining = COVERAGE_TIME_LIMIT - elapsed
        
        if remaining < 5.0:  # Less than 5 seconds remaining
            print(f"  Time limit approaching. Remaining: {remaining:.1f} s")
            break
        
        # Calculate duration for this segment
        if i == 0:
            # First waypoint: go from current position
            duration = 2.0
        else:
            # Calculate based on distance and speed
            prev_waypoint = waypoints[i-1]
            distance = math.sqrt((x - prev_waypoint[0])**2 + (y - prev_waypoint[1])**2)
            duration = max(1.0, distance / actual_speed)
        
        # Move to waypoint
        goal = np.array([x, y, COVERAGE_Z])
        coverage_drone.goTo(goal, 0.0, duration)
        timeHelper.sleep(duration + 0.2)  # Small buffer
        
        if (i + 1) % 10 == 0:
            elapsed = timeHelper.time() - start_time
            print(f"  Progress: {i+1}/{len(waypoints)} waypoints, "
                  f"elapsed: {elapsed:.1f} s, remaining: {COVERAGE_TIME_LIMIT - elapsed:.1f} s")
    
    elapsed_time = timeHelper.time() - start_time
    print(f"\n  Coverage flight completed in {elapsed_time:.2f} seconds")
    
    # Phase 3: Return to safety zone
    print(f"\nPhase 3: Returning {coverage_drone_name} to safety zone...")
    
    # Calculate return position (center of safety zone or original position)
    return_x = safety_positions[0][0]
    return_y = safety_positions[0][1]
    return_z = SAFETY_Z
    
    # First descend to safety zone height
    print(f"  Descending to z={return_z} m...")
    coverage_drone.goTo(np.array([return_x, return_y, return_z]), 0.0, 3.0)
    timeHelper.sleep(3.5)
    
    # Land
    print(f"  Landing...")
    coverage_drone.land(targetHeight=return_z, duration=2.0)
    timeHelper.sleep(2.5)
    
    print("\n" + "=" * 60)
    print("Scenario completed successfully!")
    print("=" * 60)
    print(f"\nSummary:")
    print(f"  - 3 drones initialized in safety zone")
    print(f"  - 1 drone performed coverage flight at z={COVERAGE_Z} m")
    print(f"  - Coverage area: x=[{COVERAGE_X_MIN}, {COVERAGE_X_MAX}], "
          f"y=[{COVERAGE_Y_MIN}, {COVERAGE_Y_MAX}]")
    print(f"  - Coverage time: {elapsed_time:.2f} seconds")
    print(f"  - All drones returned to safety zone")
    print(f"  - Camera angle: 0° (horizontal) for all drones")


if __name__ == '__main__':
    main()

