#!/usr/bin/env python

"""
Hackathon Scenario 3: Multi-Drone Search and Neutralize Mission - Mobile Target

Four drones coordinate to locate and neutralize a MOVING target:
- cf1 (N_1): Neutral drone 1
- cf2 (N_2): Neutral drone 2
- cf3 (P): Patrol drone
- cf4 (C): Target drone (MOBILE - moves in circular pattern)

Mission phases:
1. Initialization: All drones takeoff to 4m altitude, target starts moving
2. Patrol: N_1, N_2 check safety zone, P scans area
3. Detection: Proximity-based target detection (or fallback scanning)
4. Role Assignment: Highest battery neutral becomes Leader
5. Dynamic Engagement: Leader + Follower track moving target, jam, and neutralize
"""

from crazyflie_py import Crazyswarm
import numpy as np
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import DroneStatus
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import time
import threading
from enum import Enum


class DroneRole(Enum):
    """Enum for drone roles in the mission"""
    NEUTRAL = "NEUTRAL"
    PATROL = "PATROL"
    TARGET = "TARGET"
    LEADER = "LEADER"
    FOLLOWER = "FOLLOWER"


class MissionPhase(Enum):
    """Enum for mission phases"""
    INITIALIZATION = "INITIALIZATION"
    PATROL = "PATROL"
    DETECTION = "DETECTION"
    FALLBACK_SCAN = "FALLBACK_SCAN"
    ROLE_ASSIGNMENT = "ROLE_ASSIGNMENT"
    ENGAGEMENT = "ENGAGEMENT"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class DroneState:
    """Tracks the state of an individual drone"""

    def __init__(self, cf, name, role, initial_position):
        self.cf = cf
        self.name = name
        self.role = role
        self.initial_position = np.array(initial_position)
        self.current_position = np.array(initial_position)
        self.battery_level = 4.0  # Simulated battery voltage (V)
        self.target_detected = False
        self.target_position = None

    def update_position(self, position):
        """Update current position"""
        self.current_position = np.array(position)

    def simulate_battery_drain(self, time_delta):
        """Simulate battery drain over time (0.01V per second)"""
        self.battery_level -= 0.01 * time_delta
        self.battery_level = max(3.5, self.battery_level)  # Minimum voltage

    def detect_target(self, target_position, threshold=1.5):
        """Check if target is within detection range"""
        distance = np.linalg.norm(self.current_position - target_position)
        return distance < threshold


class MobileTarget:
    """Manages autonomous movement of the target drone in a circular/square pattern"""

    def __init__(self, drone_state, center, pattern_size=1.5):
        self.drone_state = drone_state
        self.center = np.array(center)  # Center of patrol area
        self.pattern_size = pattern_size  # Radius/half-size of pattern
        self.current_waypoint_idx = 0
        self.moving = False
        self.movement_thread = None
        self.stop_flag = threading.Event()

        # Define square waypoints around center (3m square = 1.5m radius)
        self.waypoints = [
            self.center + np.array([pattern_size, pattern_size, 0]),    # Top-right
            self.center + np.array([pattern_size, -pattern_size, 0]),   # Bottom-right
            self.center + np.array([-pattern_size, -pattern_size, 0]),  # Bottom-left
            self.center + np.array([-pattern_size, pattern_size, 0]),   # Top-left
        ]

    def start_patrol(self, timeHelper):
        """Start autonomous patrol movement"""
        if not self.moving:
            self.moving = True
            self.stop_flag.clear()
            self.movement_thread = threading.Thread(
                target=self._patrol_loop,
                args=(timeHelper,),
                daemon=True
            )
            self.movement_thread.start()

    def stop_patrol(self):
        """Stop autonomous patrol movement"""
        self.moving = False
        self.stop_flag.set()
        if self.movement_thread:
            self.movement_thread.join(timeout=2.0)

    def _patrol_loop(self, timeHelper):
        """Continuous patrol loop - moves between waypoints"""
        while self.moving and not self.stop_flag.is_set():
            # Get next waypoint
            target_wp = self.waypoints[self.current_waypoint_idx]

            # Move to waypoint
            self.drone_state.cf.goTo(target_wp, 0, 3.0)
            self.drone_state.update_position(target_wp)

            # Wait for movement to complete
            timeHelper.sleep(3.5)

            # Move to next waypoint
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)

            self.drone_state.simulate_battery_drain(3.5)

    def get_current_position(self):
        """Get current position of the target"""
        return self.drone_state.current_position.copy()


class ScenarioCoordinator(Node):
    """ROS2 node that coordinates hackathon scenario 3"""

    def __init__(self, swarm):
        super().__init__('hackathon_scenario_3_coordinator')

        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.allcfs = swarm.allcfs

        # Mission parameters from scenario spec
        self.CAGE_SIZE = np.array([10.0, 6.0, 8.0])  # X, Y, Z in meters
        self.SAFETY_ZONE_SIZE = 3.0  # meters
        self.FLIGHT_HEIGHT = 4.0  # meters (for N_1, N_2, P)
        self.TARGET_HEIGHT = 5.0  # meters (for C)
        self.DETECTION_THRESHOLD = 1.5  # meters
        self.SCAN_DURATION = 20.0  # seconds for fallback scanning

        # Scenario 3 positions from spec
        self.POSITIONS = {
            'N_1_start': np.array([2.5, 2.5, 0.0]),
            'N_2_start': np.array([2.5, 3.5, 0.0]),
            'P_start': np.array([3.0, 5.0, 0.0]),
            'C_center': np.array([6.5, 3.0, 5.0]),  # Center of patrol area
            # Fallback scanning positions
            'N_1_scan': np.array([3.0, 1.5, 4.0]),
            'N_2_scan': np.array([3.0, 3.0, 4.0]),
            'P_scan': np.array([3.0, 4.5, 4.0]),
        }

        # Initialize drone states
        self.drones = {}
        self._initialize_drones()

        # Mobile target controller
        self.mobile_target = None
        if 'cf4' in self.drones:
            self.mobile_target = MobileTarget(
                self.drones['cf4'],
                self.POSITIONS['C_center'],
                pattern_size=1.5  # 3m square = 1.5m from center
            )

        # Mission state
        self.current_phase = MissionPhase.INITIALIZATION
        self.target_found = False
        self.target_found_during_patrol = False

        # ROS2 publishers for each drone
        self.status_publishers = {}
        for name in self.drones.keys():
            topic = f'/drone_status/{name}'
            self.status_publishers[name] = self.create_publisher(DroneStatus, topic, 10)

        # Mission phase publisher
        self.phase_pub = self.create_publisher(DroneStatus, '/mission_coordinator/phase', 10)

        # Timer for publishing drone status
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10 Hz

        self.get_logger().info('Hackathon Scenario 3 Coordinator initialized')

    def _initialize_drones(self):
        """Initialize drone state objects"""
        # Assuming drones are in order: cf1, cf2, cf3, cf4
        drone_configs = [
            ('cf1', DroneRole.NEUTRAL, self.POSITIONS['N_1_start']),
            ('cf2', DroneRole.NEUTRAL, self.POSITIONS['N_2_start']),
            ('cf3', DroneRole.PATROL, self.POSITIONS['P_start']),
            ('cf4', DroneRole.TARGET, self.POSITIONS['C_center']),
        ]

        for idx, (name, role, pos) in enumerate(drone_configs):
            if idx < len(self.allcfs.crazyflies):
                cf = self.allcfs.crazyflies[idx]
                self.drones[name] = DroneState(cf, name, role, pos)
                # Add small random variation to battery levels
                self.drones[name].battery_level += np.random.uniform(-0.1, 0.1)
            else:
                self.get_logger().warn(f'Not enough drones configured for {name}')

    def publish_status(self):
        """Publish status for all drones at 10Hz"""
        current_time = self.get_clock().now().to_msg()

        for name, drone in self.drones.items():
            msg = DroneStatus()
            msg.header = Header()
            msg.header.stamp = current_time
            msg.header.frame_id = 'world'

            msg.drone_name = name
            msg.position = Point(
                x=drone.current_position[0],
                y=drone.current_position[1],
                z=drone.current_position[2]
            )
            msg.battery_level = drone.battery_level
            msg.target_detected = drone.target_detected

            if drone.target_position is not None:
                msg.target_position = Point(
                    x=drone.target_position[0],
                    y=drone.target_position[1],
                    z=drone.target_position[2]
                )
            else:
                msg.target_position = Point(x=0.0, y=0.0, z=0.0)

            msg.role = drone.role.value

            self.status_publishers[name].publish(msg)

    def run_mission(self):
        """Execute the complete mission sequence"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING HACKATHON SCENARIO 3 - MOBILE TARGET')
        self.get_logger().info('=' * 60)

        try:
            # Phase 1: Initialization
            self.phase_initialization()

            # Phase 2: Patrol
            self.phase_patrol()

            # Phase 3: Detection OR Fallback Scan
            if self.target_found_during_patrol:
                self.phase_detection_and_assignment()
            else:
                self.get_logger().warn('Target not found during patrol - initiating fallback scan')
                self.phase_fallback_scan()

            # Phase 4: Engagement (only if target found)
            if self.target_found:
                self.phase_engagement()
            else:
                self.get_logger().error('Mission failed - target not detected')
                self.current_phase = MissionPhase.FAILED

            # Phase 5: Mission Complete
            self.phase_complete()

        except Exception as e:
            self.get_logger().error(f'Mission failed: {str(e)}')
            import traceback
            traceback.print_exc()
            self.emergency_land()

    def phase_initialization(self):
        """Phase 1: Initialize drones and takeoff to flight height"""
        self.current_phase = MissionPhase.INITIALIZATION
        self.get_logger().info('PHASE 1: INITIALIZATION (10s)')

        start_time = time.time()

        # All drones except target takeoff to 4m
        for name in ['cf1', 'cf2', 'cf3']:
            if name in self.drones:
                drone = self.drones[name]
                drone.cf.takeoff(targetHeight=self.FLIGHT_HEIGHT, duration=3.0)

        # Target drone goes to center of patrol area at 5m
        if 'cf4' in self.drones:
            target = self.drones['cf4']
            target.cf.takeoff(targetHeight=self.TARGET_HEIGHT, duration=3.0)

        self.timeHelper.sleep(4.0)

        # Update drone positions after takeoff
        self.drones['cf1'].update_position([2.5, 2.5, 4.0])
        self.drones['cf2'].update_position([2.5, 3.5, 4.0])
        self.drones['cf3'].update_position([3.0, 5.0, 4.0])
        self.drones['cf4'].update_position(self.POSITIONS['C_center'])

        # Start mobile target patrol
        if self.mobile_target:
            self.get_logger().info('Starting mobile target patrol pattern...')
            self.mobile_target.start_patrol(self.timeHelper)
            self.timeHelper.sleep(1.0)

        elapsed = time.time() - start_time
        self.get_logger().info(f'Initialization complete in {elapsed:.1f}s')
        self.timeHelper.sleep(max(0, 10 - elapsed))

    def phase_patrol(self):
        """Phase 2: Patrol phase - search for mobile target"""
        self.current_phase = MissionPhase.PATROL
        self.get_logger().info('PHASE 2: PATROL (max 2 min)')

        start_time = time.time()

        # N_1 and N_2 check safety zone
        self.get_logger().info('N_1 and N_2: Checking safety zone...')

        # N_1 patrol pattern in safety zone
        n1_waypoints = [
            [1.5, 1.5, 4.0],
            [2.5, 1.5, 4.0],
            [2.5, 4.5, 4.0],
            [1.5, 4.5, 4.0],
            [2.5, 2.5, 4.0],  # Return to start
        ]

        # N_2 patrol pattern in safety zone (offset)
        n2_waypoints = [
            [1.5, 2.0, 4.0],
            [2.5, 2.0, 4.0],
            [2.5, 5.0, 4.0],
            [1.5, 5.0, 4.0],
            [2.5, 3.5, 4.0],  # Return to start
        ]

        # P patrol pattern - scan rest of area
        self.get_logger().info('P: Scanning area for mobile target...')
        p_waypoints = [
            [4.0, 5.0, 4.0],
            [6.0, 5.0, 4.0],
            [8.0, 5.0, 4.0],
            [8.0, 3.0, 4.0],
            [8.0, 1.0, 4.0],
            [6.0, 1.0, 4.0],
            [4.0, 3.0, 4.0],
        ]

        # Execute patrol movements
        # Move N_1 and N_2 through safety zone
        for i, (wp1, wp2) in enumerate(zip(n1_waypoints[:2], n2_waypoints[:2])):
            self.drones['cf1'].cf.goTo(wp1, 0, 2.0)
            self.drones['cf2'].cf.goTo(wp2, 0, 2.0)
            self.timeHelper.sleep(2.5)

            self.drones['cf1'].update_position(wp1)
            self.drones['cf2'].update_position(wp2)
            self.drones['cf1'].simulate_battery_drain(2.5)
            self.drones['cf2'].simulate_battery_drain(2.5)

            # Check if N_1 or N_2 detected target (unlikely in safety zone but possible)
            target_pos = self.mobile_target.get_current_position() if self.mobile_target else None
            if target_pos is not None:
                if self.drones['cf1'].detect_target(target_pos, self.DETECTION_THRESHOLD):
                    self.get_logger().info('!' * 60)
                    self.get_logger().info('TARGET DETECTED BY N_1!')
                    self.get_logger().info('!' * 60)
                    self.drones['cf1'].target_detected = True
                    self.drones['cf1'].target_position = target_pos
                    self.target_found = True
                    self.target_found_during_patrol = True
                    return

                if self.drones['cf2'].detect_target(target_pos, self.DETECTION_THRESHOLD):
                    self.get_logger().info('!' * 60)
                    self.get_logger().info('TARGET DETECTED BY N_2!')
                    self.get_logger().info('!' * 60)
                    self.drones['cf2'].target_detected = True
                    self.drones['cf2'].target_position = target_pos
                    self.target_found = True
                    self.target_found_during_patrol = True
                    return

        # P moves through patrol waypoints until target detected
        for i, waypoint in enumerate(p_waypoints):
            self.get_logger().info(f'P moving to waypoint {i+1}/{len(p_waypoints)}: {waypoint}')
            self.drones['cf3'].cf.goTo(waypoint, 0, 3.0)
            self.timeHelper.sleep(3.5)

            # Update position and battery
            self.drones['cf3'].update_position(waypoint)
            self.drones['cf3'].simulate_battery_drain(3.5)

            # Check for target detection (target is moving!)
            if self.mobile_target:
                target_pos = self.mobile_target.get_current_position()
                if self.drones['cf3'].detect_target(target_pos, self.DETECTION_THRESHOLD):
                    self.get_logger().info('!' * 60)
                    self.get_logger().info('TARGET DETECTED BY PATROL DRONE!')
                    self.get_logger().info('!' * 60)
                    self.drones['cf3'].target_detected = True
                    self.drones['cf3'].target_position = target_pos
                    self.target_found = True
                    self.target_found_during_patrol = True
                    break

        elapsed = time.time() - start_time
        self.get_logger().info(f'Patrol phase complete in {elapsed:.1f}s')

    def phase_fallback_scan(self):
        """Phase 3b: Fallback scanning if target not found during patrol"""
        self.current_phase = MissionPhase.FALLBACK_SCAN
        self.get_logger().info('PHASE 3b: FALLBACK SCAN (30s)')
        self.get_logger().info('All drones moving to 3m line for scanning...')

        start_time = time.time()

        # Move all three drones to scanning positions
        self.drones['cf1'].cf.goTo(self.POSITIONS['N_1_scan'], 0, 3.0)
        self.drones['cf2'].cf.goTo(self.POSITIONS['N_2_scan'], 0, 3.0)
        self.drones['cf3'].cf.goTo(self.POSITIONS['P_scan'], 0, 3.0)

        self.timeHelper.sleep(3.5)

        self.drones['cf1'].update_position(self.POSITIONS['N_1_scan'])
        self.drones['cf2'].update_position(self.POSITIONS['N_2_scan'])
        self.drones['cf3'].update_position(self.POSITIONS['P_scan'])

        for name in ['cf1', 'cf2', 'cf3']:
            self.drones[name].simulate_battery_drain(3.5)

        self.get_logger().info('Scanning hostile zone from 3m line...')

        # Hover and scan for specified duration
        scan_checks = int(self.SCAN_DURATION / 0.5)  # Check every 0.5s
        for i in range(scan_checks):
            self.timeHelper.sleep(0.5)

            # Check if any drone detects target
            if self.mobile_target:
                target_pos = self.mobile_target.get_current_position()

                for name in ['cf1', 'cf2', 'cf3']:
                    drone = self.drones[name]
                    if drone.detect_target(target_pos, self.DETECTION_THRESHOLD * 1.5):  # Slightly larger range
                        self.get_logger().info('!' * 60)
                        self.get_logger().info(f'TARGET DETECTED BY {name} DURING FALLBACK SCAN!')
                        self.get_logger().info('!' * 60)
                        drone.target_detected = True
                        drone.target_position = target_pos
                        self.target_found = True

                        # Broadcast to all drones
                        for other_name in ['cf1', 'cf2', 'cf3']:
                            self.drones[other_name].target_position = target_pos
                            self.drones[other_name].target_detected = True

                        self.phase_detection_and_assignment()
                        return

            # Simulate battery drain
            for name in ['cf1', 'cf2', 'cf3']:
                self.drones[name].simulate_battery_drain(0.5)

        elapsed = time.time() - start_time
        self.get_logger().info(f'Fallback scan complete in {elapsed:.1f}s - Target not found')
        self.get_logger().warn('MISSION FAILED - Returning to base')

    def phase_detection_and_assignment(self):
        """Phase 3: Target detected, assign Leader and Follower roles"""
        self.current_phase = MissionPhase.ROLE_ASSIGNMENT
        self.get_logger().info('PHASE 3: DETECTION & ROLE ASSIGNMENT (7s)')

        start_time = time.time()

        if not self.target_found:
            self.get_logger().warn('Target not detected!')
            return

        # Get current target position
        if self.mobile_target:
            current_target_pos = self.mobile_target.get_current_position()
        else:
            current_target_pos = self.drones['cf4'].current_position

        # Broadcast target position to all drones
        self.get_logger().info(f'Broadcasting target position: {current_target_pos}')
        self.timeHelper.sleep(1.0)

        # Update N_1 and N_2 with target info
        self.drones['cf1'].target_position = current_target_pos
        self.drones['cf2'].target_position = current_target_pos
        self.drones['cf1'].target_detected = True
        self.drones['cf2'].target_detected = True

        # Determine Leader and Follower based on battery level
        n1_battery = self.drones['cf1'].battery_level
        n2_battery = self.drones['cf2'].battery_level

        self.get_logger().info(f'N_1 battery: {n1_battery:.2f}V')
        self.get_logger().info(f'N_2 battery: {n2_battery:.2f}V')

        if n1_battery > n2_battery:
            leader_name = 'cf1'
            follower_name = 'cf2'
        else:
            leader_name = 'cf2'
            follower_name = 'cf1'

        self.drones[leader_name].role = DroneRole.LEADER
        self.drones[follower_name].role = DroneRole.FOLLOWER

        self.get_logger().info(f'LEADER: {leader_name} (Battery: {self.drones[leader_name].battery_level:.2f}V)')
        self.get_logger().info(f'FOLLOWER: {follower_name} (Battery: {self.drones[follower_name].battery_level:.2f}V)')

        elapsed = time.time() - start_time
        self.timeHelper.sleep(max(0, 7 - elapsed))

    def phase_engagement(self):
        """Phase 4: Leader and Follower engage moving target, P performs kamikaze"""
        self.current_phase = MissionPhase.ENGAGEMENT
        self.get_logger().info('PHASE 4: DYNAMIC ENGAGEMENT WITH MOVING TARGET')

        start_time = time.time()

        # Find Leader and Follower
        leader = None
        follower = None
        for drone in self.drones.values():
            if drone.role == DroneRole.LEADER:
                leader = drone
            elif drone.role == DroneRole.FOLLOWER:
                follower = drone

        if not leader or not follower:
            self.get_logger().error('Leader or Follower not assigned!')
            return

        self.get_logger().info('Leader and Follower approaching moving target...')

        # Initial approach - move to engagement distance
        if self.mobile_target:
            target_pos = self.mobile_target.get_current_position()
        else:
            target_pos = self.drones['cf4'].current_position

        leader_engage_pos = target_pos + np.array([-1.5, 0, 0])
        follower_offset = np.array([-0.5, -0.5, -0.5])
        follower_engage_pos = leader_engage_pos + follower_offset

        leader.cf.goTo(leader_engage_pos, 0, 4.0)
        follower.cf.goTo(follower_engage_pos, 0, 4.0)
        self.timeHelper.sleep(4.5)

        leader.update_position(leader_engage_pos)
        follower.update_position(follower_engage_pos)
        leader.simulate_battery_drain(4.5)
        follower.simulate_battery_drain(4.5)

        # Dynamic tracking during jamming phase
        self.get_logger().info('=' * 60)
        self.get_logger().info('JAMMING TARGET - TRACKING MOVEMENT (20s)...')
        self.get_logger().info('=' * 60)

        jamming_duration = 20.0
        tracking_interval = 1.0  # Update position every 1 second
        tracking_iterations = int(jamming_duration / tracking_interval)

        for i in range(tracking_iterations):
            # Get current target position
            if self.mobile_target:
                target_pos = self.mobile_target.get_current_position()
            else:
                target_pos = self.drones['cf4'].current_position

            # Calculate new engagement positions relative to moving target
            leader_engage_pos = target_pos + np.array([-1.5, 0, 0])
            follower_engage_pos = leader_engage_pos + follower_offset

            # Send movement commands
            leader.cf.goTo(leader_engage_pos, 0, tracking_interval)
            follower.cf.goTo(follower_engage_pos, 0, tracking_interval)

            self.timeHelper.sleep(tracking_interval)

            # Update positions and battery
            leader.update_position(leader_engage_pos)
            follower.update_position(follower_engage_pos)
            leader.simulate_battery_drain(tracking_interval)
            follower.simulate_battery_drain(tracking_interval)
            self.drones['cf3'].simulate_battery_drain(tracking_interval)

        # Kamikaze attack - P intercepts moving target
        self.get_logger().info('=' * 60)
        self.get_logger().info('PATROL DRONE: INITIATING KAMIKAZE ATTACK ON MOVING TARGET!')
        self.get_logger().info('=' * 60)

        # Track and intercept target
        intercept_duration = 5.0
        intercept_updates = int(intercept_duration / 0.5)

        for i in range(intercept_updates):
            if self.mobile_target:
                target_pos = self.mobile_target.get_current_position()
            else:
                target_pos = self.drones['cf4'].current_position

            kamikaze_pos = target_pos + np.array([0, 0, 0.5])  # 0.5m above target
            self.drones['cf3'].cf.goTo(kamikaze_pos, 0, 0.5)
            self.timeHelper.sleep(0.5)

            self.drones['cf3'].update_position(kamikaze_pos)
            self.drones['cf3'].simulate_battery_drain(0.5)

        # Stop mobile target
        if self.mobile_target:
            self.mobile_target.stop_patrol()

        self.get_logger().info('*' * 60)
        self.get_logger().info('BOOM! TARGET NEUTRALIZED!')
        self.get_logger().info('*' * 60)

        elapsed = time.time() - start_time
        self.get_logger().info(f'Engagement phase complete in {elapsed:.1f}s')

    def phase_complete(self):
        """Phase 5: Mission complete, land all drones"""
        self.current_phase = MissionPhase.COMPLETE
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION COMPLETE - LANDING ALL DRONES')
        self.get_logger().info('=' * 60)

        # Stop mobile target if still running
        if self.mobile_target:
            self.mobile_target.stop_patrol()

        # Land all drones
        self.allcfs.land(targetHeight=0.06, duration=3.0)
        self.timeHelper.sleep(4.0)

        # Print mission summary
        self.print_mission_summary()

    def print_mission_summary(self):
        """Print summary of mission results"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION SUMMARY')
        self.get_logger().info('=' * 60)

        for name, drone in self.drones.items():
            self.get_logger().info(f'{name}:')
            self.get_logger().info(f'  Role: {drone.role.value}')
            self.get_logger().info(f'  Final Battery: {drone.battery_level:.2f}V')
            self.get_logger().info(f'  Target Detected: {drone.target_detected}')

        self.get_logger().info('=' * 60)
        if self.current_phase == MissionPhase.FAILED:
            self.get_logger().info('HACKATHON SCENARIO 3: FAILED')
        else:
            self.get_logger().info('HACKATHON SCENARIO 3: SUCCESS')
        self.get_logger().info('=' * 60)

    def emergency_land(self):
        """Emergency landing for all drones"""
        self.get_logger().error('EMERGENCY LANDING!')

        # Stop mobile target
        if self.mobile_target:
            self.mobile_target.stop_patrol()

        try:
            self.allcfs.land(targetHeight=0.06, duration=2.0)
            self.timeHelper.sleep(3.0)
        except Exception as e:
            self.get_logger().error(f'Emergency landing failed: {str(e)}')


def main():
    """Main entry point for hackathon scenario 3"""
    # Initialize Crazyswarm
    swarm = Crazyswarm()

    # Create scenario coordinator
    coordinator = ScenarioCoordinator(swarm)

    # Run the mission
    coordinator.run_mission()

    # Cleanup
    coordinator.destroy_node()


if __name__ == '__main__':
    main()
