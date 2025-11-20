# Custom message for hackathon scenario drone coordination

std_msgs/Header header

# Drone identification
string drone_name

# Position information
geometry_msgs/Point position

# Battery information
float32 battery_level  # Battery voltage [V]

# Target detection
bool target_detected
geometry_msgs/Point target_position

# Role assignment
string role  # NEUTRAL, LEADER, FOLLOWER, PATROL, TARGET
