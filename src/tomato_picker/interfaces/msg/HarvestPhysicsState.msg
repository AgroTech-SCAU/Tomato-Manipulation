# Fruit arrays share their order with the numbered fruit bodies in the MJCF
uint8 AVAILABLE=0
uint8 HELD=1
uint8 RELEASED=2
uint8 COLLECTED=3
uint8 DROPPED=4
std_msgs/Header header
geometry_msgs/Pose[] fruit_poses
geometry_msgs/Twist[] fruit_twists
uint8[] fruit_states
# One-based indices; zero means no selected or held fruit
uint32 selected_index
uint32 held_index
uint32 harvested_count
bool[] left_contacts
bool[] right_contacts
float64[] left_forces
float64[] right_forces
bool ready
# Increases after every scene operation and physical state transition
uint64 revision
string phase
geometry_msgs/PoseStamped initial_tcp
