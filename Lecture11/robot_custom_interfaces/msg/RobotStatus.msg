# Robot operational mode constants
uint8 MODE_IDLE=0
uint8 MODE_MOVING=1
uint8 MODE_WORKING=2
uint8 MODE_CHARGING=3
uint8 MODE_ERROR=4

# Standard ROS header with timestamp and frame
std_msgs/Header header

# Robot identification and current mode
string robot_name
uint8 mode # Use mode constants above

# Sensor readings (temperature, humidity, etc.)
float32[] sensor_readings