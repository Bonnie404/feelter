# Standard Header
std_msgs/Header header

# Optical Flow image
# The image represents the optical flow field.
# Each pixel corresponds to a motion vector (dx, dy).
# - Image encoding is '32FC2'
# - Each pixel contains two float32 values:
#   - channel 0: motion in x direction
#   - channel 1: motion in y direction
sensor_msgs/Image flow_image