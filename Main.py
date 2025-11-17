import cv2, math, numpy as np
import matplotlib.pyplot as plt
from time import sleep
from robot_setting_information import robot_position

# -----------------------------
# Initialization
# -----------------------------
robot_x = 80
robot_y = 70
robot_size = 20
theta = 90   # Robot’s heading angle

# sensor angles (independent from robot angle)
sensor1_angle = 90     # rotates clockwise
sensor2_angle = 90    # rotates counter-clockwise
sensor_rotate_speed = 5  # degrees per frame

speed = 5
sampling = 0.05

# Load course
course_image = cv2.imread('course.png', cv2.IMREAD_GRAYSCALE)

def get_sensor_pos(robot_x, robot_y, radius, angle):
    """Compute sensor position based on independent rotating angle"""
    x = int(robot_x + radius * math.cos(math.radians(angle)))
    y = int(robot_y + radius * math.sin(math.radians(angle)))
    return x, y

# -----------------------------
# Main loop
# -----------------------------
while True:

    # Reload grayscale image
    img = course_image.copy()

    # Rotate sensors
    sensor1_angle = (sensor1_angle + sensor_rotate_speed) % 360
    sensor2_angle = (sensor2_angle - sensor_rotate_speed) % 360

    # Compute sensor positions
    s1x, s1y = get_sensor_pos(robot_x, robot_y, robot_size, sensor1_angle)
    s2x, s2y = get_sensor_pos(robot_x, robot_y, robot_size, sensor2_angle)

    # Read brightness
    sensor1_val = img[s1y, s1x]
    sensor2_val = img[s2y, s2x]

    # Draw robot + sensors
    cv2.circle(img, (int(robot_x), int(robot_y)), robot_size, 180, -1)
    cv2.circle(img, (s1x, s1y), 4, 30, -1)
    cv2.circle(img, (s2x, s2y), 4, 30, -1)

    cv2.imshow("LineTrace", img)

    if cv2.waitKey(10) == 27:
        break

    # -----------------------------
    # SENSOR DECISION LOGIC
    # -----------------------------
    found_line = False

    # If sensor1 detects the line (black-ish)
    if sensor1_val < 50:
        theta = sensor1_angle   # rotate robot toward sensor1’s direction
        found_line = True

    # If sensor2 detects the line
    if sensor2_val < 50:
        theta = sensor2_angle
        found_line = True

    # If either sensor found the line → move forward
    if found_line:
        robot_x, robot_y, theta = robot_position(robot_x, robot_y, speed, theta, 0)

    # If both sensors see nothing → keep scanning (no forward)
    # so do nothing, just continue the loop

cv2.destroyAllWindows()