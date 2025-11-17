import cv2, math, numpy as np
from robot_setting_information import robot_position

# -----------------------------
# Initialization
# -----------------------------
robot_x = 80
robot_y = 70
robot_size = 20
theta = 90   # Robot heading

# sensor angles (independent)
sensor1_angle = 90     # clockwise
sensor2_angle = 90     # counter-clockwise
sensor_rotate_speed = 13

speed = 1

# Detection control
detected_this_rotation = False     # 1回転につき1回だけ検出
rotation_start_angle = 90          # 回転開始角度
ROTATION_RANGE = 360               # 360°回ったらリセット

course_image = cv2.imread('course.png', cv2.IMREAD_GRAYSCALE)


def get_sensor_pos(robot_x, robot_y, radius, angle):
    x = int(robot_x + radius * math.cos(math.radians(angle)))
    y = int(robot_y + radius * math.sin(math.radians(angle)))
    return x, y


def has_completed_rotation(current_angle, start_angle):
    """Start_angle → current_angle が1周したかどうか判定"""
    # センサー角度は 0〜359 で循環するので差をmodで計算
    diff = (current_angle - start_angle) % 360
    return diff >= ROTATION_RANGE - sensor_rotate_speed


# -----------------------------
# Main loop
# -----------------------------
while True:

    img = course_image.copy()

    # -----------------------------
    # RESET FOR NEW ROTATION
    # -----------------------------
    if has_completed_rotation(sensor1_angle, rotation_start_angle):
        detected_this_rotation = False
        rotation_start_angle = sensor1_angle

    # -----------------------------
    # SENSOR ROTATION
    # -----------------------------
    sensor1_angle = (sensor1_angle + sensor_rotate_speed) % 360
    sensor2_angle = (sensor2_angle - sensor_rotate_speed) % 360

    # Sensor positions
    s1x, s1y = get_sensor_pos(robot_x, robot_y, robot_size, sensor1_angle)
    s2x, s2y = get_sensor_pos(robot_x, robot_y, robot_size, sensor2_angle)

    # brightness
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
    # SENSOR DECISION (LIMITED TO 1 PER ROTATION)
    # -----------------------------

    if not detected_this_rotation:   # ← 1回転に1回まで！

        # sensor1 が先に検出
        if sensor1_val < 50:
            theta = sensor1_angle
            detected_this_rotation = True

        # sensor2 が先に検出（sensor1 が未検出の場合のみ）
        elif sensor2_val < 50:
            theta = sensor2_angle
            detected_this_rotation = True

    # -----------------------------
    # MOVE
    # -----------------------------
    if detected_this_rotation:
        robot_x, robot_y, theta = robot_position(robot_x, robot_y, speed, theta, 0)

cv2.destroyAllWindows()
