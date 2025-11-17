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
theta = 90   # ロボットの進行方向角度

# センサー角度（ロボット進行方向とは独立に回転）
sensor1_angle = 90     # 時計回りに回転
sensor2_angle = 90     # 反時計回りに回転
sensor_rotate_speed = 17  # フレーム毎の回転速度

speed = 6
sampling = 0.05
savedata = []
t = 0.0
# コース画像の読み込み
course_image = cv2.imread('course.png', cv2.IMREAD_GRAYSCALE)

def get_sensor_pos(robot_x, robot_y, radius, angle):
    """センサー角度に基づいてロボットからの相対座標を計算"""
    x = int(robot_x + radius * math.cos(math.radians(angle)))
    y = int(robot_y + radius * math.sin(math.radians(angle)))
    return x, y


def is_front(sensor_angle, robot_angle):
    """
    センサー角度がロボット前方180°以内なら True
    後方180°なら False
    """
    diff = (sensor_angle - robot_angle + 540) % 360 - 180
    return abs(diff) <= 90

# -----------------------------
# Main Loop
# -----------------------------
while True:

    # グレースケール画像をコピー
    img = course_image.copy()

    # センサー角度の更新
    sensor1_angle = (sensor1_angle + sensor_rotate_speed) % 360
    sensor2_angle = (sensor2_angle - sensor_rotate_speed) % 360

    # センサー座標の計算
    s1x, s1y = get_sensor_pos(robot_x, robot_y, robot_size, sensor1_angle)
    s2x, s2y = get_sensor_pos(robot_x, robot_y, robot_size, sensor2_angle)

    # コースの明度を取得（画面外は None）
    height, width = img.shape[:2]
    def read_sensor_value(x, y):
        if 0 <= x < width and 0 <= y < height:
            return img[y, x]
        return None

    sensor1_val = read_sensor_value(s1x, s1y)
    sensor2_val = read_sensor_value(s2x, s2y)

    # ロボット本体とセンサー位置を描画
    cv2.circle(img, (int(robot_x), int(robot_y)), robot_size, 180, -1)
    cv2.circle(img, (s1x, s1y), 4, 30, -1)
    cv2.circle(img, (s2x, s2y), 4, 30, -1)

    cv2.imshow("LineTrace", img)

    if cv2.waitKey(10) == 27:
        break

    # -----------------------------
    # Sensor Decision Logic
    # -----------------------------
    found_angle = None

    # センサー1判定
    if sensor1_val is not None and sensor1_val < 50 and is_front(sensor1_angle, theta) and found_angle is None:
        found_angle = sensor1_angle

    # センサー2判定
    if sensor2_val is not None and sensor2_val < 50 and is_front(sensor2_angle, theta):
        if found_angle is None:
            found_angle = sensor2_angle
        else:
            diff_current = abs(((found_angle - theta + 540) % 360) - 180)
            diff_new = abs(((sensor2_angle - theta + 540) % 360) - 180)
            if diff_new < diff_current:
                found_angle = sensor2_angle

    # 有効な検出があればロボットを移動
    if found_angle is not None:
        theta = found_angle
        robot_x, robot_y, theta = robot_position(robot_x, robot_y, speed, theta, 0)

    # ログを保存
    sd = [
        t,
        sensor1_val if sensor1_val is not None else np.nan,
        sensor2_val if sensor2_val is not None else np.nan,
        found_angle if found_angle is not None else np.nan,
    ]
    savedata.append(sd)
    t += sampling

cv2.destroyAllWindows()

# -----------------------------
# Data Saving and Plotting
# -----------------------------
print('Simulation Ended')

if savedata:
    savedata_np = np.array(savedata, dtype=float)

    savedata_name = 'test.csv'
    np.savetxt(savedata_name, savedata_np, delimiter=',')

    time_data = savedata_np[:, 0]
    deviation_data = savedata_np[:, 3]

    fig = plt.figure()
    plt.plot(time_data, deviation_data)
    plt.ylim([-200, 200])

    savedata_name = 'ON_OFF比例制御.png'
    fig.savefig(savedata_name)

    plt.show()