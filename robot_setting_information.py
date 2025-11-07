import math,numpy as np

def sensor_position(robot_x,robot_y,robot_size,theta):
    sensor1_x = int(robot_x + (robot_size/2)*math.cos(math.radians(theta + 60)))
    sensor1_y = int(robot_y + (robot_size/2)*math.sin(math.radians(theta + 60)))
    sensor2_x = int(robot_x + (robot_size/2)*math.cos(math.radians(theta - 60)))
    sensor2_y = int(robot_y + (robot_size/2)*math.sin(math.radians(theta - 60)))
    return sensor1_x,sensor1_y,sensor2_x,sensor2_y

def robot_position(robot_x, robot_y, speed , theta, steer):
    theta = theta + steer
    robot_x = robot_x + speed * math.cos(math.radians(theta))
    robot_y = robot_y + speed * math.sin(math.radians(theta))
    return robot_x, robot_y, theta
