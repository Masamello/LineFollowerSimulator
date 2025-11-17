import cv2,math,numpy as np
import matplotlib.pyplot as plt
from time import sleep
from robot_setting_information import sensor_position, robot_position

# ####################### Initialization #######################
# Starting position of the robot
robot_x = 80
robot_y = 70
robot_size = 20
theta = 90  # Initial angle of the robot (90 degrees = pointing upward)

# Calculate initial positions of left and right sensors
sensor1_x , sensor1_y, sensor2_x, sensor2_y = sensor_position(robot_x,robot_y,robot_size,theta)

# Robot motion parameters #######################
speed = 10        # Forward speed of the robot
threshold = 150   # Brightness threshold for line detection
sampling = 0.05   # Time step between iterations
zero_count = 0    # Counter for detecting when robot goes off course

# Proportional control parameters
Kp = 0.3  # Proportional gain - higher values make steering more aggressive

# Control function that calculates steering based on sensor difference
def robot_action(control_deviation):
    # Calculate steering angle using proportional control
    # Positive deviation turns right, negative turns left
    steer = Kp * control_deviation
    return steer

# Main simulation loop #######################
t = 0  # Time counter
savedata = [0,0,0,0]  # Initialize data storage [time, sensor1, sensor2, deviation]
course_image = cv2.imread('course.png',flags=cv2.IMREAD_GRAYSCALE)

while True:
    # Read course image and get sensor values
    img = cv2.imread("course.png",flags=cv2.IMREAD_GRAYSCALE)
    sensor1_value = int(img[sensor1_y, sensor1_x]) #left sensor value (0-255)
    sensor2_value = int(img[sensor2_y, sensor2_x]) #right sensor value (0-255)
    
    # Calculate difference between sensors for steering
    control_deviation = int(sensor2_value) - int(sensor1_value)

    # Record current state
    sd = [t, sensor1_value, sensor2_value, control_deviation]
    savedata = np.vstack([savedata, sd])

    # Draw robot and sensors on the image
    cv2.circle(img, (int(robot_x), int(robot_y)), robot_size, color = 180, thickness = -1)  # Robot body
    cv2.circle(img,(sensor1_x, sensor1_y),4, 30,-1)  # Left sensor
    cv2.circle(img,(sensor2_x, sensor2_y),4, 30,-1)  # Right sensor
    cv2.imshow("LineTrace",img)

    # Check for ESC key to exit
    if cv2.waitKey(10) == 27:
        cv2.destroyAllWindows()
        break

    # Check if robot has gone off the line (both sensors read black)
    if sensor1_value == 0 and sensor2_value == 0:
        print("Time:",round(t,1))
        cv2.destroyAllWindows()
        break

    # Calculate steering and update robot position
    steer = robot_action(control_deviation)
    robot_x, robot_y, theta = robot_position(robot_x, robot_y, speed, theta, steer)
    sensor1_x, sensor1_y, sensor2_x, sensor2_y = sensor_position(robot_x, robot_y, robot_size, theta)
    t += sampling

# Save and plot results #######################
print('Simulation Ended')
savedata = savedata[1::,:]  # Remove initial placeholder row

# Save data to CSV file with parameters in filename #######################
savedata_name = "P_control_result(sumpling = " + str(sampling) + " velocity =" + str(speed) + " Kp =" + str(Kp) + ").csv"
np.savetxt(savedata_name, savedata, delimiter=',')

# Plot the control deviation over time #######################
time_data = savedata[:,0]
deviation_data = savedata[:,3]

fig = plt.figure()
plt.plot(time_data,deviation_data)
plt.ylim([-200,200])

# Save and display the plot #######################
savedata_name = "Kp = " + str(Kp) + ".png"
fig.savefig(savedata_name)
plt.show()