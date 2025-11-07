import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
from robot_setting_information import sensor_position, robot_position

# ####################### Configuration Constants #######################
# Centralized for easy tuning
class Config:
    # Robot initial state
    ROBOT_X_INIT = 80
    ROBOT_Y_INIT = 70
    ROBOT_SIZE = 20
    THETA_INIT = 90  # Degrees, pointing upward
    
    # Motion parameters
    SPEED = 10
    SAMPLING = 0.05
    
    # P control parameters
    KP = 0.3
    
    # Simulation
    COURSE_IMAGE_PATH = 'course.png'
    
    # Data saving
    DATA_FILENAME_TEMPLATE = "P_control_result(sampling={sampling} velocity={speed} Kp={kp}).csv"
    PLOT_FILENAME_TEMPLATE = "Kp={kp}.png"

# ####################### Robot Class #######################
# Manages robot position, sensors, and movement
class Robot:
    def __init__(self, x, y, size, theta):
        self.x = x
        self.y = y
        self.size = size
        self.theta = theta
        self.update_sensors()
    
    def update_sensors(self):
        # Calculate sensor positions
        self.sensor1_x, self.sensor1_y, self.sensor2_x, self.sensor2_y = sensor_position(
            self.x, self.y, self.size, self.theta
        )
    
    def get_sensor_readings(self, image):
        # Read sensor values from the image
        sensor1_value = int(image[self.sensor1_y, self.sensor1_x])
        sensor2_value = int(image[self.sensor2_y, self.sensor2_x])
        return sensor1_value, sensor2_value
    
    def move(self, speed, steer):
        # Update robot position and sensors
        self.x, self.y, self.theta = robot_position(self.x, self.y, speed, self.theta, steer)
        self.update_sensors()

# ####################### ProportionalController Class #######################
# Handles P control for steering
class ProportionalController:
    def __init__(self, kp):
        self.kp = kp
    
    def compute_steer(self, control_deviation):
        # Positive deviation turns right, negative turns left
        return self.kp * control_deviation

# ####################### Simulator Class #######################
# Manages the simulation loop, drawing, and user interaction
class Simulator:
    def __init__(self, robot, controller, image_path):
        self.robot = robot
        self.controller = controller
        self.image = cv2.imread(image_path, flags=cv2.IMREAD_GRAYSCALE)  # Pre-load for efficiency
        self.t = 0
        self.sampling = Config.SAMPLING
        self.speed = Config.SPEED
    
    def run(self, data_logger):
        while True:
            # Get sensor readings
            sensor1_value, sensor2_value = self.robot.get_sensor_readings(self.image)
            control_deviation = int(sensor2_value - sensor1_value)
            
            # Log data
            data_logger.log(self.t, sensor1_value, sensor2_value, control_deviation)
            
            # Draw robot and sensors
            img = self.image.copy()  # Work on a copy
            cv2.circle(img, (int(self.robot.x), int(self.robot.y)), self.robot.size, color=180, thickness=-1)
            cv2.circle(img, (self.robot.sensor1_x, self.robot.sensor1_y), 4, 30, -1)
            cv2.circle(img, (self.robot.sensor2_x, self.robot.sensor2_y), 4, 30, -1)
            cv2.imshow("LineTrace", img)
            
            # Check for exit
            if cv2.waitKey(10) == 27:
                cv2.destroyAllWindows()
                break
            
            # Check for line loss (exit immediately)
            if sensor1_value == 0 and sensor2_value == 0:
                print("Time:", round(self.t, 1))
                cv2.destroyAllWindows()
                break
            
            # Calculate steering and update robot
            steer = self.controller.compute_steer(control_deviation)
            self.robot.move(self.speed, steer)
            self.t += self.sampling
            sleep(self.sampling)

# ####################### DataLogger Class #######################
# Handles data collection, saving, and plotting
class DataLogger:
    def __init__(self):
        self.data = np.array([[0, 0, 0, 0]])  # [time, sensor1, sensor2, deviation]
    
    def log(self, t, sensor1, sensor2, deviation):
        self.data = np.vstack([self.data, [t, sensor1, sensor2, deviation]])
    
    def save_and_plot(self, sampling, speed, kp):
        # Remove initial placeholder
        self.data = self.data[1:]
        
        # Save to CSV
        filename = Config.DATA_FILENAME_TEMPLATE.format(sampling=sampling, speed=speed, kp=kp)
        np.savetxt(filename, self.data, delimiter=',')
        
        # Plot deviation
        time_data = self.data[:, 0]
        deviation_data = self.data[:, 3]
        fig = plt.figure()
        plt.plot(time_data, deviation_data)
        plt.ylim([-200, 200])
        
        # Save and show plot
        plot_filename = Config.PLOT_FILENAME_TEMPLATE.format(kp=kp)
        fig.savefig(plot_filename)
        plt.show()

# ####################### Main Execution #######################
if __name__ == "__main__":
    # Initialize components
    robot = Robot(Config.ROBOT_X_INIT, Config.ROBOT_Y_INIT, Config.ROBOT_SIZE, Config.THETA_INIT)
    controller = ProportionalController(Config.KP)
    data_logger = DataLogger()
    simulator = Simulator(robot, controller, Config.COURSE_IMAGE_PATH)
    
    # Run simulation
    simulator.run(data_logger)
    
    # Post-simulation: Save and plot
    print('Simulation Ended')
    data_logger.save_and_plot(Config.SAMPLING, Config.SPEED, Config.KP)
