from Classes import *

# ####################### Main Execution #######################
def main():
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

main()