# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math
from re import X  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # print(heading)
                # print('\n')
                # print(robot_pos)


                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

                x_desired_pos = -0.2
                y_desired_pos = -0.3
                h_desired_pos = 3.141592
                u=[0,0,0]
                error=[0,0,0]
                error[1] = float(robot_pos[0] - y_desired_pos)
                error[0] = float(robot_pos[1] - x_desired_pos)
                error[2] = float(h_desired_pos - heading)
                kp_x= 12
                kp_y = 12
                u[0] = float(kp_x * error[0])
                u[1] = float(kp_y * error[1])
                u[2] = float(2 * error[2])
                Vr=(2*u[0] + u[2]*0.08)/(2*0.02)
                Vl=(2*u[0] - u[2]*0.08)/(2*0.02)
                #print(error[0])
            
                if abs(error[0]) >= 0.1:
                    self.left_motor.setVelocity(u[0])
                    self.right_motor.setVelocity(u[0])
                else:
                    #print(heading)
                    if abs(heading- h_desired_pos) > (0.1):
                        print(abs(heading - h_desired_pos))
                        self.left_motor.setVelocity(-u[2])
                        self.right_motor.setVelocity(u[2])
                    else :
                        #self.left_motor.setVelocity(0)
                        #self.right_motor.setVelocity(0)
                        if abs(error[1]) >= 0.05:
                            print('ppp')
                            self.left_motor.setVelocity(u[1])
                            self.right_motor.setVelocity(u[1])
