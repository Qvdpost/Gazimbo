# --------------------------------------------
# -  Master:       Software Engineering
# -  Course:       Embedded Systems and Software
# -  Assignment:   Maze solver application for Turtlebot3
# -
# -  File name:     main.py
# -  Application:   Yakindu 4.0 Statechart application
# -  Target:        Turtlebot3 and ROS2
# -
# -  Author: Mirka Sxhouuten (orginal version)
# -  Date: July/August 2020
# -
# -  Modified to Yakindu 4.0: E.H. Steffems
# -  Date: 01 November 2020
# -
# --------------------------------------------

import sys, os, select
import numpy as np

from model import Model

from TurtleBotNode import TurtleBot
from grid.Grid import Maze

from timer.sct_timer import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist


class SCTConnect():

    """
    initialise class and main variables
    """
    def __init__(self):
        # start the ROS 2 node
        rclpy.init()
        self.node = TurtleBot()

        # Initialize the statecharts
        self.sm = Model()

    """
    Setup the statemachine and the ROS 2 node
    """
    def setup(self):
        # Setup statechart and initialize maze datastructure
        self.maze = Maze(self.sm.grid.max_col+1, self.sm.grid.max_row+1)

        self.sm.timer_service = Timer()
        self.sm.enter()

        # Setup variables to be used that will not be changed by the statecharts
        self.degrees_list = [i for i in range(0,180)]
        self.degrees_list.extend([i for i in range(-180,0)])
        self.highest_distance = 4
        self.input = ''


    """
    This function runs the cycles of the program with every while loop
    """
    def run(self):
        while not self.sm.output.finish:

            # Spin the ROS 2 node to get 1 callback
            rclpy.spin_once(self.node)

            # Parse the data of the given callback
            if self.node.last_callback == 1:
                self.get_data_laser()
            elif self.node.last_callback == 2:
                self.get_data_odom()
            elif self.node.last_callback == 3:
                self.get_data_imu()
            #print("Callback type:", self.node.last_callback)
            self.node.reset_last()

            # Get keyboard input from user
            self.input = self.timeout_input()
            if self.input != '':
                sys.stdout.write('\n')
                sys.stdout.flush()

            # Run a cycle of the statechart
            if(not self.parse_input()):
                self.sm.run_cycle()

            # Reset input
            self.input = ''

            # Run events activated in the statechart
            if self.sm.start_pos.set_zero:
                self.set_zero()
            if self.sm.grid.receive:
                self.receive_grid()
            if self.sm.grid.update:
                self.update_grid()
                self.maze.draw()

            # Publish the current speed and rotation from SCT
            # print("Velocity: ", self.sm.output.speed, self.sm.output.rotation)
            self.node.vel_publish(x=self.sm.output.speed, rz = self.sm.output.rotation)

            # Print info
            os.system('clear')
            print("========= TurtleBot Stats =========")
            print(f"velocity: {self.sm.output.speed}")
            print(f"rotation speed: {self.sm.output.rotation}")
            print(f"yaw: {self.sm.imu.yaw:.3f}")

            print("----------- Grid Info -----------")
            print(f"orientation: {self.sm.grid.orientation}")
            print(f"row: {self.sm.grid.row}")
            print(f"col: {self.sm.grid.column}")

            print(f"start pos:")
            print(f"\tzero x: {self.sm.start_pos.zero_x:.2f}")
            print(f"\tzero y: {self.sm.start_pos.zero_y:.2f}")
            print(f"\tset zero: {self.sm.start_pos.set_zero}")

            print("----------- Laser stuff -----------")
            print(f"front mean:\t{self.sm.laser_distance.dfront_mean:.2f}")
            print(f"left mean:\t{self.sm.laser_distance.dleft_mean:.2f}")
            print(f"back mean:\t{self.sm.laser_distance.dback_mean:.2f}")
            print(f"right mean:\t{self.sm.laser_distance.dright_mean:.2f}")

            print("------------ Odometry -------------")
            print(f"x: {self.sm.odom.x:.2f}")
            print(f"y: {self.sm.odom.y:.2f}")

            print("------------ Logging -------------")
            print(f"visited: {self.sm.grid.visited:.2f}")
            print(f'Wall front: {self.sm.grid.wall_front}, right: {self.sm.grid.wall_right}, '
                    f'back: {self.sm.grid.wall_back}, left: {self.sm.grid.wall_left}')

        # Print the final values and finish
        print("gems: ", self.sm.output.gems)
        print("obstacles: ", self.sm.output.obstacles)
        self.shutdown()


    """
    Destroy the ROS2 node and statemachine upon completion
    """
    def shutdown(self):
        rclpy.shutdown()
        self.sm.exit()


    """
    Parse the laser data and remove error values,
    namely the 0 values of the physical robot and the inf values of Gazebo for distance
    """
    def get_data_laser(self):
        # Get the range for the angles at the right and the front
        range_front = int(self.sm.base_values.degrees_front /2)
        range_right = int(self.sm.base_values.degrees_right /2)
        range_back  = int(self.sm.base_values.degrees_back  /2)
        range_left  = int(self.sm.base_values.degrees_left  /2)

        # set the degrees for the offset
        degrees_directions = [0,0,0,0]
        if self.sm.start_pos.laser_deg_offset < 0:
            degrees_directions = [360 + self.sm.start_pos.laser_deg_offset,
                                   90 + self.sm.start_pos.laser_deg_offset,
                                  180 + self.sm.start_pos.laser_deg_offset,
                                  270 + self.sm.start_pos.laser_deg_offset]
        else:
            degrees_directions = [0 + self.sm.start_pos.laser_deg_offset,
                                 90 + self.sm.start_pos.laser_deg_offset,
                                180 + self.sm.start_pos.laser_deg_offset,
                                270 + self.sm.start_pos.laser_deg_offset]

        """
        Parse distance data
        """
        # Get the laser distance in 4 base directions, filter out error values
        if(self.node.s_ranges[degrees_directions[0]] != 0 and
           self.node.s_ranges[degrees_directions[0]] < self.highest_distance):
            self.sm.laser_distance.d0 = self.node.s_ranges[degrees_directions[0]]
        else:
            self.sm.laser_distance.d0 = self.highest_distance

        if(self.node.s_ranges[degrees_directions[1]] != 0 and
           self.node.s_ranges[degrees_directions[1]] < self.highest_distance):
            self.sm.laser_distance.d90 = self.node.s_ranges[degrees_directions[1]]
        else:
            self.sm.laser_distance.d90 = self.highest_distance

        if(self.node.s_ranges[degrees_directions[2]] != 0 and
           self.node.s_ranges[degrees_directions[2]] < self.highest_distance):
            self.sm.laser_distance.d180 = self.node.s_ranges[degrees_directions[2]]
        else:
            self.sm.laser_distance.d180 = self.highest_distance

        if(self.node.s_ranges[degrees_directions[3]] != 0 and
           self.node.s_ranges[degrees_directions[3]] < self.highest_distance):
            self.sm.laser_distance.dm90 = self.node.s_ranges[degrees_directions[3]]
        else:
            self.sm.laser_distance.dm90 = self.highest_distance


        # Make a list of tuples to get the index for the minimum and the maximum
        list_of_tuples_min = [
                (self.node.s_ranges[i], self.degrees_list[i]) if self.node.s_ranges[i] != 0
                 else (self.highest_distance, self.degrees_list[i])
                 for i in range(len(self.node.s_ranges))]
        list_of_tuples_max = [
                (self.node.s_ranges[i], self.degrees_list[i]) if self.node.s_ranges[i] < self.highest_distance
                 else (0, self.degrees_list[i])
                 for i in range(len(self.node.s_ranges))]

        list_for_mean = [x for x in self.node.s_ranges if x != 0 and x < self.highest_distance]

        # Get the min, mean and max of all lasers and in what direction the min and max are
        self.sm.laser_distance.dmin, self.sm.laser_distance.min_deg = min(list_of_tuples_min)

        # Correct min_deg for laser_deg_offset
        if self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset < -180:
            self.sm.laser_distance.min_deg = self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset + 360
        elif self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset > 180:
            self.sm.laser_distance.min_deg = self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset - 360
        else:
            self.sm.laser_distance.min_deg -= self.sm.start_pos.laser_deg_offset

        self.sm.laser_distance.dmax, self.sm.laser_distance.max_deg = max(list_of_tuples_max)

        # Correct max_deg for laser_deg_offset
        if self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset < -180:
            self.sm.laser_distance.max_deg = self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset + 360
        elif self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset > 180:
            self.sm.laser_distance.max_deg = self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset - 360
        else:
            self.sm.laser_distance.max_deg -= self.sm.start_pos.laser_deg_offset

        self.sm.laser_distance.dmean = np.mean(list_for_mean)

        # Get the distance min, mean and max of the lasers directed towards the front
        temp_list = list_of_tuples_min[300:360]
        temp_list.extend(list_of_tuples_min[0:60])
        temp_list = temp_list[60 - range_front + self.sm.start_pos.laser_deg_offset:60 + range_front + self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dfront_min, self.sm.laser_distance.min_deg_f = min(temp_list2)
            self.sm.laser_distance.dfront_max, self.sm.laser_distance.max_deg_f = max(temp_list2)
            self.sm.laser_distance.dfront_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dfront_min = self.highest_distance
            self.sm.laser_distance.min_deg_f = 0
            self.sm.laser_distance.dfront_max = self.highest_distance
            self.sm.laser_distance.max_deg_f = 0
            self.sm.laser_distance.dfront_mean = self.highest_distance

        # Get the distance min, mean and max of the lasers directed at the right
        temp_list = list_of_tuples_min[270-range_right+self.sm.start_pos.laser_deg_offset:270+range_right+self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dright_min, self.sm.laser_distance.min_deg_r = min(temp_list2)
            self.sm.laser_distance.dright_max, self.sm.laser_distance.max_deg_r = max(temp_list2)
            self.sm.laser_distance.dright_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dright_min = self.highest_distance
            self.sm.laser_distance.min_deg_r = -90
            self.sm.laser_distance.dright_max = self.highest_distance
            self.sm.laser_distance.max_degr = -90
            self.sm.laser_distance.dright_mean = self.highest_distance

        # Get the distance min, mean and max of the lasers directed at the back
        temp_list = list_of_tuples_min[180-range_back+self.sm.start_pos.laser_deg_offset:180+range_back+self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dback_min, self.sm.laser_distance.min_deg_b = min(temp_list2)

            if self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset < -180:
                self.sm.laser_distance.min_deg_b = self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset + 360
            elif self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset > 180:
                self.sm.laser_distance.min_deg_b = self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset - 360
            else:
                self.sm.laser_distance.min_deg_b -= self.sm.start_pos.laser_deg_offset

            self.sm.laser_distance.dback_max, self.sm.laser_distance.max_deg_b = max(temp_list2)
            if self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset < -180:
                self.sm.laser_distance.max_deg_b = self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset + 360
            elif self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset > 180:
                self.sm.laser_distance.max_deg_b = self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset - 360
            else:
                self.sm.laser_distance.max_deg_b -= self.sm.start_pos.laser_deg_offset

            self.sm.laser_distance.dback_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dback_min = self.highest_distance
            self.sm.laser_distance.min_deg_b = 180
            self.sm.laser_distance.dback_max = self.highest_distance
            self.sm.laser_distance.max_deg_b = 180
            self.sm.laser_distance.dback_mean = self.highest_distance

        # Get the distance min, mean and max of the lasers directed at the left
        temp_list = list_of_tuples_min[90-range_left+self.sm.start_pos.laser_deg_offset:90+range_left+self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dleft_min, self.sm.laser_distance.min_deg_l = min(temp_list2)
            self.sm.laser_distance.dleft_max, self.sm.laser_distance.max_deg_l = max(temp_list2)
            self.sm.laser_distance.dleft_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dleft_min = self.highest_distance
            self.sm.laser_distance.min_deg_l = 90
            self.sm.laser_distance.dleft_max = self.highest_distance
            self.sm.laser_distance.max_deg_l = 90
            self.sm.laser_distance.dleft_mean = self.highest_distance


        """
        Parse the intensity values
        """
        # Get the laser distance in 4 base directions
        # Note: negative indices (-1 eg.) are handled correctly in python.
        self.sm.laser_intensity.i0 = self.node.s_intensity[0+self.sm.start_pos.laser_deg_offset]
        self.sm.laser_intensity.i90 = self.node.s_intensity[90+self.sm.start_pos.laser_deg_offset]
        self.sm.laser_intensity.i180 = self.node.s_intensity[180+self.sm.start_pos.laser_deg_offset]
        self.sm.laser_intensity.im90 = self.node.s_intensity[270+self.sm.start_pos.laser_deg_offset]

        # Get the intensity min, mean and max of the lasers directed towards the front
        temp_list = self.node.s_intensity[300:360]
        temp_list.extend(self.node.s_intensity[0:60])
        temp_list = temp_list[60-range_front+self.sm.start_pos.laser_deg_offset:60+range_front+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.ifront_min = min(temp_list)
        self.sm.laser_intensity.ifront_max = max(temp_list)
        self.sm.laser_intensity.ifront_mean = np.mean(temp_list)

        # Get the intensity min, mean and max of the lasers directed at the right
        temp_list = self.node.s_intensity[270-range_right+self.sm.start_pos.laser_deg_offset:270+range_right+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.iright_min = min(temp_list)
        self.sm.laser_intensity.iright_max = max(temp_list)
        self.sm.laser_intensity.iright_mean = np.mean(temp_list)

        # Get the intensity min, mean and max of the lasers directed at the back
        temp_list = self.node.s_intensity[180-range_back+self.sm.start_pos.laser_deg_offset:180+range_back+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.iback_min = min(temp_list)
        self.sm.laser_intensity.iback_max = max(temp_list)
        self.sm.laser_intensity.iback_mean = np.mean(temp_list)

        # Get the intensity min, mean and max of the lasers directed at the left
        temp_list = self.node.s_intensity[90-range_left+self.sm.start_pos.laser_deg_offset:90+range_left+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.ileft_min = min(temp_list)
        self.sm.laser_intensity.ileft_max = max(temp_list)
        self.sm.laser_intensity.ileft_mean = np.mean(temp_list)


    """
    Parse odometry data
    """
    def get_data_odom(self):
        self.sm.odom.x = self.node.odom_x
        self.sm.odom.y = self.node.odom_y
        self.sm.odom.z = self.node.odom_z


    """
    Parse the imu data
    """
    def get_data_imu(self):
        # Get the data from the node
        self.quaternion = self.node.orientation

        # Convert the data to Euler angles
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(self.quaternion)

        # Send the data to SCT
        self.sm.imu.roll  = self.roll
        self.sm.imu.pitch = self.pitch
        self.sm.imu.yaw   = self.yaw


    """
    User input with a timeout such to not stop program flow
    """
    def timeout_input(self, timed=0.05):
        input_found, _, _ = select.select([sys.stdin], [], [], timed)
        if input_found:
            return sys.stdin.readline().rstrip("\n")
        else:
            return ''


    """
    Parse the input and raise the appropriate event
    """
    def parse_input(self):
        if self.input == 'm':
           self.sm.computer.raise_m_press()
           return True
        elif self.input == 'w':
            self.sm.computer.raise_w_press()
            return True
        elif self.input == 'a':
            self.sm.computer.raise_a_press()
            return True
        elif self.input == 's':
            self.sm.computer.raise_s_press()
            return True
        elif self.input == 'd':
            self.sm.computer.raise_d_press()
            return True
        elif self.input == 'x':
            self.sm.computer.raise_x_press()
            return True
        else:
            return False

    # https://github.com/ROBOTIS-GIT/turtlebot3/blob/33b59cdadab82167c412787441fd8b240c9c24c1/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)*180/np.pi

        sinp = 2 * (w*y - z*x)
        pitch = np.arcsin(sinp)*180/np.pi

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)*180/np.pi

        return roll, pitch, yaw

    """
    calibrate the robot
    """
    def set_zero(self):
        self.sm.start_pos.set_zero = False

        # Set location of the start
        self.sm.start_pos.zero_x = self.sm.odom.x
        self.sm.start_pos.zero_y = self.sm.odom.y

        # Set the degree of facing south at the start
        self.sm.start_pos.zero_south_degree = self.sm.imu.yaw

        # Calculate laser offset by looking what laser is orthogonal to the wall
        if(self.sm.laser_distance.min_deg < -135):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg+180
        elif(self.sm.laser_distance.min_deg < -45):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg+90
        elif(self.sm.laser_distance.min_deg < 45):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg
        elif(self.sm.laser_distance.min_deg < 135):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg-90
        else:
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg-180

    """
    Retrieve wall data of the current node from the Maze
    """
    def receive_grid(self):
        self.sm.grid.receive = False
        self.current_node = self.maze.grid[self.sm.grid.row][self.sm.grid.column]

        self.sm.grid.wall_front = self.current_node.walls[self.sm.grid.orientation]
        self.sm.grid.wall_right = self.current_node.walls[(self.sm.grid.orientation + 1) % 4]
        self.sm.grid.wall_left  = self.current_node.walls[(self.sm.grid.orientation - 1) % 4]
        self.sm.grid.wall_back  = self.current_node.walls[(self.sm.grid.orientation - 2) % 4]
        self.sm.grid.visited = self.current_node.visited

    """
    Update Maze data with statechart information
    """
    def update_grid(self):
        self.sm.grid.update = False
        self.current_node = self.maze.grid[self.sm.grid.row][self.sm.grid.column]
        self.current_node.walls[self.sm.grid.orientation] = self.sm.grid.wall_front
        self.current_node.walls[(self.sm.grid.orientation + 1) % 4] = self.sm.grid.wall_right
        self.current_node.walls[(self.sm.grid.orientation - 1) % 4] = self.sm.grid.wall_left
        self.current_node.walls[(self.sm.grid.orientation - 2) % 4] = self.sm.grid.wall_back
        self.current_node.visited = True
        self.sm.grid.visited = True

"""
Run the program
"""
if __name__ == "__main__":
    states = SCTConnect()
    states.setup()
    try:
        states.run()
    except KeyboardInterrupt:
        states.shutdown()
