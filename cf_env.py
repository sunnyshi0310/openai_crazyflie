import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float64

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from openai_ros.openai_ros_common import ROSLauncher

from mav_msgs.msg import DroneState
from mav_msgs.msg import Actuators
from tf.transformations import quaternion_from_euler


class CrazyflieEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes the Crazyflie environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /crazyflie2_#/ground_truth/imu: IMU of the drone giving linear acceleration and angular velocity.
        * /crazyflie2_#/motor_speed: Could be used to calculate the energy.
        * /crazyflie2_#/odometry: Get position and orientation in Global space
    

        Actuators Topic List:
        * /crazyflie2_#/drone_state: command to the lower level controller.
  

        Args:
        """
        rospy.logdebug("Start CrazyflieEnv INIT...")
        time.sleep(1.0)
        # Variables that we give through the constructor.
        # None in this case

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(CrazyflieEnv, self).__init__(controllers_list=self.controllers_list,
                                             robot_name_space=self.robot_name_space,
                                             reset_controls=False,
                                             start_init_physics_parameters=False,
                                             reset_world_or_sim="WORLD")

        self.gazebo.unpauseSim()

        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        # Crazyflie 1
        #rospy.Subscriber("/crazyflie2_1/gazebo/command/motor_speed", Actuators, self.motor_callback_1)
        rospy.Subscriber("/crazyflie2_1/odometry", Odometry, self.odom_callback_1)

        self.cmd_pub_1 = rospy.Publisher('/crazyflie2_1/drone_state', DroneState, queue_size=1)


        self._check_all_publishers_ready()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished CrazyflieEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        #self._check_motor_ready()
        self._check_gt_pose_ready()
        rospy.logdebug("ALL SENSORS READY")


    def _check_motor_ready(self):
        self.motor = None
        rospy.logdebug("Waiting for motor to be READY...")
        while self.motor is None and not rospy.is_shutdown():
            try:
                self.motor = rospy.wait_for_message(
                    "/crazyflie2_1/gazebo/command/motor_speed", Range, timeout=5.0)
                rospy.logdebug("Current motor READY=>")

            except:
                rospy.logerr(
                    "Current motor not ready yet, retrying for getting sonar")

        return self.sonar

    def _check_gt_pose_ready(self):
        self.gt_pose = None
        rospy.logdebug("Waiting for odometry to be READY...")
        while self.gt_pose is None and not rospy.is_shutdown():
            try:
                self.gt_pose = rospy.wait_for_message(
                    "/crazyflie2_1/odometry", Odometry, timeout=5.0)
                rospy.logdebug("Current odometry READY=>")

            except:
                rospy.logerr(
                    "Current odometry not ready yet, retrying for getting gt_pose")

        return self.gt_pose

    def motor_callback_1(self, data):
        self.motor_1 = data

    def odom_callback_1(self, data):
        self.gt_pose_1 = data


    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rospy.logdebug("START ALL SENSORS READY")
        self._check_cmd_vel_pub_connection()
        rospy.logdebug("ALL SENSORS READY")

    def _check_cmd_vel_pub_connection(self):

        rate = rospy.Rate(10)  # 10hz
        while self.cmd_pub_1.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    # Check the takeoff here
    def _check_takeoff_pub_connection(self):
        while self.gt_pose_1.pose.pose.position.z <= 0.99 and not rospy.is_shutdown():
            rospy.logdebug(
                "The drone doesn't take off successfully yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------


    def land(self):
        """
        Sends the Landing command and checks it has landed
        It unpauses the simulation and pauses again
        to allow it to be a self contained action
        """
        self.gazebo.unpauseSim()

        #self._check_land_pub_connection()

        #land_cmd = Empty()
        #self._land_pub.publish(land_cmd)
        # When Drone is on the floor, the readings are 0.5
        self.wait_for_height(heigh_value_to_check=0.2,
                             smaller_than=True,
                             epsilon=0.05,
                             update_rate=10)

        self.gazebo.pauseSim()

    def wait_for_height(self, heigh_value_to_check, smaller_than, epsilon, update_rate):
        """
        Checks if current height is smaller or bigger than a value
        :param: smaller_than: If True, we will wait until value is smaller than the one given
        """

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0

        rospy.logdebug("epsilon>>" + str(epsilon))

        while not rospy.is_shutdown():
            current_gt_pose = self._check_gt_pose_ready()

            current_height = current_gt_pose.pose.pose.position.z

            if smaller_than:
                takeoff_height_achieved = current_height <= heigh_value_to_check
                rospy.logwarn("SMALLER THAN HEIGHT...current_height=" +
                              str(current_height)+"<="+str(heigh_value_to_check))
            else:
                takeoff_height_achieved = current_height >= heigh_value_to_check
                rospy.logwarn("BIGGER THAN HEIGHT...current_height=" +
                              str(current_height)+">="+str(heigh_value_to_check))

            if takeoff_height_achieved:
                rospy.logwarn("Reached Height!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logwarn("Height Not there yet, keep waiting...")
            rate.sleep()

    def move_base(self, delta_x,delta_y,delta_z, epsilon=0.05, update_rate=10):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed_vector: Speed in the XYZ axis of the robot base frame, because drones can move in any direction
        :param angular_speed: Speed of the angular turning of the robot base frame, because this drone only turns on the Z axis.
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        q = quaternion_from_euler(0, 0, 0)
        drone_msg = DroneState()
        drone_msg.position.x = self.gt_pose_1.pose.pose.position.x+delta_x*update_rate/1000
        drone_msg.position.y = self.gt_pose_1.pose.pose.position.y+delta_y*update_rate/1000
        drone_msg.position.z = self.gt_pose_1.pose.pose.position.z+delta_z*update_rate/1000
        drone_msg.linear_velocity.x = delta_x
        drone_msg.linear_velocity.y = delta_y
        drone_msg.linear_velocity.z = delta_z
        drone_msg.linear_acceleration.x = 0
        drone_msg.linear_acceleration.y = 0
        drone_msg.linear_acceleration.z = 0
        drone_msg.orientation.x = q[0]
        drone_msg.orientation.y = q[1]
        drone_msg.orientation.z = q[2]
        drone_msg.orientation.w = q[3]
        drone_msg.angular_velocity.x = 0 
        drone_msg.angular_velocity.y = 0
        drone_msg.angular_velocity.z = 0
        drone_msg.angular_acceleration.x = 0
        drone_msg.angular_acceleration.y = 0
        drone_msg.angular_acceleration.z = 0


        self._check_cmd_vel_pub_connection()
        self.cmd_pub_1.publish(drone_msg)

        self.wait_time_for_execute_movement()

    def wait_time_for_execute_movement(self):
        """
        Because this Parrot Drone position is global, we really dont have
        a way to know if its moving in the direction desired, because it would need
        to evaluate the diference in position and speed on the local reference.
        """
        time.sleep(1.0)


    def get_motor(self):
        return self.motor_1

    def get_gt_pose(self):
        return self.gt_pose_1
