#!/usr/bin/env python

import rospy
import tf
import math
from math import atan2, sqrt, cos, sin, pi
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged

WAYPOINTS = [
    (0, 0),
    (1.5, 0),
    (1.5, 1.5),
    (0, 1.5),
    (0, 0)
]


class PID(object):
    """PID class returns the pid error
    """

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prevTime = None
        self.prevError = 0
        self.cumError = 0

    def calculate_pid(self, error):
        cur_time = rospy.get_time()
        if self.prevTime == None:
            self.prevTime = cur_time
            self.prevError = error
            return self.kp * error
        dt = cur_time - self.prevTime
        self.prevTime = cur_time
        self.cumError += error * dt
        pid = self.kp * error + self.kd * \
            (error - self.prevError) / dt + \
            min(max(-0.2, self.ki * self.cumError), 0.2)
        self.prevError = error
        return pid


class Bebop_functions():
    def __init__(self):
        # Bebop_control node creation
        rospy.init_node('Bebop_control', anonymous=True)

        # Publisher which will publish to the topic '/bebop/takeoff'.
        self.takeoff_publisher = rospy.Publisher('/bebop/takeoff',
                                                 Empty, queue_size=10)

        # Publisher which will publish to the topic '/bebop/land'.
        self.landing_publisher = rospy.Publisher('/bebop/land',
                                                 Empty, queue_size=10)

        # Publisher which will publish to the topic '/bebop/cmd_vel'.
        self.cmdvel_publisher = rospy.Publisher('/bebop/cmd_vel',
                                                Twist, queue_size=10)

        # A subscriber to the topic '/bebop/states/ardrone3/PilotingState/FlyingStateChanged'. self.update_state is called
        # when a message of type Ardrone3PilotingStateFlyingStateChanged is received.
        self.pilotingstate_subscriber = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
                                                         Ardrone3PilotingStateFlyingStateChanged, self.update_state)
        # Subscriber to the vicon transform node to read the position and quaternions of the bebop2
        self.bebopcoord_subscriber = rospy.Subscriber(
            '/vicon/parrot2020/parrot2020', TransformStamped, self.bebop_pose)

        self.state = None
        self.yaw = 0
        self.bebopose = Vector3()

        self.controlX = PID(0.08, 0.001, 0.05)
        self.controlY = PID(0.08, 0.001, 0.05)
        self.controlZ = PID(0.08, 0.001, 0.05)

        self.vel_lim = 2.0
        self.finished = False
        self.waypoint = WAYPOINTS.pop(0)
        # TransformStamped message update rate limited to 5 Hz, which limits to 5 Hz the subscribing rate.
        self.rate = rospy.Rate(5)

    def takeoff(self):  # Take off command function
        while self.state != 2:
            hello_str = "Takeoff command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.takeoff_publisher.publish(Empty())
            self.rate.sleep()
        print("Takeoff is done!")

    def land(self):  # Land command function
        while self.state != 0:
            hello_st = "Land command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_st)
            self.landing_publisher.publish(Empty())
            self.rate.sleep()

        print("Landing completed succesfully!")

    def move(self):  # Move function with a P controller.

        twist = Twist()
        # destination fixed z
        z = 1.0

        while not rospy.is_shutdown() and not self.finished:

            delta_x = self.waypoint[0] - self.bebopose.x  # position error
            delta_y = self.waypoint[1] - self.bebopose.y
            delta_z = z - self.bebopose.z

            rho = sqrt(delta_x*delta_x + delta_y*delta_y +
                       delta_z*delta_z)  # error to the goal
            error_x = math.cos(self.yaw) * delta_x + \
                math.sin(self.yaw) * delta_y
            error_y = -math.sin(self.yaw) * delta_x + \
                math.cos(self.yaw) * delta_y

            rospy.loginfo("rho: {:.2f}, yaw: {:.2f}".format(rho, self.yaw))

            # robot's body frame convertion for x and y linear velocities
            twist.linear.x = max(min(
                self.controlX.calculate_pid(error_x), self.vel_lim), -self.vel_lim)
            twist.linear.x = max(min(
                self.controlY.calculate_pid(error_y), self.vel_lim), -self.vel_lim)
            twist.linear.x = max(min(
                self.controlZ.calculate_pid(delta_z), self.vel_lim), -self.vel_lim)

            # publishing of angular rates and linear velocities
            self.cmdvel_publisher.publish(twist)
            if rho < 0.2:
                if len(WAYPOINTS) != 0:
                    self.waypoint = WAYPOINTS.pop(0)
                    rospy.sleep(3)
                else:
                    self.finished = True

            self.rate.sleep()
        if self.finished:
            self.land()
        print("Move is done!")

    # /bebop/states... callback subscriber to read the state of the parrot
    def update_state(self, message):

        self.state = message.state

    def safety_check(self):
        if self.bebopose.x > 3.0 or self.bebopose.x > 3.0 or self.bebopose.z > 2.0:
            self.finished = True

    # /vicon/yolanda/yolanda callback subscriber to know the position and the yaw angle of the robot
    def bebop_pose(self, msg):

        self.bebopose = msg.transform.translation
        quat = msg.transform.rotation
        (roll, pitch, self.yaw) = tf.transformations.euler_from_quaternion(
            (quat.x, quat.y, quat.z, quat.w))
        print "current position = {}, {}, {}".format(
            self.bebopose.x, self.bebopose.y, self.bebopose.z)
        self.safety_check()


if __name__ == '__main__':
    try:
        node = Bebop_functions()
        node.takeoff()
        node.move()
        node.land()
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))
