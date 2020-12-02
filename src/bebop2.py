#!/usr/bin/env python

import rospy
import tf
from math import atan2, sqrt, cos, sin, pi
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged


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
        self.bebopcoord_subscriber = rospy.Subscriber('/vicon/yolanda/yolanda', TransformStamped, self.bebop_pose)

        self.state=None
        self.yaw=0
        self.bebopose=Vector3()
        self.rate = rospy.Rate(5)  # TransformStamped message update rate limited to 5 Hz, which limits to 5 Hz the subscribing rate.

    def takeoff(self): # Take off command function
        while self.state != 2:
            hello_str = "Takeoff command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.takeoff_publisher.publish(Empty())
            self.rate.sleep()

        print("Takeoff is done!")
    
    def land (self): # Land command function
        while self.state != 0:
            hello_st = "Land command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_st)
            self.landing_publisher.publish(Empty())
            self.rate.sleep()
        
        print("Landing completed succesfully!")
    
    def move(self): # Move function with a P controller.

        krho = 0.02 # P controller parameters setup
        kalpha = 0.2
        kbeta = 0.01
        twist = Twist()

        x = 0
        y = 0
        z = 1.0

        while not rospy.is_shutdown():

            delta_x = x - self.bebopose.x # position error
            delta_y = y - self.bebopose.y
            delta_z = z - self.bebopose.z

            speed_x = delta_x*krho # calculated speeds for the P control
            speed_y = delta_y*krho
            speed_z = delta_z*krho
            

            rho = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z) # error to the goal
            rospy.loginfo("rho: {:.2f}, yaw: {:.2f}".format(rho, self.yaw))

            twist.linear.x = speed_x*cos(self.yaw) + speed_y*sin(self.yaw) # robot's body frame convertion for x and y linear velocities
            twist.linear.y = - speed_x*sin(self.yaw) + speed_y*cos(self.yaw)
            twist.linear.z = speed_z
            
            self.cmdvel_publisher.publish(twist) # publishing of angular rates and linear velocities
            self.rate.sleep()

        print("Move is done!")

    def update_state(self, message): # /bebop/states... callback subscriber to read the state of the parrot

        self.state = message.state
    
    def bebop_pose(self, msg): # /vicon/yolanda/yolanda callback subscriber to know the position and the yaw angle of the robot 

        self.bebopose = msg.transform.translation

        quat = msg.transform.rotation
        (roll, pitch, self.yaw) = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))


if __name__ == '__main__':
    try:
        node = Bebop_functions()
        #node.takeoff()
        node.move()
        #node.land()
    except rospy.ROSInterruptException:
        pass