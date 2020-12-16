#!/usr/bin/env python

import rospy
import tf
import math
import json
import matplotlib.pyplot as plt
from uuid import uuid4
from math import atan2, sqrt, cos, sin, pi
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged


UNIQUE_ID = uuid4()

WAYPOINTS = [
    (0, 0),
    (1.5, 0),
    (1.5, 1.5),
    (0, 1.5),
    (0, 0)
]

x_plot = {
    'error': [],
    'time': []
}
y_plot = {
    'error': [],
    'time': []
}
z_plot = {
    'error': [],
    'time': []
}


# function to add to JSON 
def write_json(new_data, filename='data.json'): 
    with open(filename) as json_file: 
        data = json.load(json_file) 
        
        temp = data['PID'] 
   
        # appending data to emp_details  
        temp.append(new_data)

    with open(filename,'w') as f: 
        json.dump(data, f, indent=4)


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
            self.ki * self.cumError
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
            'vicon/Bepop2020/Bepop2020', TransformStamped, self.bebop_pose)

        self.state = None
        self.yaw = 0
        self.bebopose = Vector3()

        self.controlX = PID(0.3, 0.006, 0.4)
        self.controlY = PID(0.3, 0.006, 0.42)
        self.controlZ = PID(0.12, 0.015, 0.25)

        self.vel_lim = 2.0
        self.finished = False
        self.waypoint = WAYPOINTS.pop(0)
        # TransformStamped message update rate limited to 5 Hz, which limits to 5 Hz the subscribing rate.
        self.rate = rospy.Rate(10)

    def takeoff(self):  # Take off command function
        while self.state != 2 and not rospy.is_shutdown():
            hello_str = "Takeoff command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.takeoff_publisher.publish(Empty())
            self.rate.sleep()
        print("Takeoff is done!")

    def land(self):  # Land command function
        self.finished = True
        self.cmdvel_publisher.publish(Twist())
        while self.state != 0 and not rospy.is_shutdown():
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


            # robot's body frame convertion for x and y linear velocities
            twist.linear.x = max(min(
                self.controlX.calculate_pid(error_x), self.vel_lim), -self.vel_lim)
            twist.linear.y = max(min(
                self.controlY.calculate_pid(error_y), self.vel_lim), -self.vel_lim)
            twist.linear.z = max(min(
                self.controlZ.calculate_pid(delta_z), self.vel_lim), -self.vel_lim)

            # rospy.loginfo("rho: {:.2f}, yaw: {:.2f},pos = {}, {}, {}, twist: {}".format(rho, self.yaw,
            #                                                                                self.bebopose.x, self.bebopose.y, self.bebopose.z, twist.linear))
            x_plot['error'].append(error_x)
            x_plot['time'].append(rospy.get_time())
            y_plot['error'].append(error_y)
            y_plot['time'].append(rospy.get_time())
            z_plot['error'].append(delta_z)
            z_plot['time'].append(rospy.get_time())
            # rospy.loginfo("rho: {:.2f},pos = {:.2f}, {:.2f}, {:.2f}".format(rho, self.bebopose.x, self.bebopose.y, self.bebopose.z))
            rospy.loginfo("rho: {:.2f},err = {:.2f}, {:.2f}, {:.2f}".format(rho, error_x, error_y, delta_z))
            # publishing of angular rates and linear velocities
            self.cmdvel_publisher.publish(twist)
            if rho < 0.1:
                if len(WAYPOINTS) != 0:
                    self.waypoint = WAYPOINTS.pop(0)
                    print "new waypoint {}, {}".format(
                        self.waypoint[0], self.waypoint[1])
                    rospy.sleep(3)
                else:
                    self.finished = True

            self.rate.sleep()
        self.cmdvel_publisher.publish(twist)
        if self.finished:
            self.cmdvel_publisher.publish(twist)
            self.land()
        print("Move is done!")

    # /bebop/states... callback subscriber to read the state of the parrot
    def update_state(self, message):

        self.state = message.state

    def safety_check(self):
        if self.bebopose.x > 4.0 or self.bebopose.x > 4.0 or self.bebopose.z > 2.0:
            print "EMERGENCY LANDING"
            self.finished = True

    # /vicon/yolanda/yolanda callback subscriber to know the position and the yaw angle of the robot
    def bebop_pose(self, msg):

        self.bebopose = msg.transform.translation
        quat = msg.transform.rotation
        (roll, pitch, self.yaw) = tf.transformations.euler_from_quaternion(
            (quat.x, quat.y, quat.z, quat.w))
        self.safety_check()


if __name__ == '__main__':
    try:
        import subprocess, shlex
        command = "rosbag record -x bebop/image_raw -O {}".format(str(UNIQUE_ID))
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)
        node = Bebop_functions()
        node.takeoff()
        rospy.sleep(2)
        node.move()
        node.land()

        new_data = {
            "fig": str(UNIQUE_ID),
            "x": {
                "p": node.controlX.kp,
                "i": node.controlX.ki,
                "d": node.controlX.kd,
            },
            "y": {
                "p": node.controlY.kp,
                "i": node.controlY.ki,
                "d": node.controlY.kd,
            },
            "z": {
                "p": node.controlZ.kp,
                "i": node.controlZ.ki,
                "d": node.controlZ.kd,
            },
        }
        write_json(new_data)

        fig, (idx1, idx2, idx3) = plt.subplots(3,1)
        idx1.plot(x_plot['time'], x_plot['error'])
        idx1.set_xlabel('x-axis')
        idx2.set_ylabel('error')
        idx2.plot(y_plot['time'], y_plot['error'])
        idx1.set_xlabel('x-axis')
        idx2.set_ylabel('error')
        idx3.plot(z_plot['time'], z_plot['error'])
        idx1.set_xlabel('x-axis')
        idx2.set_ylabel('error')
        fig.savefig('plots/{}'.format(str(UNIQUE_ID)))

        rosbag_proc.send_signal(subprocess.signal.SIGINT)
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))
