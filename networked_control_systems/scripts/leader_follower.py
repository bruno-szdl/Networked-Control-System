#! /usr/bin/env python

# Importing custom msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Importing other libraries
import message_filters #This library allows a callback function to listen to multiple topics simultaneously
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
import random

#Creating the Subscriber and Publisher class
class SubAndPub:
    def __init__(self):

        self.name = rospy.get_param("~robot_name")

        #Minimum distance for communication
        self.min_dist = 10

        #Chance of successfully receiving a message
        self.chances = 0.9

        #Number of robots
        self.n_robots = 6

        #Defining the type of the message to be published
        self.twist_msg = Twist()

        # Defining the publisher variable
        pub_topic = "/" + self.name + "/cmd_vel"
        self.cmdvel_pub = rospy.Publisher(pub_topic, Twist, queue_size=10)

        #Defining the subscribers variables
        self.subscribers = self.get_odom_subscribers(self.n_robots)

        # Synchronizing the messages from the subscribers
        self.ts = message_filters.ApproximateTimeSynchronizer(self.subscribers, 20, 0.5)

        # Calling the Callback function
        self.ts.registerCallback(self.Callback)


    #Function to put the subscribers topics into a list
    #The first topic must be of the robot according to ~robot_name
    #The other ones follow an increasing numerical order
    def get_odom_subscribers(self, n_robots):
        subscribers = []
        sub_topic = "/" + self.name + "/odom"
        subscribers.append(message_filters.Subscriber(sub_topic, Odometry))
        for i in range(n_robots):
            sub_topic = "/tb3_" + str(i) + "/odom"
            if self.name not in sub_topic:
                subscribers.append(message_filters.Subscriber(sub_topic, Odometry))
        return subscribers

    #Function to get the distances between this robot and the leader
    #If this robot cannot receive information about the leader,
    #it follows the other robots
    #The desired point is also calculated
    def get_distances(self, this_robot_pos, other_robots_pos):
        distances = np.zeros((3,5))
        for i in range(self.n_robots-1):
            distances[0,i] = other_robots_pos[0,i] - this_robot_pos[0,0]
            distances[1,i] = other_robots_pos[1,i] - this_robot_pos[1,0]
            distances[2,i] = np.sqrt(distances[0,i]**2 + distances[1,i]**2)

        #If leader is close enough
        if (distances[2,0] < self.min_dist) and (random.random() < self.chances):
            desired_x = this_robot_pos[0,0] + distances[0,0] + random.uniform(-0.1, 0.1)
            desired_y = this_robot_pos[1,0] + distances[1,0] + random.uniform(-0.1, 0.1)

        #If leader is not close enough
        else:
            n = 0
            sum_x = 0
            sum_y = 0
            for i in range(1, self.n_robots-1):
                if (distances[2,i] < self.min_dist) and (random.random() < self.chances):
                    n += 1
                    sum_x += distances[0,i] + random.uniform(-0.1, 0.1)
                    sum_y += distances[1,i] + random.uniform(-0.1, 0.1)
            desired_x = this_robot_pos[0,0] + sum_x
            desired_y = this_robot_pos[1,0] + sum_y

        x_distance = desired_x - this_robot_pos[0,0]
        y_distance = desired_y - this_robot_pos[1,0]

        return x_distance, y_distance

    #Function to move the robot publishing to /cmd_vel topic
    def move(self, x_distance, y_distance, desired_angle, this_robot_orientation):
        if abs(x_distance) < 0.5 and abs(y_distance) < 0.5:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        elif desired_angle - this_robot_orientation > 0.1:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.5
        elif desired_angle - this_robot_orientation < -0.1:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = -0.5
        else:
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.z = 0.0

        self.cmdvel_pub.publish(self.twist_msg)

    def Callback(self, odom_sub_1_info, odom_sub_2_info, odom_sub_3_info, odom_sub_4_info, odom_sub_5_info, odom_sub_6_info):
        
        this_robot_pos = np.array([                                      
                                  [odom_sub_1_info.pose.pose.position.x],
                                  [odom_sub_1_info.pose.pose.position.y]
                                  ])

        this_robot_orientation = (R.from_quat([0, 0, odom_sub_1_info.pose.pose.orientation.z, odom_sub_1_info.pose.pose.orientation.w])).as_euler('zyx')[0]

        other_robots_pos = np.array([
                                    [odom_sub_2_info.pose.pose.position.x, odom_sub_3_info.pose.pose.position.x,
                                    odom_sub_4_info.pose.pose.position.x, odom_sub_5_info.pose.pose.position.x,
                                    odom_sub_6_info.pose.pose.position.x],
                                    [odom_sub_2_info.pose.pose.position.y, odom_sub_3_info.pose.pose.position.y,
                                    odom_sub_4_info.pose.pose.position.y, odom_sub_5_info.pose.pose.position.y,
                                    odom_sub_6_info.pose.pose.position.y]
                                    ])

        x_distance, y_distance = self.get_distances(this_robot_pos, other_robots_pos)

        desired_angle = np.arctan2(y_distance, x_distance)

        self.move(x_distance, y_distance, desired_angle, this_robot_orientation)
        
if __name__ == "__main__":

    #Initializing the node
    rospy.init_node("leader_follower", anonymous=True)

    # Calling the constructor function of the SubAndPub object
    SubAndPub()

    rospy.spin()

