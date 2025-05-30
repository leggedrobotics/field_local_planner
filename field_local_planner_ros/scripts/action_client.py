#!/usr/bin/env python

import rospy
import rospkg
import actionlib
from field_local_planner_msgs.msg import MoveToAction, MoveToGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tr
import math


class ActionClient:
    def __init__(self):
        # Read params
        self.goal_topic = rospy.get_param("field_local_planner/goal_topic", "/initialpose")
        self.action_server_name = rospy.get_param("field_local_planner/action_server_name", '/field_local_planner/action_server')
        print(self.action_server_name)
        
        # Set subscriber
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseWithCovarianceStamped, self.goal_callback, queue_size=10)
        
        # Setup action client
        self.client = actionlib.SimpleActionClient(self.action_server_name, MoveToAction)
        self.client.wait_for_server()

        rospy.loginfo(f"ActionClient initialized with server [{self.action_server_name}]. Listening to topic {self.goal_topic}")

        # Spin
        rospy.spin()

    def goal_callback(self, msg: PoseWithCovarianceStamped):
        goal = MoveToGoal()
        goal.goal = msg

        # rospy.loginfo(f"Sending new goal:\n{goal.goal}")
        self.client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("field_local_planner_action_client")
    client = ActionClient()