#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ActionClient:
    def __init__(self):
        # Read params
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.action_server_name = rospy.get_param("~action_server_name", '/move_base')
        
        # Set subscriber
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=10)
        
        # Setup action client
        self.client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)
        self.client.wait_for_server()

        rospy.loginfo(f"ActionClient initialized with server [{self.action_server_name}]. Listening to topic {self.goal_topic}")

        # Spin
        rospy.spin()

    def goal_callback(self, msg: PoseStamped):
        goal = MoveBaseGoal()
        goal.target_pose = msg

        rospy.loginfo(f"Sending new goal:\n{goal.target_pose}")
        self.client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("move_base_simple")
    client = ActionClient()
