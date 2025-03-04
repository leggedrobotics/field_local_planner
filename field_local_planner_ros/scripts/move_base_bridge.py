#!/usr/bin/env python3
"""
  @file move_base_bridge.py
  @brief Bridge for bridging MoveBase actions to the proprietary field local planner
  @author Tobit Flatscher
  @copyright Oxford Robotics Institute (2024)
"""

import actionlib
from field_local_planner_msgs.msg import MoveToAction, MoveToGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
import rospy


class MoveBaseToMoveToBridge:
    """
      ROS action bridge for bridging MoveBase action goals to MoveTo
    """

    def __init__(self):
        """
          Initialise the action client and action server
        """
        # Create action client for MoveTo
        rospy.loginfo("Initializing MoveTo client...")
        self._move_to_client = actionlib.SimpleActionClient("/field_local_planner/action_server",
          MoveToAction)
        self._move_to_client.wait_for_server()
        rospy.loginfo("Successfully connected to MoveTo server...")

        rospy.loginfo("Initialising MoveBase server...")
        self._move_base_server = actionlib.SimpleActionServer("/move_base",
          MoveBaseAction, self._bridge, False)
        self._move_base_server.start()
        rospy.loginfo("Successfully initialised MoveBase server...")
        return

    def _bridge(self, goal: MoveBaseGoal) -> None:
        """
          Bridge the move_base_msgs::MoveBase to a field_local_planner_msgs::MoveTo
        """
        rospy.loginfo("Received new MoveBase goal...")
        move_to_goal = MoveToGoal()
        move_to_goal.goal.header = goal.target_pose.header
        move_to_goal.goal.pose.pose = goal.target_pose.pose
        rospy.logdebug("Moving to " + str(goal) + "...")
        self._move_to_client.send_goal(move_to_goal)

        while (
            not self._move_to_client.wait_for_result(timeout=rospy.Duration(0.2))
            and not rospy.is_shutdown()
        ):
            if self._move_base_server.is_preempt_requested():
                rospy.loginfo("Preempting goal...")
                self._move_to_client.cancel_goal()
                self._move_base_server.set_preempted()
                return

        rospy.loginfo("MoveTo completed!")
        move_base_result = MoveBaseResult()
        self._move_base_server.set_succeeded(move_base_result)
        return

if __name__ == '__main__':
    rospy.init_node("move_base_bridge")
    bridge = MoveBaseToMoveToBridge()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

