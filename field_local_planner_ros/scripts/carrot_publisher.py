#!/usr/bin/env python

import rospy
import rospkg
from field_local_planner_msgs.msg import Status
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tr
import math


class CarrotPublisher:
    def __init__(self):
        # Default variables
        rospack = rospkg.RosPack()
        self.path_pkg = rospack.get_path("field_local_planner_ros")
        default_carrot_file = self.path_pkg + "/config/meshes/carrot/carrot.dae"

        # Read params
        self.goal_topic = rospy.get_param("~goal_topic", "/field_local_planner/current_goal")
        self.status_topic = rospy.get_param("~status_topic", "/field_local_planner/status")
        self.carrot_file = rospy.get_param("~carrot_mesh_file", default_carrot_file)
        self.carrot_rotation_freq = rospy.get_param("~carrot_rotation_freq", 5.0)  # Hz
        self.carrot_size = rospy.get_param("~carrot_size", 0.3)  # In meters
        self.carrot_vertical_offset = rospy.get_param("~carrot_vertical_offset", 0.5)  # In meters

        # Set subscriber
        self.status_pub = rospy.Subscriber(self.status_topic, Status, self.status_callback, queue_size=10)
        self.status_pub = rospy.Subscriber(self.status_topic, Status, self.status_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseWithCovarianceStamped, self.goal_callback, queue_size=10)

        # Set publisher
        self.carrot_pub = rospy.Publisher("/field_local_planner/real_carrot", Marker, queue_size=10)

        # Setup marker
        self.make_carrot_marker()
    
    def make_carrot_marker(self):
        self.marker = Marker()
        self.marker.id = 0
        self.marker.ns = "carrot"
        self.marker.scale.x = self.carrot_size
        self.marker.scale.y = self.carrot_size
        self.marker.scale.z = self.carrot_size
        self.marker.color.a = 1.0
        self.marker.mesh_use_embedded_materials = True
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = "file://" + self.carrot_file

    def status_callback(self, msg: Status):
        if msg.goal_reached:
            self.marker.action = Marker.DELETE
        else:
            self.marker.action = Marker.ADD

        # Publish
        self.marker.header.stamp = rospy.Time.now()
        try:
            self.carrot_pub.publish(self.marker)
        except rospy.exceptions.ROSException:
            pass

    def goal_callback(self, msg: PoseWithCovarianceStamped):
        t = msg.header.stamp.to_sec()
        yaw = math.fmod(self.carrot_rotation_freq * t, 2 * math.pi)
        q = tr.quaternion_about_axis(yaw, (0, 0, 1))
        pose = msg.pose.pose

        # Update the marker
        self.marker.header = msg.header
        self.marker.action = Marker.ADD
        
        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]

        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]
        self.marker.pose.position = pose.position
        self.marker.pose.position.z = self.marker.pose.position.z + self.carrot_vertical_offset    


if __name__ == "__main__":
    rospy.init_node("field_local_planner_carrot_publisher")
    carrot = CarrotPublisher()
    rospy.spin()