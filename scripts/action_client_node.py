#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotState  # Custom message
from geometry_msgs.msg import Point  # Incoming goal
from std_msgs.msg import Float32    # For cancel and result

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node')

        # Action client setup
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server connected!")

        # Publishers
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        self.goal_result_pub = rospy.Publisher('/goal_result', Float32, queue_size=10)
        self.last_target_pub = rospy.Publisher('/last_target', Point, queue_size=10)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/goal_topic', Point, self.goal_callback)
        rospy.Subscriber('/cancel_goal_topic', Float32, self.cancel_callback)

        # State
        self.robot_state = RobotState()

    def odom_callback(self, msg):
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        self.robot_state.vel_x = msg.twist.twist.linear.x
        self.robot_state.vel_z = msg.twist.twist.angular.z
        self.state_pub.publish(self.robot_state)

    def goal_callback(self, msg):
        rospy.loginfo(f"New goal received: ({msg.x}, {msg.y})")
        self.send_goal(msg.x, msg.y)

    def cancel_callback(self, msg):
        rospy.loginfo("Goal cancelation requested")
        self.cancel_goal()

    def send_goal(self, x, y):
        # Publish the last goal
        target_point = Point(x=x, y=y, z=0.0)
        self.last_target_pub.publish(target_point)

        # Create and send action goal
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal,
                              done_cb=self.done_callback,
                              feedback_cb=self.feedback_callback)
        rospy.loginfo(f"Goal sent to action server: ({x}, {y})")

    def cancel_goal(self):
        self.client.cancel_goal()
        self.goal_result_pub.publish(Float32(0.0))  # 0.0 means cancelled

    def feedback_callback(self, feedback):
        # Optional: log feedback
        rospy.logdebug(f"Feedback: {feedback}")

    def done_callback(self, status, result):
        # Action finished — report success
        rospy.loginfo("Goal reached successfully")
        self.goal_result_pub.publish(Float32(1.0))  # 1.0 means reached

    def spin(self):
        rospy.loginfo("Action Client Node is running...")
        rospy.spin()

if __name__ == "__main__":
    node = ActionClientNode()
    node.spin()

