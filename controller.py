#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_interfaces.msg import Turtle
from my_interfaces.msg import TurtleArray
from my_interfaces.srv import TurtleTaken


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.turtle_hunt_ = None
        self.hunt_closest_ = True
        self.pose_ = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Turtle controller is now on.")

    def callback_pose(self, msg):
        self.pose_ = msg
    
    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.hunt_closest_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y                    
                    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_hunt_ = closest_turtle

            else: 
                self.turtle_hunt_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ == None or self.turtle_hunt_ == None: 
            return
        
        dist_x = self.turtle_hunt_.x - self.pose_.x
        dist_y = self.turtle_hunt_.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()

        if distance > 0.4:

            msg.linear.x = 2*distance
        
            goal_orientation = math.atan2(dist_y, dist_x)
            diff = goal_orientation - self.pose_.theta

            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = 6*diff
        else:
            msg.linear.x = 0.0
            msg.angular.z  = 0.0
            self.call_turtle_taken_server(self.turtle_hunt_.name)
            self.turtle_hunt_ = None

        self.cmd_vel_publisher_.publish(msg)
        
    def call_turtle_taken_server(self, turtle_name):
        client = self.create_client(TurtleTaken, "turtle_taken")
        while not client.wait_for_service(1.0):
          self.get_logger().warn("Waiting for Server ...")
        
        request = TurtleTaken.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_turtle_taken, turtle_name=turtle_name))
    
    def callback_call_turtle_taken(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " run so fast :(")

        except Exception as e:
            self.get_logger().error("Service call failed %r " % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()