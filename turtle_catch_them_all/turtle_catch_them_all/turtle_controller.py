#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle  # type: ignore
from my_robot_interfaces.msg import TurtleArray  # type: ignore
from my_robot_interfaces.srv import CatchTurtle
from functools import partial


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")  # type: ignore

        self.turtle_to_catch = None 

        self.pose = None
        self.cmd_vel_pub = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turle_subscriber = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch = msg.turtles[0]

    def control_loop(self):
        if self.pose is None or self.turtle_to_catch == None:
            return

        distance_x = self.turtle_to_catch.x - self.pose.x
        distance_y = self.turtle_to_catch.y - self.pose.y
        distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

        msg = Twist()

        if distance > 0.5:
            # position
            msg.linear.x = 2 * distance

            # orientation
            goal_theta = math.atan2(distance_y, distance_x)
            diff = goal_theta - self.pose.theta

            # Normalize the angle difference to the range [-π, π]
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            msg.angular.z = 6 * diff

        else:
            # target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch.name)
            self.turtle_to_catch = None

        self.cmd_vel_pub.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial
                                 (self.callback_call_catch_turtle, turtle_name=turtle_name))

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be caught")

        except Exception as e:
            self.get_logger().warning("Service call Failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
