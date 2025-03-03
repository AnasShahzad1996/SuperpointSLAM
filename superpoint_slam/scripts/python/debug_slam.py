#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from slam_interfaces.msg import Slamframe

class PoseReceiver(Node):
    def __init__(self):
        super().__init__("my_publisher")
        print("This is the pose receiver")
        self.mode = 0
        self.count = 0
        self.subscription = self.create_subscription(
            Slamframe,  # Replace YourMessageType with your actual message type
            "keyframes",  # Replace your_topic_name with the name of the topic you want to subscribe to
            self.callback,
            10,  # Adjust the queue size according to your needs
        )
        print("Pose receiver")

    def callback(self, msg):
        print (msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    my_receiver = PoseReceiver()
    try:
        rclpy.spin(my_receiver)
    except:
        print ("Exiting receiver node")
    my_receiver.destroy_node()
