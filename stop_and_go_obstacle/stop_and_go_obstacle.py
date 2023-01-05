import sys

import geometry_msgs.msg
import std_msgs.msg
import rclpy
from threading import Thread, Event

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
    
from rclpy.qos import QoSProfile
from rclpy.node import Node

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

class StopSub(Node):
    def __init__(self):
        super().__init__('sub')
        self.publishers_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel_stop_and_go', 10)
        self.subscriber_ = self.create_subscription(std_msgs.msg.Bool, 'have_obstacle', self.obstacle_callback, 10)
        self.stop_sign = True
        self.event = Event()
        self.pub_thread = Thread(target=self._thread_pub, daemon=True)
        self.pub_thread.start()

        if (self.stop_sign):
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.publishers_.publish(twist)

    def obstacle_callback(self, msg: std_msgs.msg.Bool):
        self.get_logger().info('Get Stop Sign')
        self.stop_sign = False

    def _thread_pub(self):
        
        while True:
            if self.event.is_set() :
                #thread destroy
                return
            if (not self.stop_sign):
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                self.publishers_.publish(twist)

        # if (msg.data):
        #     #self.get_logger().info('Get Stop Sign')
        #     self.stop_sign = True
        # elif (not msg.data):
        #     #self.get_logger().info('Get Go Sign')
        #     self.stop_sign = False

def main():
    rclpy.init()

    node = StopSub()
    rclpy.spin(node)
    
    node.event.set()
    node.destroy_node()
    rclpy.shutdown()
    
    # try:
    #     while True:
    #         if (node2.stop_sign):
    #             twist = geometry_msgs.msg.Twist()
    #             twist.linear.x = 0.0
    #             twist.linear.y = 0.0
    #             twist.linear.z = 0.0
    #             twist.angular.x = 0.0
    #             twist.angular.y = 0.0
    #             twist.angular.z = 0.0
    #             pub.publish(twist)

    # except Exception as e:
    #     print(e)

if __name__ == '__main__':
    main()
