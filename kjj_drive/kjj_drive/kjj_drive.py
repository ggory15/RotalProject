import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf2_msgs
import tf2_ros
import tf_transformations
import geometry_msgs.msg

import socket
import math
from threading import Thread, Event

class Kjj_drive(Node):
  def __init__(self):
    super().__init__('kjj_drive')
    self.get_logger().info("Init Start")
    #Publisher
    self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
    #Subscription
    self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self._sub_cmd_vel_callback, 10)
    #Brodcaster
    self.brodcaster_odom = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
    #init time
    self.currentTime = self.get_clock().now().nanoseconds
    self.lastTime = self.get_clock().now().nanoseconds
    #init odometry
    self._set_odom(0,0,0)
    #init velocityCmd socket
    self.velCmdSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.targetIP = '192.168.0.10'
    self.targetPort = 3450
    #Thread start
    self.event = Event()
    self.odom_thread = Thread(target=self._thread_odometry, daemon=True)
    self.odom_thread.start()

  def _sub_cmd_vel_callback(self, msg: Twist):
    sendMsg = "$VEL,%7f,%7f,%7f#" % (msg.linear.x, msg.linear.y, msg.angular.z)
    try:
      self.velCmdSock.sendto(sendMsg.encode(),('192.168.0.10',3450))
    except:
      self.velCmdSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.velCmdSock.sendto(sendMsg.encode(),('192.168.0.10',3450))

  def _thread_odometry(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.0.123", self.targetPort))

    while True:
      if self.event.is_set() :
        #thread destroy
        return

      try:
        recvMsg, addr = sock.recvfrom(128)
        #self.get_logger().info("msg recv!: " + recvMsg.decode())
      except:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("192.168.0.123", self.targetPort))
        recvMsg, addr = sock.recvfrom(128)

      recvStr = recvMsg.decode()
      data = recvStr[recvStr.find('$')+1:recvStr.find('#')]
      splitedData = data.split(',')
      if splitedData[0] == 'VEL' :
        vx = float(splitedData[1])
        vy = float(splitedData[2])
        vth = float(splitedData[3])

      self._odom_calc(vx, vy, -vth)
      self._odom_publish(vx,vy, -vth)

  def _odom_calc(self, vx, vy, vth):
    self.currentTime = self.get_clock().now().nanoseconds
    dt = (self.currentTime / 1000000000 - self.lastTime / 1000000000)
    dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
    dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
    dth = vth * dt

    self.x += dx
    self.y += dy
    self.th += dth

    self.lastTime = self.currentTime

  def _odom_publish(self, vx, vy, vth):
    #quaternion created from yaw
    quatData = tf_transformations.quaternion_from_euler(0, 0, self.th)
    odom_quat = Quaternion(x=quatData[0], y=quatData[1], z=quatData[2], w=quatData[3])

    #1st publish the transform
    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header.stamp = self.get_clock().now().to_msg()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'base_link'

    odom_trans.transform.translation.x = self.x
    odom_trans.transform.translation.y = self.y
    odom_trans.transform.translation.z = 0.0

    odom_trans.transform.rotation = odom_quat

    self.brodcaster_odom.sendTransform(odom_trans)

    #2nd publish the odometry
    odom = Odometry()
    odom.header.stamp = self.get_clock().now().to_msg()
    odom.header.frame_id = 'odom'

    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = odom_quat
    self.pub_odom.publish

    odom.child_frame_id = 'base_link'
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    self.pub_odom.publish(odom)

  def _set_odom(self, x, y, th):
    self.x = x
    self.y = y
    self.th = th

def main(args=None):
  rclpy.init(args= args)

  kjj_drive = Kjj_drive()

  rclpy.spin(kjj_drive)

  kjj_drive.event.set()
  kjj_drive.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
