import rclpy
from rclpy.node import Node
from squaternion import Quaternion  # 쿼터니언에서 오일러 각, 오일러 각에서 쿼터니언으로 변환하기 위한 모듈 import
from nav_msgs.msg import Odometry
from ssafy_msgs.msg import TurtlebotStatus  # 터틀봇 메시지 타입을 사용하기 위해 import
from math import pi,cos,sin
import tf2_ros  # tf2_ros에 broadcaster를 사용하기 위해 import
# tf2_ros
# ROS에서 변환을 다루는 데 사용되는 다양한 기능 제공
# 시간에 따른 변환 관리, 변환 트리 유지 관리, 변환 요청 및 발행, 변환 메시지의 송수신 등
# tf2_ros.broadcaster
# tf 시스템에서 변환을 송신하는 데 사용
import geometry_msgs.msg

class odom(Node):

  def __init__(self):
    super().__init__('odom')

    # 노드 => 터틀봇의 상태 메시지를 읽고 odometry를 출력하는...
    self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
    self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
    # StaticTransformBroadcaster
    # 시간에 따라 변하지 않는 변환
    # 로봇의 부속물의 상대적인 위치나 회전, 레이저 스캐너의 설치 위치 등
    self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
    self.odom_msg = Odometry()
    # map과 base_link 간의 좌표계
    self.base_link_transform = geometry_msgs.msg.TransformStamped()
    # base_link와 laser 간의 좌표계
    self.laser_transfrom = geometry_msgs.msg.TransformStamped() 
    # 메시지가 들어왔는지 여부를 확인하기 위한 변수
    self.is_status = False
    self.is_calc_theta = False


    self.x = 0
    self.y = 0
    self.theta = 0
    self.prev_time = 0

    # odometry 메시지에 좌표계 이름을 적어 준다. 'map' 좌표계 위에 odometry가 그려짐
    self.odom_msg.header.frame_id = 'map'
    self.odom_msg.child_frame_id = 'base_link'

    # 'map' -> 'base_link' 좌표계 이름 설정
    self.base_link_transform = geometry_msgs.msg.TransformStamped()
    self.base_link_transform.header.frame_id = 'map'
    self.base_link_transform.child_frame_id = 'base_link'

    # 'base_link' -> 'laser' 좌표계 이름 설정
    self.laser_transfrom = geometry_msgs.msg.TransformStamped()
    self.laser_transfrom.header.frame_id = 'base_link'
    self.laser_transfrom.child_frame_id = 'laser'

    # 'base_link'로 부터 laser 좌표계가 이동(translation)된 값 입력. 터틀봇으로 부터 1m 위에 달려있다는 것을 의미 (고정된 위치 -> 고정된 값)
    # init에 넣은 이유는 센서의 위치는 바뀌지 않기 때문에  
    self.laser_transfrom.transform.translation.x = 0.0
    self.laser_transfrom.transform.translation.y = 0.0
    self.laser_transfrom.transform.translation.z = 1.0

    self.laser_transfrom.transform.rotation.w = 1.0
  
  def status_callback(self, msg):
    
    if self.is_status == False:  # 처음 데이터가 들어왔을 때
      self.is_status = True
      self.prev_time = rclpy.clock.Clock().now()  # 이전 시간을 저장해 둠 (적분을 해야하기 때문)
    
    else:
      self.current_time = rclpy.clock.Clock().now()
      self.period = (self.current_time - self.prev_time).nanoseconds/1000000000

      # a = b (b : 상태 메시지에 있는 정보)
      linear_x = msg.twist.linear.x
      angular_z = -msg.twist.angular.z

      # odometry 수식, 적분의 효과를 가져온다
      self.x += linear_x*cos(self.theta)*self.period
      self.y += linear_x*sin(self.theta)*self.period
      self.theta += 2*angular_z*self.period

      # 계산한 theta는 오일러 각이기 때문에 쿼터니언으로 변환
      q = Quaternion.from_euler(0, 0, self.theta)

      # 좌표계를 broadcast할 때는 시간을 꼭 넣어줘야 하기 때문에 시간을 넣어줌
      self.base_link_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
      self.laser_transfrom.header.stamp = rclpy.clock.Clock().now().to_msg()

      # 계산한 x, y가 이동 (translation) 값이 됨
      self.base_link_transform.transform.translation.x = self.x
      self.base_link_transform.transform.translation.y = self.y

      # 계산한 q가 회전 (rotation) 값이 됨
      self.base_link_transform.transform.rotation.x = q.x
      self.base_link_transform.transform.rotation.y = q.y
      self.base_link_transform.transform.rotation.z = q.z
      self.base_link_transform.transform.rotation.w = q.w

      # odometry 메시지도 이동, 회전, 제어 값을 채움
      # 현재 위치의 x, y 좌표
      self.odom_msg.pose.pose.position.x = self.x
      self.odom_msg.pose.pose.position.y = self.y
      # 현재 방향을 나타내는 쿼터니언
      self.odom_msg.pose.pose.orientation.x = q.x
      self.odom_msg.pose.pose.orientation.y = q.y
      self.odom_msg.pose.pose.orientation.z = q.z
      self.odom_msg.pose.pose.orientation.w = q.w
      # 현재 선속도와 각속도
      self.odom_msg.twist.twist.linear.x = linear_x
      self.odom_msg.twist.twist.angular.x = angular_z

      # 좌표계를 broadcast하고, 
      self.broadcaster.sendTransform(self.base_link_transform)
      self.broadcaster.sendTransform(self.laser_transfrom)
      # odometry 메시지를 publish 함
      self.odom_publisher.publish(self.odom_msg)
      
      self.prev_time = self.current_time

def main(args = None):
  rclpy.init(args=args)
  odom_node = odom()
  rclpy.spin(odom_node)
  odom_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()