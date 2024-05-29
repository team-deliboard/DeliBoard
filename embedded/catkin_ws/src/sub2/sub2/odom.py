import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi,cos,sin
import tf2_ros
import geometry_msgs.msg
import time
import numpy as np
import sub2.locationInfo as locationInfo
# odom 노드는 로봇의 속도(/turtlebot_status), Imu센서(/imu) 메시지를 받아서 로봇의 위치를 추정하는 노드입니다.
# sub1_odom은 imu로 부터 받은 Quaternion을 사용하거나 각속도, 가속도 데이터를 이용해서 로봇의 포즈를 추정 할 것입니다.

# 노드 로직 순서
# 1. publisher, subscriber, broadcaster 만들기
# 2. publish, broadcast 할 메시지 설정
# 3. imu 에서 받은 quaternion을 euler angle로 변환해서 사용
# 4. 로봇 위치 추정
# 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast


class odom(Node):

    def __init__(self):
        super().__init__('odom')
        
        # 로직 1. publisher, subscriber, broadcaster 만들기
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 로봇의 pose를 저장해 publish 할 메시지 변수 입니다.
        self.odom_msg=Odometry()
        # Map -> base_link 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.base_link_transform=geometry_msgs.msg.TransformStamped()
        # base_link -> laser 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.laser_transform=geometry_msgs.msg.TransformStamped()
        self.is_status=False
        self.is_imu=False
        self.is_calc_theta=False
        # x,y,theta는 추정한 로봇의 위치를 저장할 변수 입니다.
        # map 시작 위치에 맞게 시작 위치를 지정
        self.x= locationInfo.odom_X
        self.y= locationInfo.odom_Y
                
        self.theta= pi / 2
        # imu_offset은 초기 로봇의 orientation을 저장할 변수 입니다.
        self.imu_offset=0
        self.prev_time=0

        
        # 로직 2. publish, broadcast 할 메시지 설정

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


    def imu_callback(self, msg):
        # Quaternion 데이터 추출 및 Euler 각도로 변환
        imu_q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        _, _, yaw = imu_q.to_euler()

        if self.is_imu == False:    
            self.is_imu = True
            # 첫 IMU 데이터로부터 yaw 값을 imu_offset으로 저장
            self.imu_offset = yaw
        else:
            # 로봇의 현재 방향(θ)를 업데이트: yaw에서 imu_offset을 빼줌
            self.theta = yaw - self.imu_offset

            # self.theta 값을 -π ~ π 범위로 정규화
            self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi / 2


    def listener_callback(self, msg):
        print('linear_vel : {}  angular_vel : {}'.format(msg.twist.linear.x,-msg.twist.angular.z))
        if self.is_imu ==True:
            if self.is_status == False :    # 처음 데이터가 들어왔을 때
                self.is_status=True
                self.prev_time=rclpy.clock.Clock().now()    # 이전 시간을 저장해 둠 (적분을 해야하기 때문)
            else :
                
                self.current_time=rclpy.clock.Clock().now()
                # 계산 주기를 저장한 변수 입니다. 단위는 초(s)
                self.period=(self.current_time-self.prev_time).nanoseconds/1000000000
                # 로봇의 선속도, 각속도를 저장하는 변수, 시뮬레이터에서 주는 각 속도는 방향이 반대이므로 (-)를 붙여줍니다.
                linear_x=msg.twist.linear.x
                angular_z=-msg.twist.angular.z
                
                # 로직 4. 로봇 위치 추정
                # (테스트) linear_x = 1, self.theta = 1.5707(rad), self.period = 1 일 때
                # self.x=0, self.y=1 이 나와야 합니다. 로봇의 헤딩이 90도 돌아가 있는
                # 상태에서 선속도를 가진다는 것은 x축방향이 아니라 y축방향으로 이동한다는 뜻입니다. 

                self.x += linear_x*cos(self.theta)*self.period
                self.y += linear_x*sin(self.theta)*self.period
                self.theta += 2*angular_z*self.period

                # 좌표계를 broadcast할 때는 시간을 꼭 넣어줘야 하기 때문에 시간을 넣어줌                
                self.base_link_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                
                
                # 로직 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast
                # 계산한 theta는 오일러 각이기 때문에 쿼터니언으로 변환
                q = Quaternion.from_euler(0, 0, self.theta)

                # 계산한 x, y가 이동 (translation) 값이 됨
                self.base_link_transform.transform.translation.x = self.x
                self.base_link_transform.transform.translation.y = self.y

                # 계산한 q가 회전 (rotation) 값이 됨
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                self.base_link_transform.transform.rotation.w = q.w

                # odometry 메시지도 이동, 회전, 제어 값을 채움
                self.odom_msg.pose.pose.position.x = self.x
                self.odom_msg.pose.pose.position.y = self.y
                self.odom_msg.pose.pose.orientation.x = q.x
                self.odom_msg.pose.pose.orientation.y = q.y
                self.odom_msg.pose.pose.orientation.z = q.z
                self.odom_msg.pose.pose.orientation.w = q.w
                self.odom_msg.twist.twist.linear.x = linear_x
                self.odom_msg.twist.twist.angular.x = angular_z

                # 좌표계를 broadcast하고, odometry 메시지를 publish 함
                self.broadcaster.sendTransform(self.base_link_transform)
                self.broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)
                self.prev_time=self.current_time

        
def main(args=None):
    rclpy.init(args=args)

    sub1_odom = odom()

    rclpy.spin(sub1_odom)


    sub1_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()