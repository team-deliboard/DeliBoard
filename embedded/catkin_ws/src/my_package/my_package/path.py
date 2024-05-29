import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi,sqrt
import os

class pathPub(Node):

  def __init__(self):
    super().__init__('path_pub')
    
    # 로봇의 위치(/odom)를 받아서 전역 경로(/global_path), 지역 경로(/local_path)를 내보내주기 위해 publisher, subscriber 생성
    # 이 노드(path.py)로 들어오는 데이터 : odom / 계산 결과 나가는(출력) 데이터 : global_path, local_path
    self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
    self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
    self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

    self.odom_msg = Odometry()
    self.is_odom = False

    # global_path는 map 좌표계 기준으로 생성 됨
    self.global_path_msg = Path()
    self.global_path_msg.header.frame_id = 'map'

    # 텍스트파일(경로데이터)을 읽기 위해 경로를 지정
    self.f = open('C:\\Users\\SSAFY\\Desktop\\catkin_ws\\src\\test_209\\path\\test.txt','r')
    # 텍스트파일을 라인 별로 읽어서 리스트에 저장
    lines = self.f.readlines()
    # 라인을 split해서 x, y를 따로 나누고, global_path_msg에 append함
    for line in lines:
      tmp = line.split()
      read_pose=PoseStamped()
      read_pose.pose.position.x = float(tmp[0])
      read_pose.pose.position.y = float(tmp[1])
      read_pose.pose.orientation.w = 1.0
      self.global_path_msg.poses.append(read_pose)
    self.f.close()

    # 주기가 0.02초인 타이머 함수 설정
    # global_path는 텍스트 파일을 한 번 읽으면 끝이지만, local_path는 로봇이 움직일 때 마다 계속 업데이트 해줘야 하기 때문
    time_period = 0.02
    self.timer = self.create_timer(time_period, self.timer_callback)
    # local_path 경로점 사이즈
    self.local_path_size = 15
    self.count = 0

  # odometry 메시지 저장
  def listener_callback(self,msg):
    self.is_odom = True
    self.odom_msg = msg

  def timer_callback(self):
    if self.is_odom == True:  # odometry 메시지가 들어 왔으면
      local_path_msg = Path()  # local_path_msg 생성
      local_path_msg.header.frame_id = '/map'

      # 로봇의 위치 x, y
      x = self.odom_msg.pose.pose.position.x
      y = self.odom_msg.pose.pose.position.y

      # 최소값을 저장하기 위한 변수 초기 값을 inf로 설정
      # 로봇과 가장 가까운 경로 점 찾기
      min_dis =float('inf')
      current_waypoint = -1

      for i,waypoint in enumerate(self.global_path_msg.poses):  # 전역 경로점들 중 로봇과 가장 가까운 포인트를 찾는 과정
        distance = sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
        if distance < min_dis:
          min_dis = distance
          current_waypoint = i

      # local_path_msg에 데이터를 담는데, size만큼 있을 때, 없을 때 예외 처리
      if current_waypoint != -1:
        # 현재 경로점부터 local_path_size만큼 지역경로 메시지에 경로점을 넣음
        if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
          for num in range(current_waypoint, current_waypoint + self.local_path_size):
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
            tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
            tmp_pose.pose.orientation.w = 1.0
            local_path_msg.poses.append(tmp_pose)
        
        # 현재 경로점부터 local_path_size크기 만큼 경로점이 남아 있지 않는다면, 남은 경로점을 다 넣음
        else:
          for num in range(current_waypoint, len(self.global_path_msg.poses)):
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
            tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
            tmp_pose.pose.orientation.w = 1.0
            local_path_msg.poses.append(tmp_pose)
    
      # 지역경로(전역경로에서 뽑아온 경로점들)를 publish 한다. 추 후 경로 추종하는데 사용
      self.local_path_pub.publish(local_path_msg)

    # 타이머 콜백 함수는 0.02초인데, 전역경로를 0.02초 만큼 publish할 필요가 없어 10배 늦춤
    if self.count%10 == 0:
      self.global_path_pub.publish(self.global_path_msg)
    self.count += 1

def main(args = None):
  rclpy.init(args=args)
  test_path = pathPub()
  rclpy.spin(test_path)
  test_path.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()