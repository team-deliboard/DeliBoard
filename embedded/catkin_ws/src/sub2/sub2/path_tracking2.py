import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Point32, PoseStamped
from ssafy_msgs.msg import TurtlebotStatus, HandControl
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
import time, socket, json, threading
import sub2.locationInfo as locationInfo
from std_msgs.msg import String, Bool

HOST = 'j10a210.p.ssafy.io'
PORT = 54321
HOME_X = locationInfo.HOME_X
HOME_Y = locationInfo.HOME_Y

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,1)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)  # goal_pose를 publish 하기 위해 생성
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.flag1_pub = self.create_publisher(Bool, 'flag1', 1)
        self.flag2_pub = self.create_publisher(Bool, 'flag2', 1)
        self.canMove = True
        #self.canMove_sub = self.create_subscription(Bool, '/canMove', self.canMove_callback, 1)

        self.flag1 = False
        self.flag2 = False
        self.canMove = True
       
        # 로직 1. 제어 주기 및 타이머 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom=False
        self.is_path=False
        self.is_status=False
        self.is_lidar = False
        self.collision = False

        self.odom_msg=Odometry()
        self.path_msg=Path()
        self.cmd_msg=Twist()
        self.robot_yaw=0.0

        # 로직 2. 파라미터 설정
        self.lfd=0.1
        self.min_lfd=0.1
        self.max_lfd=2.0

        self.flag = True # home o : true, home x : false
        self.socket_flag = False # 목적지(room)에 도착하는 순간 True, socket 통신 response
        self.cnt = 0
        self.pathCnt = 0

        # PID 제어를 위한 변수
        self.Kp = 0.95
        self.Ki = 0.0
        self.Kd = 0.06
        self.prev_error = 0.0
        self.integral = 0.0


    def goal_callback(self, msg):
        # 목적지 집으로 업데이트
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        if self.goal_x != HOME_X or self.goal_y != HOME_Y:
            self.flag = False
            self.socket_flag = False

        self.canMove = True
        print(f'goal x, y : {self.goal_x}, {self.goal_y}')
    
    def update_pid(self, target_distance, current_distance):    
        if self.integral >= 0.2:
            self.integral = 0
        error = target_distance - current_distance
        self.integral += error
        derivative = error - self.prev_error

        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 오차가 충분히 작다고 판단되는 임계값
        error_threshold = 0.2
        # 오차가 임계값 이하일 경우 적분항 초기화
        if abs(error) < error_threshold:
            self.integral = 0.0

        self.prev_error = error

        return control

    def timer_callback(self):
        if self.is_status and self.is_odom and self.is_path and self.is_lidar and self.canMove:
            # 전방, 오른쪽, 왼쪽 벽까지의 거리 측정
            front_distances = self.lidar_msg.ranges[-30:] + self.lidar_msg.ranges[:30]  # 전방 80도 범위의 거리 측정
            right_diagonal_distances = self.lidar_msg.ranges[30:60]  # 오른쪽 대각선 방향의 거리
            left_diagonal_distances = self.lidar_msg.ranges[300:330]  # 왼쪽 대각선 방향의 거리
            right_distances = self.lidar_msg.ranges[60:120]  # 오른쪽 벽까지의 거리
            left_distances = self.lidar_msg.ranges[240:300]  # 왼쪽 벽까지의 거리


            # 최소 거리 계산 (0.1보다 큰 값들 중에서 최소값 선택)
            front_distance = np.mean([dist for dist in front_distances if dist > 0.1])
            right_diagonal_distance = np.mean([dist for dist in right_diagonal_distances if dist > 0.1])
            left_diagonal_distance = np.mean([dist for dist in left_diagonal_distances if dist > 0.1])
            right_distance = np.mean([dist for dist in right_distances if dist > 0.1])
            left_distance = np.mean([dist for dist in left_distances if dist > 0.1])

            target_distance = 0.5  # 목표 거리

            if front_distance < target_distance:
                # 전방에 물체가 있을 경우 후진
                self.cmd_msg.linear.x = -0.6
                self.cmd_msg.angular.z = 0.0  # 직진 후진이므로 각속도는 0
                self.cmd_pub.publish(self.cmd_msg)
                # self.integral = 0
                return
            elif right_distance < target_distance:
                control_signal = self.update_pid(target_distance, right_distance)
                self.cmd_msg.linear.x = 0.4
                self.cmd_msg.angular.z = control_signal * 1.1
                self.cmd_pub.publish(self.cmd_msg)
                return
            elif left_distance < target_distance:
                control_signal = self.update_pid(target_distance, left_distance)
                self.cmd_msg.linear.x = 0.4
                self.cmd_msg.angular.z = -control_signal * 1.22  # 음의 피드백으로 조정
                self.cmd_pub.publish(self.cmd_msg)
                return
            elif right_diagonal_distance < target_distance:
                control_signal = self.update_pid(target_distance, right_diagonal_distance)
                self.cmd_msg.linear.x = 0.4  # 대각선 방향이므로 속도 조절
                self.cmd_msg.angular.z = control_signal / 1.7  # 대각선 방향 조정을 위해 제어 신호 감소
                self.cmd_pub.publish(self.cmd_msg)
                return
            elif left_diagonal_distance < target_distance:
                control_signal = self.update_pid(target_distance, left_diagonal_distance)
                self.cmd_msg.linear.x = 0.4  # 대각선 방향이므로 속도 조절
                self.cmd_msg.angular.z = -control_signal / 1.7  # 음의 피드백으로 대각선 방향 조정
                self.cmd_pub.publish(self.cmd_msg)
                return
        # self.cnt+=1
        # if self.cnt == 20:
        #     print(f'status : {self.is_status}, odom : {self.is_odom}, path : {self.is_path}, lidar : {self.is_lidar}, canMove : {self.canMove}')
        #     self.cnt = 0
            else:
                if len(self.path_msg.poses)> 1:
                    self.is_look_forward_point= False

                    # 로봇의 현재 위치를 나타내는 변수
                    robot_pose_x=self.odom_msg.pose.pose.position.x
                    robot_pose_y=self.odom_msg.pose.pose.position.y
                    
                    goal_dis = sqrt(pow(self.goal_x - robot_pose_x, 2) + pow(self.goal_y - robot_pose_y, 2))
                    bool_msg = Bool()
                    bool_msg.data = True
                    
                    if goal_dis < 0.48 and bool_msg.data:
                        print('도착')
                        self.cmd_msg.linear.x = 0.0
                        self.cmd_msg.angular.z = 0.0
                        for _ in range(3):
                            self.cmd_pub.publish(self.cmd_msg)

                        self.canMove = False
                        self.flag1_pub.publish(bool_msg)
                        bool_msg.data = False
                        return

                    # 로봇이 경로에서 떨어진 거리를 나타내는 변수
                    lateral_error= sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))
                    #print(robot_pose_x,robot_pose_y,lateral_error)

                    self.lfd = (self.status_msg.twist.linear.x + lateral_error) * 0.5  # 로봇의 선속도, 경로로 부터 떨어진 거리를 이용해 전방주시거리 결정
                    # 계산한 전방주시거리가 최소, 최대값을 넘어갔으면 최소, 최대값으로 제한
                    if self.lfd < self.min_lfd:
                        self.lfd = self.min_lfd
                    if self.lfd > self.max_lfd:
                        self.lfd = self.max_lfd

                    min_dis=float('inf')

                    for num, waypoint in enumerate(self.path_msg.poses) : 
                        self.current_point = waypoint.pose.position

                        # 로봇과 가장 가까운 경로점과 모든 경로점과의 거리 탐색
                        dis = sqrt(pow(self.path_msg.poses[0].pose.position.x- self.current_point.x, 2) + pow(self.path_msg.poses[0].pose.position.y-self.current_point.y, 2))

                        # 정방주시거리에 가장 가깝게 있는 경로점 선택 
                        if abs(dis - self.lfd) < min_dis : 
                            min_dis = abs(dis-self.lfd)
                            self.forward_point = self.current_point
                            self.is_look_forward_point = True

                    if self.is_look_forward_point :
                        # print('foward')
                    # 전방주시포인트를 로봇 좌표계로 변경 후, 로봇과 전방주시포인트와의 각도 계산 
                        global_forward_point=[self.forward_point.x ,self.forward_point.y,1]

                        trans_matrix = np.array([
                            [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                            [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                            [0, 0, 1]])

                        det_trans_matrix = np.linalg.inv(trans_matrix)
                        local_forward_point = det_trans_matrix.dot(global_forward_point)
                        theta = -atan2(local_forward_point[1], local_forward_point[0])

                        # if self.collision == True:
                        #     print('collision')
                        #     self.cmd_msg.linear.x = 0.0
                        #     self.cmd_pub.publish(self.cmd_msg)

                        #else:
                        # print('track')
                        out_vel = 0.74
                        out_rad_vel=theta * 0.59  # 각속도를 theta에 2를 곱해서 사용. 클수록 더 빠르게 경로에 수렴

                        self.cmd_msg.linear.x=out_vel
                        self.cmd_msg.angular.z=out_rad_vel
                        self.cmd_pub.publish(self.cmd_msg)

                else :
                    print("no found forward point")
                    self.cmd_msg.linear.x=0.0
                    self.cmd_msg.angular.z=0.0
                    self.cmd_pub.publish(self.cmd_msg)
        else:
            self.cmd_msg.linear.x=0.0
            self.cmd_msg.angular.z=0.0
            self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
    
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _,_,self.robot_yaw = q.to_euler()

    # 각도 확인필요
    def lidar_callback(self, msg):
        self.pcd_msg = PointCloud()
        self.lidar_msg = msg
        self.pcd_msg.header.frame_id = "map"
        
        if self.is_path == True and self.is_odom == True:
            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw
            t = np.array([[cos(theta), -sin(theta), pose_x],
                        [sin(theta), cos(theta), pose_y],
                        [0, 0, 1]])

            for angle, r in enumerate(msg.ranges):
                global_point = Point32()
                if 0.0 < r < 12:
                    local_x = r * cos(angle * pi / 180)
                    local_y = r * sin(angle * pi / 180)
                    local_point = np.array([[local_x], [local_y], [1]])
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    self.pcd_msg.points.append(global_point)

            self.collision = False
            for waypoint in self.path_msg.poses:
                for lidar_point in self.pcd_msg.points:
                    distance = sqrt(pow(waypoint.pose.position.x - lidar_point.x, 2) +
                                    pow(waypoint.pose.position.y - lidar_point.y, 2))
                    if distance < 0.05:
                        self.collision = True
                        #print('collision')

            self.is_lidar = True

    def path_callback(self, msg):
        if self.pathCnt == 10:
            print('local_path call back')
        self.pathCnt+=1
        self.is_path=True
        self.path_msg=msg

    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg


def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()

    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()