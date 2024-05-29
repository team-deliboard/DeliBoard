import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose,PoseStamped, Twist
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin
from collections import deque
from std_msgs.msg import Bool
from ssafy_msgs.msg import TurtlebotStatus, HandControl

import socket, json, threading, time, queue
import sub2.locationInfo as locationInfo

# Server address
HOST = 'j10a210.p.ssafy.io'
PORT = 54321

drink = locationInfo.drink
game1 = locationInfo.game1
game2 = locationInfo.game2
game3 = locationInfo.game3
room = locationInfo.room
HOME_X = locationInfo.HOME_X
HOME_Y = locationInfo.HOME_Y
order_queue = queue.Queue()

lock = threading.Lock()
objectStatus = [False,False,False] 

# 0 : order, 1 : return, 2 : drink

class a_star(Node):

    def __init__(self):
        super().__init__('a_Star')
        # 로직 1. publisher, subscriber 만들기
        # odom은 로봇의 위치를 받아서 경로 탐색할 때 출발지로 찍기 위해서
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,1)
        self.goal_sub = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,1)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)  # goal_pose를 publish 하기 위해 생성
        self.flag1_sub = self.create_subscription(Bool, '/flag1', self.flag1_callback, 5)
        #self.flag2_sub = self.create_subscription(Bool, '/flag2', self.flag2_callback, 5)

        self.socket_connection_thread = threading.Thread(target = self.socket_connection)  # socket 통신
        self.goal_pub_thread = threading.Thread(target = self.goal_pub_queue)
        
        self.obj_pub = self.create_publisher(HandControl, 'hand_control', 10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.canMove_pub = self.create_publisher(Bool, 'canMove', 3)
        
        self.cmd_msg=Twist()
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        #self.is_found_path=False
        self.is_grid_update=False
        
        # goal_msg
        self.goal_msg = PoseStamped()
        self.goal_msg.header.stamp = self.get_clock().now().to_msg()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.z = 0.0

        self.goal_msg.pose.orientation.x = 0.0
        self.goal_msg.pose.orientation.y = 0.0
        self.goal_msg.pose.orientation.z = 0.0
        self.goal_msg.pose.orientation.w = 1.0

        # command(move) done
        self.flag1 = False
        self.flag2 = False

        # 로직 2. 파라미터 설정
        self.goal = [184,224]
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x= -6.5 - 8.75
        self.map_offset_y= 9.0 - 8.75

        self.GRIDSIZE=350

        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]
        
        self.client_socket = None
        self.socket_connection_thread.start()
        self.goal_pub_thread.start()

    def moveRobot(self, x, y):
        self.goal_msg.pose.position.x = float(x)
        self.goal_msg.pose.position.y = float(y)
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        for _ in range(3):
            self.cmd_pub.publish(self.cmd_msg)
        
        time.sleep(2)

        self.goal_pub.publish(self.goal_msg)

    def goal_pub_queue(self):
        cnt = 0
        readyflag = True
        command = ""

        while True:
            if order_queue.empty(): # 비어있으면 스킵
                cnt = 0
                continue

            if readyflag:
                cnt += 1
                x, y, command = order_queue.get()

                if cnt == 2 and command == "1":
                    tmp = float(y) + 0.033
                    y = tmp

                if cnt == 1 and command == "0":
                    tmp = float(y) - 0.025
                    y = tmp

                self.moveRobot(x, y)  # goal publish
                readyflag = False

            if  self.flag1:
                print('도착')
                self.flag1 = False
                self.cmd_msg.linear.x = 0.01
                self.cmd_msg.angular.z = -0.3
                for _ in range(3):
                    self.cmd_pub.publish(self.cmd_msg)

                if cnt == 1:
                    print('lift')
                    # object(2)
                    self.object_control(2, command)

                    if command == "1":  # 반납 명령일 때 obj 들면 response
                        self.socket_response(command)

                elif cnt == 2:
                    self.cmd_msg.linear.x = 0.01
                    self.cmd_msg.angular.z = -0.3
                    for _ in range(3):
                        self.cmd_pub.publish(self.cmd_msg)

                    print('put')
                    # object(3)
                    self.object_control(3, command)

                    if command == "0":  # 주문 명령일 때 obj 내리면
                        self.socket_response(command)

                elif cnt == 3:
                    print('home')

                self.cmd_msg.linear.x = -0.1
                self.cmd_msg.angular.z = 0.0
                for _ in range(3):
                    self.cmd_pub.publish(self.cmd_msg)

                readyflag = True

    def flag1_callback(self, msg):
        self.flag1 = msg.data

    def status_callback(self, msg):
        lock.acquire()
        objectStatus[0]=msg.can_lift
        objectStatus[1]=msg.can_put
        objectStatus[2]=msg.can_use_hand
        lock.release()
    
    def object_control(self, mode, command):
        a=int(mode)
        self.obj_msg=HandControl() 
        self.obj_msg.control_mode=int(a)

        self.cmd_msg.linear.x = 0.05
        self.cmd_msg.angular.z = 0.0
        #cnt = 0
        # if command == "return" and a == 3:
        #     self.cmd_msg.linear.x = 0.01
        #     self.cmd_msg.angular.z = -0.3
        self.obj_msg.put_distance = 1.17
        self.obj_msg.put_height= 0.5

        if command == "1":
            self.obj_msg.put_distance = 1.44
            self.obj_msg.put_height = 0.7

        for _ in range(3):
            self.cmd_pub.publish(self.cmd_msg)
        
        time.sleep(1)

        if a == 2:
            while not bool(objectStatus[2]):
                self.cmd_pub.publish(self.cmd_msg)
                self.obj_pub.publish(self.obj_msg)    

        elif a == 3:
            while bool(objectStatus[2]):
                self.cmd_pub.publish(self.cmd_msg)
                self.obj_pub.publish(self.obj_msg)
                #cnt+=1
        
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        for _ in range(3):
            self.cmd_pub.publish(self.cmd_msg)

    # socket 첫 통신 연결
    def socket_connection(self):
        while True:
            try:
                # socket start
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print(f"try to connect {HOST}:{PORT}...")

                # connect & 처음엔 turtle message 보내기
                try:
                    self.client_socket.connect((HOST, PORT))
                    print("connected to Server!")

                    time.sleep(1)
                    self.client_socket.sendall("turtle".encode('utf-8'))

                    # self.client_socket.settimeout(30)

                    # data receive
                    while True:
                        json_data = None
                        # receive command
                        try:
                            response = self.client_socket.recv(1024)

                            if response:
                                json_data = json.loads(response.decode('utf-8'))
                                print(f'recv_data : {json_data}')
                                # self.client_socket.sendall("Success".encode('utf-8'))
                                # print('Send response "Success"')

                                command = json_data['command']
                                x = json_data['locationX']
                                y = json_data['locationY']

                                if str(command) == "0":
                                    order_queue.put((x, y, "0"))
                                    order_queue.put((room[0], room[1], "0"))
                                    order_queue.put((HOME_X, HOME_Y, "0"))
                                    print("qeueu : game -> room")

                                elif str(command) == "1":
                                    order_queue.put((room[0], room[1], "1"))
                                    order_queue.put((x, y, "1"))
                                    order_queue.put((HOME_X, HOME_Y, "1"))
                                    print("qeueu : room -> game or drink")
                                
                                elif str(command) == "2":
                                    order_queue.put((x, y, "2"))
                                    order_queue.put((room[0], room[1], "2"))
                                    order_queue.put((HOME_X, HOME_Y, "2"))
                                    print("qeueu : drink -> room")

                                print('queue put done')

                            else:
                                print("No data received!")
                                break
                       
                        except Exception as e:
                            print("An error occurred:", e)
                        # except socket.timeout:
                        #     print("Timed out while waiting for data")

                except Exception as e:
                    print("An error occurred:", e)
                    
            except Exception as e:
                    print(f'error in goal_pose_receive : {e}')
            
            time.sleep(1)
            self.client_socket.close()
    
    def socket_response(self, command):
        try:
            response = {"message":"Success", "roomNumber" : 1, "storeId" : 1, "command" : command}
            self.client_socket.sendall(json.dumps(response).encode('utf-8'))
            print(f'send response : {response}')

        except Exception as e:
                    print(f'error in socket_reponse : {e}')

    def grid_update(self):
        self.is_grid_update=True

        map_to_grid = np.array(self.map_msg.data)
        self.grid = np.reshape(map_to_grid, (self.map_size_y, self.map_size_x))

    def pose_to_grid_cell(self,x,y):
        map_point_x = 0
        map_point_y = 0

        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return map_point_x,map_point_y


    def grid_cell_to_pose(self,grid_cell):

        x = 0
        y = 0

        x = grid_cell[0] * self.map_resolution + self.map_offset_x
        y = grid_cell[1] * self.map_resolution + self.map_offset_y

        return [x,y]


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg

    def goal_callback(self,msg):
        print('==========================receive goal_pose==========================')
        print(f'x : {msg.pose.position.x}, y : {msg.pose.position.y}')
        
        if msg.header.frame_id=='map':
            
            goal_x, goal_y = self.pose_to_grid_cell(msg.pose.position.x, msg.pose.position.y)
            self.goal = (goal_x, goal_y)
            #print(msg)
            
            if self.is_map ==True and self.is_odom==True  :
                if self.is_grid_update==False :
                    self.grid_update()
        
                self.final_path=[]

                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                # print(f'now odom x, y : {x}, {y}')
                start_grid_cell=self.pose_to_grid_cell(x,y)
                # print(f'start_grid_cell : {start_grid_cell}')
                # print(f'goal_grid_cell : {self.goal}')

                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
                
                self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])

                
                # 다익스트라 알고리즘을 완성하고 주석을 해제 시켜주세요. 
                # 시작지, 목적지가 탐색가능한 영역이고, 시작지와 목적지가 같지 않으면 경로탐색을 합니다.
                # print('==========if check==========')
                # print(f'self.grid[start_grid_cell[0]][start_grid_cell[1]] : {self.grid[start_grid_cell[0]][start_grid_cell[1]]}')
                # print(f'self.grid[self.goal[0]][self.goal[1]] : {self.grid[self.goal[0]][self.goal[1]]}')
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] >= 100:
                    self.grid[start_grid_cell[0]][start_grid_cell[1]] = 0

                if self.grid[self.goal[0]][self.goal[1]] >= 100:
                    self.grid[self.goal[0]][self.goal[1]] = 0

                if self.grid[start_grid_cell[0]][start_grid_cell[1]] <= 50  and self.grid[self.goal[0]][self.goal[1]] <= 50  and start_grid_cell != self.goal :
                    # print('시작지, 목적지가 탐색가능한 영역이고, 시작지와 목적지가 같지 않으면 경로탐색을 합니다.')
                    self.dijkstra(start_grid_cell)


                self.global_path_msg=Path()
                self.global_path_msg.header.frame_id='map'
                for grid_cell in reversed(self.final_path) :
                    tmp_pose=PoseStamped()
                    waypoint_x,waypoint_y=self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x=waypoint_x
                    tmp_pose.pose.position.y=waypoint_y
                    tmp_pose.pose.orientation.w=1.0
                    self.global_path_msg.poses.append(tmp_pose)

                if len(self.final_path)!=0 :
                    print("경로 생성 o")
                    self.a_star_pub.publish(self.global_path_msg)
                
                else :
                    print("경로 생성 X")

    def heuristic(self, node, goal):
        D = 1
        D2=np.sqrt(2)
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def dijkstra(self,start):
        print(f'dijkstra start, start : {start}')
        
        Q = deque()
        Q.append(start)
        self.cost[start[0]][start[1]] = 0
        while Q:
            current = Q.popleft()
            #print(f'currnet pos : {current}')

            if current == self.goal:
                break
            for i in range(8):
                next = (current[0] + self.dx[i], current[1] + self.dy[i])
                #print(f'next[0] : {next[0]}, next[1] : {next[1]}')
                if 0 <= next[0] < self.GRIDSIZE and 0 <= next[1] < self.GRIDSIZE:
                    if self.grid[next[1]][next[0]] < 50:  # 장애물 확인
                        nextCost = self.cost[current[0]][current[1]] + self.dCost[i]

                        # # 반발력 적용
                        # if self.is_obstacle_close(next):
                        #     nextCost += self.obstacle_repulsive_force(next)
                        
                        if self.cost[next[0]][next[1]] > nextCost:
                            Q.append(next)
                            self.path[next[0]][next[1]] = current
                            self.cost[next[0]][next[1]] = nextCost
        # 경로 추적
        node = self.goal
        #print(f'path[node[0]] : {self.path[node[0]]}, path[node[1]] : {self.path[node[1]]}')
        while node != start:
            self.final_path.append(node)
            try:
                node = self.path[node[0]][node[1]]
            except Exception as e:
                print(f'error : {e}')
                print(f'node[0] : {node[0]}, node[1] : {node[1]}')
    
    # def is_obstacle_close(self, node):
    # # 장애물 주변에 있는지 여부를 확인하는 함수
    # # 장애물 주변에 있다면 True, 아니면 False 반환
    # # 여기서는 장애물로 판단할 셀의 값이 50 미만인지를 확인하여 장애물로 판단
    # # 일정 범위 내에 장애물이 있는 경우 True 반환
    #     obstacle_range = 5  # 장애물을 인지할 범위
    #     for i in range(-obstacle_range, obstacle_range+1):
    #         for j in range(-obstacle_range, obstacle_range+1):
    #             if 0 <= node[0]+i < self.GRIDSIZE and 0 <= node[1]+j < self.GRIDSIZE:
    #                 if self.grid[node[1]+j][node[0]+i] < 50:
    #                     return True
    #     return False

    # def obstacle_repulsive_force(self, node):
    #     # 장애물로부터의 반발력을 계산하는 함수
    #     # 장애물과의 거리에 비례하여 반발력을 계산
    #     # 일정 거리 이상의 장애물에는 영향을 미치지 않도록 함
    #     obstacle_range = 5  # 장애물을 인지할 범위
    #     max_repulsive_force = 10  # 최대 반발력
        
    #     repulsive_force = 0
    #     for i in range(-obstacle_range, obstacle_range+1):
    #         for j in range(-obstacle_range, obstacle_range+1):
    #             if 0 <= node[0]+i < self.GRIDSIZE and 0 <= node[1]+j < self.GRIDSIZE:
    #                 distance = ((i)**2 + (j)**2)**0.5
    #                 if distance <= obstacle_range:
    #                     repulsive_force += max_repulsive_force * (1 - distance/obstacle_range)
    #     return repulsive_force
        

        
def main(args=None):
    rclpy.init(args=args)

    global_planner = a_star()

    rclpy.spin(global_planner)


    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
