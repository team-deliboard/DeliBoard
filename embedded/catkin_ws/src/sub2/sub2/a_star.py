import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin,sqrt
from collections import deque
import heapq

# a_star 노드는  OccupancyGrid map을 받아 grid map 기반 최단경로 탐색 알고리즘을 통해 로봇이 목적지까지 가는 경로를 생성하는 노드입니다.
# 로봇의 위치(/pose), 맵(/map), 목표 위치(/goal_pose)를 받아서 전역경로(/global_path)를 만들어 줍니다. 
# goal_pose는 rviz2에서 2D Goal Pose 버튼을 누르고 위치를 찍으면 메시지가 publish 됩니다. 
# 주의할 점 : odom을 받아서 사용하는데 기존 odom 노드는 시작했을 때 로봇의 초기 위치가 x,y,heading(0,0,0) 입니다. 로봇의 초기위치를 맵 상에서 로봇의 위치와 맞춰줘야 합니다. 
# 따라서 sub2의`` odom 노드를 수정해줍니다. turtlebot_status 안에는 정답데이터(절대 위치)가 있는데 그 정보를 사용해서 맵과 로봇의 좌표를 맞춰 줍니다.

# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 파라미터 설정
# 3. 맵 데이터 행렬로 바꾸기
# 4. 위치(x,y)를 map의 grid cell로 변환
# 5. map의 grid cell을 위치(x,y)로 변환
# 6. goal_pose 메시지 수신하여 목표 위치 설정
# 7. grid 기반 최단경로 탐색


class a_star(Node):
    def __init__(self):
        super().__init__('a_Star')
        # 로직 1. publisher, subscriber 만들기
        # odom은 로봇의 위치를 받아서 경로 탐색할 때 출발지로 찍기 위해서
        self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)  # goal_pose를 publish 하기 위해 생성

        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False


        # 로직 2. 파라미터 설정
        self.goal = [184,224]
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-6.5-8.75
        self.map_offset_y=9-8.75

        self.GRIDSIZE=350

        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]


    def grid_update(self):
        self.is_grid_update=True

        '''
        로직 3. 맵 데이터 행렬로 바꾸기
        메시리르 통해 받은 맵은 1차원 배열로 들어오기 때문에 2차원 행렬로 바꿔준다.
        '''
        map_to_grid = np.array(self.map_msg.data)
        self.grid = np.reshape(map_to_grid, (self.map_size_y, self.map_size_x))



    def pose_to_grid_cell(self,x,y):
        map_point_x = 0
        map_point_y = 0
        '''
        로직 4. 위치(x,y)를 map의 grid cell로 변환 
        (테스트) pose가 (-8,-4)라면 맵의 중앙에 위치하게 된다. 따라서 map_point_x,y 는 map size의 절반인 (175,175)가 된다.
        pose가 (-16.75,12.75) 라면 맵의 시작점에 위치하게 된다. 따라서 map_point_x,y는 (0,0)이 된다.
        위치(x,y)를 맵 cell로 변환하는 함수. 목적지나 출발지 같은 정보는 위치로 가지고 있다.
        '''
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return map_point_x,map_point_y


    def grid_cell_to_pose(self,grid_cell):

        x = 0
        y = 0
        
        '''
        로직 5. map의 grid cell을 위치(x,y)로 변환
        (테스트) grid cell이 (175,175)라면 맵의 중앙에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 중앙인 (-8,-4)가 된다.
        grid cell이 (350,350)라면 맵의 제일 끝 좌측 상단에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 좌측 상단인 (0.75,6.25)가 된다.
        최단경로 탐색 결과는 맵의 cell로 얻어지기 때문에 전역경로로 만들 때는 위치(x,y)로 변환행서 사용해야 한다.
        '''

        x = grid_cell[0] * self.map_resolution + self.map_offset_x
        y = grid_cell[1] * self.map_resolution + self.map_offset_y

        return [x,y]


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg
        # self.grid_update()

    def goal_callback(self,msg):
        print(f'x : {msg.pose.position.x}, y : {msg.pose.position.y}')
        
        if msg.header.frame_id=='map':
            
            '''
            로직 6. goal_pose 메시지 수신하여 목표 위치 설정
            '''
            goal_x, goal_y = self.pose_to_grid_cell(msg.pose.position.x, msg.pose.position.y)
            self.goal = (goal_x, goal_y)
            print(msg)
            print(f"Received goal: {self.goal}")

            
            if self.is_map ==True and self.is_odom==True  :
                if self.is_grid_update==False :
                    self.grid_update()
                    print("Grid updated.")
        
                self.final_path=[]

                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                print(f'now odom x, y : {x}, {y}')

                start_grid_cell=self.pose_to_grid_cell(x,y)
                print(f"Start grid cell: {start_grid_cell}")
                print(self.grid[start_grid_cell[0]][start_grid_cell[1]])
                print(self.grid[self.goal[0]][self.goal[1]])
                
                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
                self.cost = np.array([[400 * 400 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])

                
                # 다익스트라 알고리즘을 완성하고 주석을 해제 시켜주세요. 
                # 시작지, 목적지가 탐색가능한 영역이고, 시작지와 목적지가 같지 않으면 경로탐색을 합니다.
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] <= 50  and self.grid[self.goal[0]][self.goal[1]] <= 50  and start_grid_cell != self.goal :
                    print("Starting A* algorithm.")
                    self.final_path = self.a_star(start_grid_cell, self.goal)


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

    def a_star(self, start, goal):
        directions = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, sqrt(2)), (-1, 1, sqrt(2)), (1, -1, sqrt(2)), (1, 1, sqrt(2))]
        rows, cols = self.grid.shape

        if self.grid[start[0]][start[1]] > 50 or self.grid[goal[0]][goal[1]] > 50:
            return None

        open_list = []
        heapq.heappush(open_list, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_list:
            _, current_cost, current = heapq.heappop(open_list)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path

            for dx, dy, move_cost in directions:
                next = (current[0] + dx, current[1] + dy)
                if 0 <= next[0] < rows and 0 <= next[1] < cols and self.grid[next[0]][next[1]] <= 50:
                    # 이동 비용에 그리드 값(지형 난이도 등)을 반영하여 총 비용 계산
                    # 여기서는 단순화를 위해 grid 값이 0인 경우만 고려 (이동 가능한 경우)
                    new_cost = current_cost + move_cost  # 대각선 이동은 루트 2 비용
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + self.heuristic(next, goal)
                        heapq.heappush(open_list, (priority, new_cost, next))
                        came_from[next] = current
        return None


        
def main(args=None):
    rclpy.init(args=args)

    global_planner = a_star()

    rclpy.spin(global_planner)


    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
