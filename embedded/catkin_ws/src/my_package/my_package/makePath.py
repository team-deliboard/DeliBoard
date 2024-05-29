import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
import os
from math import sqrt

class makePath(Node):

    def __init__(self):
        super().__init__('make_path')

        self.path_pub = self.create_publisher(Path, 'global_path', 10)  # global_path pub
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)  # 위치 값 받고

        pkg_path = os.getcwd()
        print(pkg_path)
        back_folder = '..'
        folder_name = 'path'
        file_name = 'test.txt'
        full_path = os.path.join(pkg_path,folder_name,file_name)
        print(full_path)
        self.f = open(full_path, 'w')
        # self.f.write('==========open & write==========')
        self.f.flush()

        self.is_odom = True
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
    
    def listener_callback(self, msg):
        if self.is_odom == False :
            self.is_odom = True
            self.prev_x = msg.pose.pose.position.x
            self.prev_y = msg.pose.pose.position.y
        else:
            waypoint_pose = PoseStamped()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

            if distance > 0.1:
                waypoint_pose.pose.position.x = x
                waypoint_pose.pose.position.y = y
                waypoint_pose.pose.orientation.w = 1.0
                self.path_msg.poses.append(waypoint_pose)
                self.path_pub.publish(self.path_msg)
                data = '{0}\t{1}\n'.format(x, y)
                print('data : ' + data)
                # self.f.write('test')
                self.f.write(data)
                self.f.flush()
                self.prev_x = x
                self.prev_y = y

def main(args=None):
    rclpy.init(args=args)
    odom_based_make_path = makePath()
    rclpy.spin(odom_based_make_path)
    odom_based_make_path.f.close()
    odom_based_make_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()