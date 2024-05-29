import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # 노드에서 사용할 메시지 타입 import
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus  # 노드에서 사용할 메시지 타입 import

class Communicator(Node):

    # create_publisher(메시지 타입, 토픽, 큐 사이즈)
    # create_subscriber(메시지 타입, 토픽, 콜백 함수, 큐 사이즈)

    def __init__(self):
        super().__init__('Communicator')
        # 제어 메시지에 맞는 타입, 토픽을 publish 해야 하기 때문에 publisher 생성
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # 상태 메시지에 맞는 타입, 토픽을 subscribe 해야 하기 때문에 subscriber 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        # 0.1초 주기로 timer_callback 함수가 호출되는 타이머 생성
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cmd_publisher2 = self.create_publisher(EnviromentStatus, 'envir_status', 10)

        # 필요한 메시지 변수 미리 생성
        self.cmd_msg=Twist() 
        self.cmd_msg2=EnviromentStatus()
        self.turtlebot_status_msg=TurtlebotStatus()

    # '/turtlebot_status'메시지가 들어올 때마다 호출되는 함수
    def status_callback(self, msg):
        print('Linear Velocity {0}, Angular Velocity : {1} Battery : {2}'.format(msg.twist.linear.x,msg.twist.angular.z,msg.battery_percentage))
    
    def timer_callback(self):
        self.cmd_msg.angular.z=1.0
        self.cmd_msg.linear.x = 1.0
        self.cmd_msg2.weather = 'Snowy'
        
        self.cmd_publisher2.publish(self.cmd_msg2)
        # 메시지 전송
        self.cmd_publisher.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    com = Communicator()
    rclpy.spin(com)
    com.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()