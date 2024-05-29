import rclpy, time,threading
from rclpy.node import Node
#from geometry_msgs.msg import Twist  # 노드에서 사용할 메시지 타입 import
from ssafy_msgs.msg import HandControl,TurtlebotStatus  # 노드에서 사용할 메시지 타입 import

x=[False,False,False] 
lock=threading.Lock()
class Carry(Node):
    
    # create_publisher(메시지 타입, 토픽, 큐 사이즈)
    # create_subscriber(메시지 타입, 토픽, 콜백 함수, 큐 사이즈)
    def timer_callback(self):
        while True:
            print(' 1: 프리뷰 / 2: 들기 / 3: 놓기\n')
            a=int(input())
            self.cmd_msg=HandControl() 
            self.cmd_msg.control_mode=int(a)
            
            cnt=0
            if(a==1):
                while not bool(x[1]):
                    #print(x[1])
                    self.cmd_msg.put_distance=0.5
                    self.cmd_msg.put_height=1.5
                    self.cmd_publisher.publish(self.cmd_msg)
                    #print(cnt)
                    cnt+=1
                    time.sleep(0.01)
            elif(a==2):
                while not bool(x[2]):
                    #print(x[2])
                    self.cmd_publisher.publish(self.cmd_msg)    
                    #print(cnt)
                    cnt+=1
                    time.sleep(0.01)
            elif(a==3):
                while bool(x[2]):
                    #print(x[2])
                    self.cmd_msg.put_distance=0.5
                    self.cmd_msg.put_height=1.5
                    self.cmd_publisher.publish(self.cmd_msg)
                    #print(cnt)
                    cnt+=1
                    time.sleep(0.01)
                    
            # 메시지 전송
            print("publish complete\n")
        
    def __init__(self):
        super().__init__('Carry')
        # 제어 메시지에 맞는 타입, 토픽을 publish 해야 하기 때문에 publisher 생성
        self.cmd_publisher = self.create_publisher(HandControl, 'hand_control', 10)
        # 상태 메시지에 맞는 타입, 토픽을 subscribe 해야 하기 때문에 subscriber 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,100)
        # 0.1초 주기로 timer_callback 함수가 호출되는 타이머 생성
        #self.timer = self.create_timer(10, self.timer_callback)
        self.turtlebot_status_msg=TurtlebotStatus()
        
        self.thread_1=threading.Thread(target=self.subs)
        self.thread_2=threading.Thread(target=self.timer_callback)
        #self.thread_1.run()
        # 필요한 메시지 변수 미리 생성
        #self.cmd_msg=HandControl() 
        
        self.thread_1.start()
        self.thread_2.start()
        
        print("init\n")

    # '/turtlebot_status'메시지가 들어올 때마다 호출되는 함수
    def subs(self):
        while True:
            #self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,100)
            lock.acquire()
            print(x[0],x[1],x[2])
            lock.release()
            time.sleep(2)
    def status_callback(self, msg):
        #print("status_callback\n")
        lock.acquire()
        x[0]=msg.can_lift
        x[1]=msg.can_put
        x[2]=msg.can_use_hand
        #print('{0}, {1}, {2}\n'.format(msg.can_lift,msg.can_put,msg.can_use_hand))
        lock.release()
        
        #print('lift {0}, put : {1} use_hand : {2} \n'.format(msg.can_lift,msg.can_put,msg.can_use_hand))
        
    
    

def main(args=None):
    rclpy.init(args=args)
    com = Carry()
    rclpy.spin(com)
    com.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__ == '__main__':
    main()
