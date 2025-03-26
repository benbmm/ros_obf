import rclpy
from rclpy.node import Node
from interfaces.msg import Output    # 匯入自定義的消息類型 Output
import sys
import select
import time

class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)  # ROS2 节点初始化
        self.publisher_ = self.create_publisher(Output, "output", 10)  # 創建Publisher發布者
        self.timer_period = 0.1  # 每0.1秒發布一次
        self.step = 1
        self.l_adj = 1.0
        self.r_adj = 1.0
        self.last_input = None
        self.timer = self.create_timer(self.timer_period, self.timer_callback)  # 定時器

    def timer_callback(self):
        msg = Output()
        msg.step = self.step
        msg.l_adj = self.l_adj
        msg.r_adj = self.r_adj

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: step=%d, l_adj=%.2f, r_adj=%.2f' % (msg.step, msg.l_adj, msg.r_adj))
        self.step += 1

    def update_control_parameters(self, input_key):
        if input_key == 'W':
            self.l_adj = 1.0
            self.r_adj = 1.0
        elif input_key == 'S':
            self.l_adj = 0.0
            self.r_adj = 0.0
        elif input_key == 'A':
            self.l_adj = 0.5
            self.r_adj = 1.0
        elif input_key == 'D':
            self.l_adj = 1.0
            self.r_adj = 0.5
        else:
            return  # 保持當前設定
        self.get_logger().info(f"Key '{input_key}' pressed: l_adj={self.l_adj}, r_adj={self.r_adj}")

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("keyboard_control_publisher")

    print("Control the robot using keys:")
    print("W - Forward")
    print("S - Stop")
    print("A - Left")
    print("D - Right")
    print("Press 'Q' to exit")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:  # 檢查是否有鍵盤輸入
                input_key = sys.stdin.read(1).upper()  # 讀取按鍵並轉為大寫
                if input_key == 'Q':
                    print("Exiting...")
                    break
                node.update_control_parameters(input_key)

            time.sleep(0.1)  # 每 0.1 秒檢查一次

    except KeyboardInterrupt:
        print("Keyboard Interrupt. Exiting...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
