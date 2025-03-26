import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Obstacle():
    def __init__(self):
        self.LIDAR_ERR = 0.05
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()

    def get_scan(self):
        msg = rospy.wait_for_message("scan", LaserScan)
        self.scan_filter = []
        for i in range(720):	#將一整圈的雷達切成720段
            if i <= 30 and i > 0:	#欲偵測的方向
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])

    def obstacle(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            self.get_scan()

            if min(self.scan_filter) < 0.1:	#避障距離（公尺）
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self._cmd_pub.publish(self.twist)
                rospy.loginfo('Stop!')
                
                

            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))

            self._cmd_pub.publish(self.twist)

def main():
    rospy.init_node('turtlebot_scan')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

