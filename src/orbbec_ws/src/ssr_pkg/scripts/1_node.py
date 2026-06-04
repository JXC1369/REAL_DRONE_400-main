#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    # 初始化节点
    rospy.init_node('talker', anonymous=True)
    
    # 创建发布者，话题名为"1"，消息类型为String
    pub = rospy.Publisher('1_topic', String, queue_size=10)
    
    # 设置循环频率
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        # 准备消息内容
        message = "1"
        
        # 发布消息
        pub.publish(message)
        
        # 打印日志
        rospy.loginfo("Publishing: %s", message)
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass