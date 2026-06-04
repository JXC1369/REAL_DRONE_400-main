#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading

def talker():
    # 初始化节点
    rospy.init_node('interactive_talker', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('topic', String, queue_size=10)
    
    # 设置循环频率
    rate = rospy.Rate(10)  # 10Hz
    
    # 共享变量，用于存储话题内容
    message_content = "初始内容"
    content_lock = threading.Lock()
    
    # 启动一个线程用于监听用户输入
    def input_listener():
        nonlocal message_content
        while not rospy.is_shutdown():
            new_content = input("请输入新的话题内容（输入exit退出）: ")
            if new_content == "exit":
                rospy.signal_shutdown("用户退出")
                break
            with content_lock:
                message_content = new_content
    
    # 启动输入监听线程
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()
    
    while not rospy.is_shutdown():
        # 获取当前的话题内容
        with content_lock:
            current_content = message_content
        
        # 发布消息
        pub.publish(current_content)
        
        # 打印日志
        rospy.loginfo("Publishing: %s", current_content)
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
