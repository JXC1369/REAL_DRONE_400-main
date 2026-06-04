#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def publish_fake_drone_position():
    rospy.init_node('fake_drone_position_publisher', anonymous=True)
    pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'map'

    # 可修改的模拟位置序列
    path = [
        (1.0, 1.0, 1.0),
        (3.0, 3.0, 3.0),
        (5.0, 4.0, 3.0),  # 任务1触发点
        (7.0, 4.0, 3.0),
        (7.0, 5.0, 3.0),
        (7.0, 7.0, 3.0),
        (10.0, 10.0, 3.0),  # 任务2触发点
    ]
    idx = 0

    task1_point = (5.0, 4.0, 3.0)
    task2_point = (10.0, 10.0, 3.0)

    task1_hold = False
    task1_delay_triggered = False
    task1_finished = False
    reached_task2 = False
    task1_logged = False
    task2_logged = False

    def task1_status_callback(msg):
        nonlocal task1_delay_triggered, task1_finished, task1_hold
        if msg.data == 'task1_wait' and not task1_delay_triggered:
            rospy.loginfo('收到 task1_wait，延时 3 秒')
            rospy.sleep(3.0)
            task1_delay_triggered = True
        elif msg.data == 'task1_done':
            rospy.loginfo('收到 task1_done，继续发布位置')
            task1_finished = True
            task1_hold = False

    rospy.Subscriber('/task1_status', String, task1_status_callback)
    rospy.loginfo('Fake drone position publisher started on /mavros/local_position/pose')

    while not rospy.is_shutdown():
        x, y, z = path[idx]
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        pub.publish(pose_msg)
        rospy.loginfo_throttle(5, f'Publishing fake drone position: x={x:.1f}, y={y:.1f}, z={z:.1f}')

        if (x, y, z) == task1_point and not task1_finished:
            if not task1_logged:
                rospy.loginfo(f'到达任务1触发点 ({x:.1f}, {y:.1f}, {z:.1f})，等待 shibie 任务1 指令')
                task1_logged = True
            task1_hold = True
        elif (x, y, z) == task2_point:
            if not task2_logged:
                rospy.loginfo(f'到达任务2触发点 ({x:.1f}, {y:.1f}, {z:.1f})，保持当前位置不再改变')
                task2_logged = True
            reached_task2 = True

        if not task1_hold and not reached_task2:
            idx = (idx + 1) % len(path)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_fake_drone_position()
    except rospy.ROSInterruptException:
        pass
