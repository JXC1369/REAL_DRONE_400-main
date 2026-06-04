#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
waypoint_sequence_sender.py
用于按顺序发布航点给规划器，并根据规划器的反馈（以及外部任务信号）决定是否前进、等待或重试。

功能概览：
- 从参数服务器读取航点序列、话题名和坐标系
- 依次发布每个航点（`PoseStamped`），等待规划器通过 `planner/goal_reached` 反馈确认到达
- 支持两类特殊反馈代码：触发 `task1` 或 `task2`，在外部任务完成信号后再继续
- 处理最终航点由 `/task2_center_waypoints` 发布的情况

参数（可通过 ROS 参数设置）：
- `~goal_topic`：发布目标的 topic，默认 `/move_base_simple/goal`
- `~feedback_topic`：订阅规划器到达反馈的 topic，默认 `/drone_0_ego_planner_node/planner/goal_reached`
- `~frame_id`：目标坐标系，默认 `map`
- `~waypoints`：航点列表，每个元素为字典（包含 x,y,z, wait_time, 可选 feedback_timeout）
- `~final_waypoint_timeout`：等待 `/task2_center_waypoints` 的超时（秒）
- `~final_goal_feedback_timeout`：发布最终航点后等待规划器反馈的超时（秒）
"""

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int32, String, Float32MultiArray


def load_param(name, default):
    """从参数服务器读取参数，如果不存在则返回默认值。

    Args:
        name (str): 参数名（可以是私有参数，如 '~waypoints'）
        default: 参数不存在时的默认值

    Returns:
        参数的值或默认值
    """
    if rospy.has_param(name):
        return rospy.get_param(name)
    return default


class WaypointSequenceSender:
    """按序发布航点并根据反馈推进的节点封装类。

    行为细节：
    - 发布当前航点到 `self.goal_topic`（`PoseStamped`）
    - 订阅规划器的到达反馈 `self.feedback_topic`。
      反馈为 `Int32` 类型：
        1 -> 表示正常确认到达（继续下一个航点）
        2 -> 表示触发 task1，需要等待 `/task1_status` 发布 'task1_done'
        3 -> 表示触发 task2，需要等待 `/task2_position_offset` 消息
    - 在处理完所有预设航点后，等待 `/task2_center_waypoints` 的最终航点，并对其进行同样的发布与等待反馈流程
    """

    def __init__(self):
        # 从参数服务器读取配置
        self.goal_topic = load_param('~goal_topic', '/move_base_simple/goal')
        self.feedback_topic = load_param('~feedback_topic', '/drone_0_ego_planner_node/planner/goal_reached')
        self.frame_id = load_param('~frame_id', 'world')
        self.waypoints = load_param('~waypoints', [
            {'x': 2.89, 'y': 1.13, 'z': 0.4, 'wait_time': 0.0},
            # {'x': -0.17, 'y': 1.47, 'z': 0.4, 'wait_time': 0.0},
            # {'x': -0.19, 'y': 2.69, 'z': 0.4, 'wait_time': 0.0},
            # {'x': 0.52, 'y': 0.99, 'z': 0.4, 'wait_time': 0.0},
            # {'x': 3.79, 'y': 3.02, 'z': 0.4, 'wait_time': 0.0},
            # {'x': 4.63, 'y': 1.16, 'z': 0.4, 'wait_time': 0.0},
        ])

        # 当前处理的航点索引
        self.current_index = 0
        # 标志位：是否收到可继续前进的反馈
        self.feedback_received = False
        # 等待外部任务完成的标志，分别对应 task1 / task2
        self.waiting_for_task1 = False
        self.waiting_for_task2 = False

        rospy.loginfo('WaypointSequenceSender: goal_topic=%s feedback_topic=%s', self.goal_topic, self.feedback_topic)

        # 发布器：将 PoseStamped 发送给规划器
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)

        # 最终航点由其他节点通过 /task2_center_waypoints 发布（PoseArray）
        self.final_waypoint = None
        self.final_waypoint_received = False
        rospy.Subscriber('/task2_center_waypoints', PoseArray, self.task2_center_waypoints_callback)

        # 订阅规划器反馈（Int32）以及用于处理外部任务的 topic
        rospy.Subscriber(self.feedback_topic, Int32, self.feedback_callback)
        rospy.Subscriber('/task1_status', String, self.task1_status_callback)
        rospy.Subscriber('/task2_position_offset', Float32MultiArray, self.task2_offset_callback)

        # 小延迟确保话题注册完成
        rospy.sleep(0.5)

        if not self.waypoints or len(self.waypoints) == 0:
            rospy.logerr('No waypoints provided. Set parameter ~waypoints')
            return

        # 启动主循环（阻塞直到处理完成或节点关闭）
        self.run()

    def feedback_callback(self, msg):
        """处理规划器发来的 goal_reached 反馈。

        约定：
        - payload 为 Int32 且值为 1/2/3 时分别代表不同含义（见类注释）
        - 若 payload 无法解析为整数，则视为通用到达确认
        """
        try:
            code = int(msg.data)
            if code == 1:
                # 普通到达确认：可以前进到下一个航点
                rospy.loginfo('Received planner/goal_reached signal (1)')
                self.feedback_received = True
            elif code == 2:
                # 触发 task1：等待 /task1_status 发布 task1_done
                rospy.loginfo('Received planner/goal_reached signal (2) -> waiting for /task1_status')
                self.waiting_for_task1 = True
                self.waiting_for_task2 = False
                self.feedback_received = False
            elif code == 3:
                # 触发 task2：等待 /task2_position_offset 的消息
                rospy.loginfo('Received planner/goal_reached signal (3) -> waiting for /task2_position_offset')
                self.waiting_for_task2 = True
                self.waiting_for_task1 = False
                self.feedback_received = False
            else:
                # 未知数值：仅打印日志，不改变状态
                rospy.loginfo('Received planner/goal_reached: %s', str(msg.data))
        except Exception:
            # 无法解析为整数时，将其视为普通到达确认（兼容不同消息格式）
            rospy.loginfo('Received planner/goal_reached (unknown payload)')
            self.feedback_received = True

    def task1_status_callback(self, msg):
        """接收 `/task1_status` 的状态消息，等待 'task1_done' 后继续。
        仅在 `self.waiting_for_task1` 为 True 时才处理。
        """
        if not self.waiting_for_task1:
            return
        try:
            data = str(msg.data)
        except Exception:
            data = ''
        # 当外部任务报告完成时允许继续
        if 'task1_done' in data:
            rospy.loginfo('Received /task1_status task1_done -> proceed to next waypoint')
            self.feedback_received = True
            self.waiting_for_task1 = False
        else:
            rospy.loginfo('Received /task1_status: %s (waiting for task1_done)', data)

    def task2_offset_callback(self, msg):
        """接收 `/task2_position_offset` 的消息，收到后视为 task2 完成并继续。
        仅在 `self.waiting_for_task2` 为 True 时才处理。
        """
        if not self.waiting_for_task2:
            return
        rospy.loginfo('Received /task2_position_offset -> proceed to next waypoint')
        self.feedback_received = True
        self.waiting_for_task2 = False

    def task2_center_waypoints_callback(self, msg):
        """回调：接收 `/task2_center_waypoints`，用于最终进近点。

        该 topic 发布一个 PoseArray，节点会取第一个 Pose 作为最终目标（若存在）。
        """
        try:
            if msg.poses and len(msg.poses) > 0:
                # 记录第一个 pose 作为最终航点
                self.final_waypoint = msg.poses[0]
                self.final_waypoint_received = True
                rospy.loginfo('Received /task2_center_waypoints final pose')
        except Exception:
            rospy.logwarn('Error processing /task2_center_waypoints message')

    def publish_waypoint(self, wp):
        """将输入字典形式的航点转换为 `PoseStamped` 并发布到规划器。

        Args:
            wp (dict): 包含键 'x','y','z' 和可选 'wait_time'
        """
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = self.frame_id
        p.pose.position.x = float(wp.get('x', 0.0))
        p.pose.position.y = float(wp.get('y', 0.0))
        p.pose.position.z = float(wp.get('z', 0.0))
        p.pose.orientation.w = 1.0
        self.goal_pub.publish(p)
        rospy.loginfo('Published waypoint %d/%d: x=%.3f y=%.3f z=%.3f wait=%.3f',
                      self.current_index+1, len(self.waypoints), p.pose.position.x, p.pose.position.y, p.pose.position.z, float(wp.get('wait_time', 0.0)))

    def run(self):
        """主循环：逐个发布航点并等待反馈或超时，然后根据结果推进或重试。

        流程：
        - 发布当前航点
        - 等待 `self.feedback_received` 被置位（或超时）
        - 若收到确认，则睡眠 `wait_time`（用于在航点之间停留），然后推进索引
        - 若超时则短暂延时后重发当前航点
        - 在处理完所有预设航点后，等待 `/task2_center_waypoints` 的最终航点，并对其执行相同的发布与等待逻辑
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.current_index < len(self.waypoints):
            wp = self.waypoints[self.current_index]
            wait_time = float(wp.get('wait_time', 0.0))

            # 重置反馈标志并发布航点
            self.feedback_received = False
            self.publish_waypoint(wp)

            # 等待来自规划器的反馈，带超时保护
            start = rospy.Time.now()
            timeout = float(wp.get('feedback_timeout', 300.0))
            rospy.loginfo('Waiting for feedback for waypoint %d (timeout=%.1f s)...', self.current_index+1, timeout)
            while not rospy.is_shutdown() and not self.feedback_received:
                if (rospy.Time.now() - start).to_sec() > timeout:
                    rospy.logwarn('Timeout waiting for planner/goal_reached for waypoint %d', self.current_index+1)
                    break
                rate.sleep()

            if self.feedback_received:
                # 收到确认后可选等待一段时间再继续
                rospy.loginfo('Waypoint %d reached. Sleeping %.3f seconds before next.', self.current_index+1, wait_time)
                if wait_time > 0.0:
                    rospy.sleep(wait_time)
                self.current_index += 1
            else:
                # 超时：短暂等待后重发当前航点
                rospy.logwarn('Re-publishing waypoint %d after short delay', self.current_index+1)
                rospy.sleep(1.0)

        # 所有预设航点处理完毕，等待最终航点（如果有的话）
        if self.current_index >= len(self.waypoints):
            rospy.loginfo('All waypoints processed.')
            rospy.loginfo('Waiting for /task2_center_waypoints to command final approach...')
            wait_start = rospy.Time.now()
            wait_timeout = float(load_param('~final_waypoint_timeout', 300.0))
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and not self.final_waypoint_received:
                if (rospy.Time.now() - wait_start).to_sec() > wait_timeout:
                    rospy.logwarn('Timeout waiting for /task2_center_waypoints')
                    break
                rate.sleep()

            if self.final_waypoint_received and self.final_waypoint is not None:
                p = self.final_waypoint
                rospy.loginfo('Publishing final waypoint from /task2_center_waypoints: x=%.3f y=%.3f z=%.3f', p.position.x, p.position.y, p.position.z)
                ps = PoseStamped()
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = self.frame_id
                ps.pose = p
                self.goal_pub.publish(ps)
                # 等待规划器对最终航点的反馈
                fb_start = rospy.Time.now()
                fb_timeout = float(load_param('~final_goal_feedback_timeout', 300.0))
                self.feedback_received = False
                while not rospy.is_shutdown() and not self.feedback_received:
                    if (rospy.Time.now() - fb_start).to_sec() > fb_timeout:
                        rospy.logwarn('Timeout waiting for planner/goal_reached for final waypoint')
                        break
                    rate.sleep()


if __name__ == '__main__':
    rospy.init_node('waypoint_sequence_sender')
    try:
        WaypointSequenceSender()
    except rospy.ROSInterruptException:
        pass
