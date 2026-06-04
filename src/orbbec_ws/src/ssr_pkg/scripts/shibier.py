#!/usr/bin/env python3
#coding=utf-8

import rospy
import cv2
import numpy as np
import os
import subprocess
from std_msgs.msg import String, Float32MultiArray, Int8, Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageDraw, ImageFont
from collections import deque

# 全局变量
target_color = None
target_color_name = None
task1_done = False  # 标志任务1是否已执行
latest_frame = None
bridge = CvBridge()
pixel_to_cm = None  # 每像素对应的实际距离（cm）
task1_status_pub = None
task2_offset_pub = None       # 任务2偏差发布器
task2_waypoints_pub = None    # 任务2圆心航点发布器
task2_deviation_pub = None    # 任务2偏差合格标志发布器 (0/1)
task2_timer = None            # 任务2定时器
latest_task2_waypoints = None
current_position = None
current_yaw = 0.0
position_topic = '/mavros/local_position/pose'  # 无人机位置话题
position_tolerance = 0.5  # 米
block_position_history = deque(maxlen=5)

# 任务2超时控制
task2_start_time = None
task2_timeout_seconds = 20.0




# 任务1、任务2触发坐标
task1_position = [5.0, 4.0, 3.0]
task2_position = [10.0, 10.0, 3.0]




# 任务状态标记
task1_position_reached = False
task2_position_reached = False


# 判断轮廓是否为圆形的函数
def is_circular(contour, threshold=0.7):
    area = cv2.contourArea(contour)
    if area == 0:
        return False
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    return circularity > threshold


def is_position_reached(current, target, tolerance):
    if current is None or target is None or any(v is None for v in target):
        return False
    dx = current[0] - target[0]
    dy = current[1] - target[1]
    dz = current[2] - target[2]
    distance = np.sqrt(dx * dx + dy * dy + dz * dz)
    return distance <= tolerance


def quaternion_to_yaw(q):
    # ROS四元数 -> yaw(弧度)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def body_offset_to_enu(forward_m, right_m, yaw):
    """
    机体系偏移 -> ENU偏移
    forward_m: 前向为正
    right_m: 右向为正
    yaw: 机头朝向相对ENU X轴
    """
    d_enu_x = forward_m * np.cos(yaw) + right_m * np.sin(yaw)
    d_enu_y = forward_m * np.sin(yaw) - right_m * np.cos(yaw)
    return d_enu_x, d_enu_y


# 回调函数，处理无人机位置话题
def position_callback(pose_msg):
    global current_position, current_yaw, task1_done, target_color, target_color_name
    global task1_position_reached, task2_position_reached, task2_timer
    global task2_offset_pub, task2_waypoints_pub, task2_deviation_pub

    current_position = (
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z
    )
    current_yaw = quaternion_to_yaw(pose_msg.pose.orientation)
    # 位置回调仅用于更新当前位置和朝向，任务触发改为监听 planner/goal_reached 话题
    return


def goal_reached_callback(msg):
    """监听 planner/goal_reached，收到 2 则触发任务1，收到 3 则触发任务2。"""
    global task1_done, target_color, target_color_name, task2_timer, task2_position_reached, current_position, task2_start_time

    try:
        code = int(msg.data)
    except Exception:
        rospy.logwarn(f'Invalid planner/goal_reached payload: {msg.data}')
        return

    if code == 2:
        rospy.loginfo('Received goal_reached=2 -> trigger Task1 (identify color)')
        if not task1_done:
            target_color, target_color_name = identify_target_color()
            if target_color is not None:
                rospy.loginfo(f"任务1完成，已保存目标颜色: {target_color_name}")
                task1_done = True
            else:
                rospy.loginfo('任务1未识别到目标色块')
        else:
            rospy.loginfo('Task1 already done; ignoring code 2')

    elif code == 3:
        rospy.loginfo('Received goal_reached=3 -> trigger Task2 (start periodic computation)')
        if not task1_done:
            rospy.logwarn('Task2 trigger received but Task1 not done yet')
            # still allow starting task2 if desired; here we proceed only if task1_done
            return

        if task2_timer is None:
            # ensure we have a current position before starting task2; if not, warn but still set flag
            if current_position is None:
                rospy.logwarn('Current position unknown when triggering Task2; timer will start but computations may wait for position')
            task2_position_reached = True
            task2_timer = rospy.Timer(rospy.Duration(1.5), task2_timer_callback)
            task2_start_time = rospy.Time.now()
            rospy.loginfo('启动任务2定时器: 周期 1.5s, 记录开始时间')
        else:
            rospy.loginfo('Task2 timer already running')

    else:
        rospy.loginfo(f'Received planner/goal_reached={code} (no task trigger)')


def task2_timer_callback(event):
    global current_position, current_yaw, target_color_name, pixel_to_cm, latest_task2_waypoints
    global task2_offset_pub, task2_waypoints_pub, task2_deviation_pub, task2_position_reached
    global task2_start_time, task2_timeout_seconds, task2_timer

    # 超时优先处理：即使未识别到颜色或当前位置未知，也应在超时后发布后备航点
    if task2_start_time is not None and (rospy.Time.now() - task2_start_time).to_sec() >= task2_timeout_seconds:
        rospy.logwarn('任务2超时：%ds 内未完成识别，发送降落指令', int(task2_timeout_seconds))
        try:
            cmd = 'rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"'
            subprocess.call(cmd, shell=True)
            rospy.loginfo('已发送降落指令')
        except Exception as e:
            rospy.logwarn(f'发送降落指令失败: {e}')

        if task2_deviation_pub is not None:
            task2_deviation_pub.publish(Int8(0))

        try:
            if task2_timer is not None:
                task2_timer.shutdown()
        except Exception:
            pass
        task2_timer = None
        task2_start_time = None
        return

    if not task2_position_reached:
        return

    positions = identify_blocks_and_calculate_positions(target_color_name)
    if not positions:
        # 如果在 task2 启动后超过超时时间仍未检测到，则发布后备最终航点
        if task2_start_time is not None and (rospy.Time.now() - task2_start_time).to_sec() >= task2_timeout_seconds:
            rospy.logwarn('任务2超时：%ds 内未检测到目标色块，发送降落指令', int(task2_timeout_seconds))
            try:
                cmd = 'rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"'
                subprocess.call(cmd, shell=True)
                rospy.loginfo('已发送降落指令')
            except Exception as e:
                rospy.logwarn(f'发送降落指令失败: {e}')

            if task2_deviation_pub is not None:
                task2_deviation_pub.publish(Int8(0))

            # 停止定时器，避免重复发布
            try:
                if task2_timer is not None:
                    task2_timer.shutdown()
            except Exception:
                pass
            task2_timer = None
            # 清理开始时间，表明已处理超时
            task2_start_time = None
            return

        rospy.logwarn('任务2定时计算：未检测到目标色块，暂不发布最终航点')
        if task2_deviation_pub is not None:
            task2_deviation_pub.publish(Int8(0))
        return

    uav_x, uav_y, uav_z = current_position
    yaw = current_yaw

    offset_msg = Float32MultiArray()
    offset_data = []
    wp_msg = PoseArray()
    wp_msg.header.stamp = rospy.Time.now()
    wp_msg.header.frame_id = 'world'

    for i, (pos, distance_pixel, distance_real) in enumerate(positions):
        dx_cm = pos[0] * pixel_to_cm if pixel_to_cm else 0.0
        dy_cm = -pos[1] * pixel_to_cm if pixel_to_cm else 0.0
        rospy.loginfo(f'定时任务2: {target_color_name}色块{i+1} 需要向右移动 {dx_cm:.2f} cm, 向前移动 {dy_cm:.2f} cm')
        offset_data.extend([float(dx_cm), float(dy_cm)])

        right_m = dx_cm / 100.0
        forward_m = dy_cm / 100.0
        d_enu_x, d_enu_y = body_offset_to_enu(forward_m, right_m, yaw)

        p = Pose()
        p.position.x = uav_x + d_enu_x
        p.position.y = uav_y + d_enu_y
        p.position.z = uav_z
        p.orientation.w = 1.0
        wp_msg.poses.append(p)

    offset_msg.data = offset_data
    latest_task2_waypoints = wp_msg

    if task2_offset_pub is not None:
        task2_offset_pub.publish(offset_msg)

    if task2_waypoints_pub is not None and len(wp_msg.poses) > 0:
        task2_waypoints_pub.publish(wp_msg)

    within_threshold = True
    deviation_threshold_cm = 3.0
    for p in wp_msg.poses:
        dx = abs((current_position[0] - p.position.x) * 100.0)
        dy = abs((current_position[1] - p.position.y) * 100.0)
        dz = abs((current_position[2] - p.position.z) * 100.0)
        if dx > deviation_threshold_cm or dy > deviation_threshold_cm or dz > deviation_threshold_cm:
            within_threshold = False
            break

    if task2_deviation_pub is not None:
        task2_deviation_pub.publish(Int8(1 if within_threshold else 0))


# 任务1：识别目标色块并保存颜色
def identify_target_color():
    global pixel_to_cm, task1_status_pub

    # 定义三原色的HSV范围，并放宽检测条件
    color_ranges = {
        'red': [
            (np.array([0, 80, 0]), np.array([15, 255, 255])),
            (np.array([170, 80, 80]), np.array([180, 255, 255]))
        ],
        'green': [
            (np.array([36, 50, 0]), np.array([85, 255, 255]))
        ],
        'blue': [
            (np.array([90, 50, 0]), np.array([130, 255, 255]))
        ]
    }

    stable_color_name = None
    stable_detected_color = None
    stable_diameter = None
    stable_start_time = None
    duration = rospy.Duration(3)  # 3秒稳定时间
    task1_wait_published = False
    overall_start_time = rospy.Time.now()

    rospy.loginfo("开始识别目标色块，需要保持3秒稳定...")

    while not rospy.is_shutdown():
        try:
            image_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=1)
            cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            current_max_area = 0
            current_color_name = None
            current_detected_color = None
            current_diameter = None

            for color, ranges in color_ranges.items():
                mask = None
                for lower, upper in ranges:
                    sub_mask = cv2.inRange(hsv, lower, upper)
                    mask = sub_mask if mask is None else cv2.bitwise_or(mask, sub_mask)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)
                    if is_circular(largest_contour) and area > current_max_area:
                        current_max_area = area
                        current_color_name = color
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        roi = hsv[y:y+h, x:x+w]
                        current_detected_color = np.mean(roi, axis=(0, 1)).tolist()
                        (center, radius) = cv2.minEnclosingCircle(largest_contour)
                        current_diameter = radius * 2

            if current_color_name is not None:
                if stable_color_name == current_color_name and stable_start_time is not None:
                    if rospy.Time.now() - stable_start_time >= duration:
                        stable_detected_color = current_detected_color
                        stable_diameter = current_diameter
                        pixel_to_cm = 40 / stable_diameter if stable_diameter and stable_diameter > 0 else None
                        if task1_status_pub is not None:
                            task1_status_pub.publish('task1_done')
                        rospy.loginfo(f"任务1完成，识别到颜色: {current_color_name}")
                        return stable_detected_color, current_color_name
                else:
                    stable_color_name = current_color_name
                    stable_detected_color = current_detected_color
                    stable_diameter = current_diameter
                    stable_start_time = rospy.Time.now()
                    if task1_status_pub is not None and not task1_wait_published:
                        task1_status_pub.publish('task1_wait')
                        task1_wait_published = True
                    rospy.loginfo(f"检测到颜色: {stable_color_name}，开始计时...")
            else:
                stable_color_name = None
                stable_detected_color = None
                stable_diameter = None
                stable_start_time = None

            # 如果总体等待超过15秒仍未识别成功，默认保存为红色并返回
            if rospy.Time.now() - overall_start_time >= rospy.Duration(15):
                # 选择一个代表性的HSV值作为红色的默认值
                default_red_hsv = [0, 128, 128]
                pixel_to_cm = None
                if task1_status_pub is not None:
                    task1_status_pub.publish('task1_done')
                return default_red_hsv, 'red'

            del cv_image, hsv, mask, contours
            rospy.sleep(0.1)

        except rospy.ROSException as e:
            rospy.logwarn(f"获取图像失败: {e}")
            stable_color_name = None
            stable_detected_color = None
            stable_diameter = None
            stable_start_time = None
            pixel_to_cm = None
            rospy.sleep(0.5)

    rospy.loginfo("任务1未完成，节点关闭或中断")
    return None, None


# 任务2：根据保存的目标颜色识别三个色块并输出位置
def identify_blocks_and_calculate_positions(target_color_name):
    global pixel_to_cm, block_position_history

    if not target_color_name:
        rospy.logwarn("任务2: target_color_name 为空，无法检测")
        return []

    ranges_map = get_color_ranges()
    if target_color_name not in ranges_map:
        rospy.logwarn(f"任务2: 无效颜色名 {target_color_name}")
        return []

    try:
        image_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=2)
    except rospy.ROSException as e:
        rospy.logwarn(f"任务2: 获取图像失败: {e}")
        return []

    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    height, width = cv_image.shape[:2]
    center = (width // 2, height // 2)

    # 1) 按颜色名使用固定HSV范围（红色已在 get_color_ranges 里支持双区间环绕）
    color_ranges = ranges_map[target_color_name]
    mask = None
    for lower, upper in color_ranges:
        sub_mask = cv2.inRange(hsv, lower, upper)
        mask = sub_mask if mask is None else cv2.bitwise_or(mask, sub_mask)

    # 2) 形态学开闭运算，先去噪再补洞
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 3) 轮廓筛选
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    current_positions = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 200:
            continue
        if not is_circular(contour):
            continue

        M = cv2.moments(contour)
        if M['m00'] <= 0:
            continue

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        relative_pos = (cx - center[0], cy - center[1])  # (右+, 下+)
        current_positions.append(relative_pos)

        if len(current_positions) >= 3:
            break

    # 按横向位置排序，尽量让同一序号对应同一目标，便于平滑
    current_positions.sort(key=lambda p: p[0])

    # 没检测到就返回空，并清理历史，避免旧值残留
    if not current_positions:
        block_position_history.clear()
        del cv_image, hsv, mask, contours
        return []

    # 4) 短窗口中值平滑（减少抖动）
    block_position_history.append(current_positions)

    smoothed_positions = []
    for i in range(len(current_positions)):
        xs, ys = [], []
        for hist in block_position_history:
            if len(hist) > i:
                xs.append(hist[i][0])
                ys.append(hist[i][1])

        if xs and ys:
            sx = int(np.median(xs))
            sy = int(np.median(ys))
            smoothed_positions.append((sx, sy))
        else:
            smoothed_positions.append(current_positions[i])

    # 5) 输出格式保持不变: (relative_pos, distance_pixel, distance_real)
    positions = []
    for relative_pos in smoothed_positions:
        distance_pixel = float(np.hypot(relative_pos[0], relative_pos[1]))
        distance_real = distance_pixel * pixel_to_cm if pixel_to_cm else 0.0
        positions.append((relative_pos, distance_pixel, distance_real))

    del cv_image, hsv, mask, contours
    return positions


# 获取基本颜色范围
def get_color_ranges():
    return {
        'red': [
            (np.array([0, 80, 80]), np.array([15, 255, 255])),
            (np.array([170, 80, 80]), np.array([180, 255, 255]))
        ],
        'green': [
            (np.array([30, 80, 80]), np.array([90, 255, 255]))
        ],
        'blue': [
            (np.array([90, 80, 80]), np.array([150, 255, 255]))
        ]
    }


# 订阅图像回调
def image_callback(image_msg):
    global latest_frame
    try:
        latest_frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logwarn(f"图像转换失败: {e}")


# 在窗口中绘制检测框
def draw_detection_boxes(frame):
    if frame is None:
        return frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)

    # 绘制圆形等用cv2
    if target_color is not None:
        tolerance = 20
        lower = np.array([
            max(0, target_color[0] - tolerance),
            max(0, target_color[1] - tolerance),
            max(0, target_color[2] - tolerance)
        ])
        upper = np.array([
            min(180, target_color[0] + tolerance),
            min(255, target_color[1] + tolerance),
            min(255, target_color[2] + tolerance)
        ])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if is_circular(contour):
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                center_pt = (int(cx), int(cy))
                radius = int(radius)
                cv2.circle(frame, center_pt, radius, (0, 255, 255), 2)
    else:
        for color_name, ranges in get_color_ranges().items():
            mask = None
            for lower, upper in ranges:
                sub_mask = cv2.inRange(hsv, lower, upper)
                mask = sub_mask if mask is None else cv2.bitwise_or(mask, sub_mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if is_circular(contour) and cv2.contourArea(contour) > 300:
                    (cx, cy), radius = cv2.minEnclosingCircle(contour)
                    center_pt = (int(cx), int(cy))
                    radius = int(radius)
                    color = (0, 0, 255) if color_name == 'red' else (0, 255, 0) if color_name == 'green' else (255, 0, 0)
                    cv2.circle(frame, center_pt, radius, color, 2)

    cv2.circle(frame, center, 4, (255, 255, 255), -1)

    # 转换为PIL绘制文本
    pil_img = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    except:
        font = ImageFont.load_default()

    def to_rgb(bgr):
        return (bgr[2], bgr[1], bgr[0])

    if target_color is not None:
        for contour in contours:
            if is_circular(contour):
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                center_pt = (int(cx), int(cy))
                radius = int(radius)
                draw.text(
                    (center_pt[0] - radius, center_pt[1] - radius - 30),
                    f'Target: {target_color_name}',
                    fill=to_rgb((0, 255, 255)),
                    font=font
                )
        draw.text((10, 10), 'Target color saved, tracking target', fill=to_rgb((0, 255, 255)), font=font)
    else:
        for color_name, ranges in get_color_ranges().items():
            mask = None
            for lower, upper in ranges:
                sub_mask = cv2.inRange(hsv, lower, upper)
                mask = sub_mask if mask is None else cv2.bitwise_or(mask, sub_mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if is_circular(contour) and cv2.contourArea(contour) > 300:
                    (cx, cy), radius = cv2.minEnclosingCircle(contour)
                    center_pt = (int(cx), int(cy))
                    radius = int(radius)
                    color = (0, 0, 255) if color_name == 'red' else (0, 255, 0) if color_name == 'green' else (255, 0, 0)
                    draw.text(
                        (center_pt[0] - radius, center_pt[1] - radius - 30),
                        f'{color_name}',
                        fill=to_rgb(color),
                        font=font
                    )
        draw.text((10, 10), 'Target color not saved, send task1 command to identify', fill=to_rgb((255, 255, 255)), font=font)

    draw.text((10, height - 30), f'Center: {center}', fill=to_rgb((255, 255, 255)), font=font)

    # 转回OpenCV格式
    frame = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    return frame


if __name__ == '__main__':
    rospy.init_node('color_detection_node')

    position_topic = rospy.get_param('~position_topic', position_topic)
    task1_position = rospy.get_param('~task1_position', task1_position)
    task2_position = rospy.get_param('~task2_position', task2_position)
    position_tolerance = rospy.get_param('~position_tolerance', position_tolerance)

    task1_status_pub = rospy.Publisher('/task1_status', String, queue_size=1)
    task2_offset_pub = rospy.Publisher('/task2_position_offset', Float32MultiArray, queue_size=10)
    task2_waypoints_pub = rospy.Publisher('/task2_center_waypoints', PoseArray, queue_size=10)
    task2_deviation_pub = rospy.Publisher('/task2_deviation_flag', Int8, queue_size=1)

    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber(position_topic, PoseStamped, position_callback)
    rospy.Subscriber('/drone_0_ego_planner_node/planner/goal_reached', Int32, goal_reached_callback)

    headless = os.environ.get('DISPLAY', '') == ''
    if not headless:
        cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
    else:
        rospy.loginfo('Headless mode detected: GUI disabled')
    rate = rospy.Rate(30)
    rospy.loginfo(
        f"颜色检测节点已启动，位置话题: {position_topic}，任务1坐标: {task1_position}，"
        f"任务2坐标: {task2_position}，偏差发布: /task2_position_offset，航点发布: /task2_center_waypoints"
    )

    while not rospy.is_shutdown():
        if latest_frame is not None:
            display_frame = latest_frame.copy()
            display_frame = draw_detection_boxes(display_frame)
            if not headless:
                cv2.imshow('Camera View', display_frame)

        if not headless:
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                rospy.loginfo('检测窗口已关闭')
                break
        rate.sleep()

    if not headless:
        cv2.destroyAllWindows()