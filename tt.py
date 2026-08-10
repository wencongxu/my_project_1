#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
import sys
import traceback
from std_msgs.msg import String
from action_t import Action_t
from std_msgs.msg import UInt16MultiArray
import vision_bridge

rospy.loginfo("=== 无人机视觉导航系统启动 ===")
rospy.loginfo(f"Python版本: {sys.version}")

drone = Action_t("sim_nolog")
rospy.loginfo("无人机控制对象初始化完成")

vision_bridge.init_vision_subscribers()

# ================================================================
RING_CENTER_Z    = 1.6
ALIGN_THRESHOLD  = 0.08

flag = 1
QR_STATE = 30
all_item = ['', '', '', '']
qr_result = ['', '', '']
topic_name = "/servo_angles"

pub = rospy.Publisher(topic_name, UInt16MultiArray, queue_size=10)
rospy.sleep(1)
msg = UInt16MultiArray()


def set_servo_angles(angles):
    try:
        msg.data = angles
        pub.publish(msg)
        time.sleep(0.2)
        rospy.loginfo(f"已发布舵机角度: {msg.data}")
        return True
    except ValueError as e:
        rospy.logerr(f"角度设置失败：{e}")


def check_down_action():
    """下降投放 → 回升到1.2m"""
    try:
        cur_x, cur_y, cur_z = drone.get_current_xyz()
        rospy.loginfo(f"投放: ({cur_x:.1f}, {cur_y:.1f}, {cur_z:.2f})")
        drone.send_position_x_y_z_t(cur_x, cur_y, 0.8, 5)
        rospy.loginfo("投放完成")
        drone.send_position_x_y_z_t(cur_x, cur_y, 1.2, 5)
        rospy.loginfo("回升完成: 1.2m")
    except KeyboardInterrupt:
        drone.stop_all_threads()
        time.sleep(0.1)
        drone.land_lock_vz_t(-0.2, 5)
        while not drone.control_complete(): rospy.sleep(0.1)
        drone.lock()
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        drone.stop_all_threads()
        time.sleep(0.1)
        drone.land_lock_vz_t(-0.2, 5)
        while not drone.control_complete(): rospy.sleep(0.1)
        drone.lock()


def fly_through_ring(timeout=35.0):
    """对齐穿越"""
    start_t = time.time()
    FIXED_Y = None
    attempt = 0

    while not rospy.is_shutdown():
        if time.time() - start_t > timeout:
            rospy.logwarn("超时"); return False
        if drone.stop_thread_flag.is_set():
            rospy.logwarn("中断"); return False

        rfound = vision_bridge.ring_found
        rdx = vision_bridge.ring_dx

        if not rfound or abs(rdx) > 5.0:
            rospy.logwarn_throttle(2.0, "圆环丢失")
            cur_x, cur_y, _ = drone.get_current_xyz()
            if FIXED_Y is None: FIXED_Y = cur_y
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(cur_x, FIXED_Y, RING_CENTER_Z, 90, 0.1, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.sleep(0.3)
            continue

        cur_x, cur_y, _ = drone.get_current_xyz()
        if FIXED_Y is None: FIXED_Y = cur_y

        if abs(rdx) < ALIGN_THRESHOLD:
            rospy.loginfo(f"对齐! rdx={rdx:+.3f} cur_x={cur_x:.2f}")
            drone.send_position_x_y_z_t_yaw(cur_x, -2.5, RING_CENTER_Z, 6,-1.57)
            rospy.loginfo("穿越成功!")
            return True

        attempt += 1
        if attempt > 10:
            rospy.logwarn("多次尝试仍未对齐"); return False

        target_x = cur_x - rdx
        target_x = max(4.5, min(7.5, target_x))
        rospy.loginfo(f"对齐: rdx={rdx:+.3f} → X={target_x:.2f}")
        drone.publish_nav_goal_x_y_z_yaw_tol_frame(target_x, FIXED_Y, RING_CENTER_Z, 90, 0.1, 1, use_thread=True)
        while not drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)

    return False


# ================================================================
while not rospy.is_shutdown():
    try:
        if flag == 1 and not getattr(drone, 'unlock_thread_running', False):
            rospy.loginfo("状态1: 解锁"); drone.unlock(use_thread=True); flag = 2

        elif flag == 2 and drone.control_complete():
            rospy.loginfo("状态2: 起飞 1.2m")
            drone.send_position_x_y_z_t(0, 0, 1.2, 5, use_thread=True)
            flag = 3

        elif flag == 3 and drone.control_complete():
            rospy.loginfo("状态3: 飞往二维码位置")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(1.8, 0, 1.2, 90, 0.3, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            flag = QR_STATE

        elif flag == QR_STATE and drone.control_complete():
            rospy.loginfo("状态30: 驻留等待QR码识别")
            drone.send_position_x_y_z_t(1.8, 0, 1.2, 3, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                qr_result = vision_bridge.qr_words
                if qr_result[0] != '' and qr_result[1] != '' and qr_result[2] != '':
                    rospy.loginfo(f"QR识别成功: {qr_result}")
                    all_item[0] = qr_result[0]; all_item[1] = qr_result[1]
                    drone.stop_all_threads()
                    while not drone.control_complete(): rospy.sleep(0.05)
                    flag = 4; break
                rospy.sleep(0.1)
            else:
                rospy.logwarn("QR未识别，重试"); flag = QR_STATE

        elif flag == 4 and drone.control_complete():
            rospy.loginfo("状态4: 图片靶1")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(3.6, -1.6, 1.2, 90, 0.3, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            check_down_action(); flag = 5

        elif flag == 5 and drone.control_complete():
            rospy.loginfo("状态5: 图片靶2")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(1.8, -1.6, 1.2, 90, 0.3, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            check_down_action(); flag = 6

        elif flag == 6 and drone.control_complete():
            rospy.loginfo("状态6: 图片靶3")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(1.8, 1.6, 1.2, 90, 0.3, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            check_down_action(); flag = 7

        elif flag == 7 and drone.control_complete():
            rospy.loginfo("状态7: 图片靶4")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(3.6, 1.6, 1.2, 90, 0.3, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            check_down_action(); flag = 8

        elif flag == 8 and drone.control_complete():
            rospy.loginfo("状态8: 特殊靶")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(6.0, 1.0, 1.2, 90, 0.1, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            check_down_action()
            # 上升到1.6m
            drone.send_position_x_y_z_t(6.0, 1.0, RING_CENTER_Z, 5)
            flag = 9

        elif flag == 9 and drone.control_complete():
            rospy.loginfo("状态9: 穿越圆环")
            # drone.publish_nav_goal_x_y_z_yaw_tol_frame(6.0, 1.0, RING_CENTER_Z, 90, 0.1, 1, use_thread=True)
            # while not drone.control_complete() and not rospy.is_shutdown():
            #     rospy.sleep(0.1)
            success = fly_through_ring(timeout=25.0)
            if success:
                rospy.loginfo("穿越成功!"); flag = 10
            else:
                rospy.logwarn("穿越失败，退回重试"); flag = 9

        elif flag == 10 and drone.control_complete():
            rospy.loginfo("状态10: 定向降落")
            direction = qr_result[2].strip().lower()
            if direction == 'left':    land_x, land_y = 0.0, 1.6
            elif direction == 'right': land_x, land_y = 0.0, -1.6
            else:                      land_x, land_y = 0.0, -1.6
            rospy.loginfo(f"降落点: ({land_x}, {land_y})")
            drone.publish_nav_goal_x_y_z_yaw_tol_frame(land_x, land_y, 1.2, 90, 0.3, 1, use_thread=True)
            while not drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            drone.land_auto()
            flag = 12

        elif flag == 12:
            rospy.loginfo("状态12: 任务完成!"); drone.stop_all_threads(); break

    except KeyboardInterrupt:
        rospy.loginfo("中断"); drone.stop_all_threads(); break
    except Exception as e:
        rospy.logerr(f"错误: {e}\n{traceback.format_exc()}")
        drone.stop_all_threads(); rospy.sleep(1)

    rospy.sleep(0.1)

try: drone.stop_all_threads()
except: pass
rospy.loginfo("程序结束")
