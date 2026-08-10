#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉桥接模块 (vision_bridge)
功能: 订阅QR码和圆环检测的ROS话题，存储为全局变量供tt.py使用

不修改 ros_initialization.py，不修改 action_t.py
所有新增的视觉相关订阅和回调都放在这里
"""

import rospy
from std_msgs.msg import String

# ========== QR码识别全局变量 ==========
# 格式: "man,apple,left" → qr_words = ['man', 'apple', 'left']
qr_words = ['', '', '']
qr_received = False

# ========== 圆环检测全局变量 ==========
# 像素坐标: 圆环中心在640x640图像中的位置
ring_x = -1.0
ring_y = -1.0
ring_radius = -1.0         # 圆环外径半径(像素)
# 实际偏移(米): 利用已知圆环外径1.2m自动换算
#   dx > 0 → 圆环在无人机右侧, 需右移
#   dy > 0 → 圆环在无人机下方, 需下降
ring_dx = 0.0              # 实际水平偏移(米)
ring_dy = 0.0              # 实际垂直偏移(米)
ring_found = False
ring_confidence = 0.0      # 检测置信度(0~1)


# ========== 回调函数 ==========

def _qr_callback(data):
    """
    QR码识别结果回调
    订阅话题: /qr_decode_result
    消息格式: "man,apple,left" (3个逗号分隔的英文单词)
    """
    global qr_words, qr_received
    try:
        raw = data.data.strip()
        if not raw:
            return

        parts = [p.strip() for p in raw.split(',')]
        if len(parts) >= 3:
            qr_words = parts[:3]
            qr_received = True
            rospy.loginfo(
                f"[vision_bridge] QR识别成功: "
                f"类别1='{qr_words[0]}', 类别2='{qr_words[1]}', 方向='{qr_words[2]}'"
            )
        else:
            rospy.logwarn(
                f"[vision_bridge] QR格式不正确(期望3个单词): '{raw}'"
            )
    except Exception as e:
        rospy.logwarn(f"[vision_bridge] QR回调异常: {e}")


def _ring_callback(data):
    """
    圆环检测结果回调
    订阅话题: /ring_detect_info
    消息格式:
      检测到: "circle: x:320 y:240 r:150 dx:0.123 dy:-0.456 c:0.92"
      未检测到: "not_found"
    dx, dy: 利用已知圆环外径1.2m自动换算的实际偏移(米)
    """
    global ring_x, ring_y, ring_radius, ring_dx, ring_dy, ring_found, ring_confidence
    try:
        raw = data.data.strip()

        if not raw or raw == 'not_found':
            ring_x = -1.0
            ring_y = -1.0
            ring_radius = -1.0
            ring_dx = 0.0
            ring_dy = 0.0
            ring_found = False
            ring_confidence = 0.0
            return

        # 解析: "circle: x:320 y:240 r:150 dx:0.123 dy:-0.456 c:0.92"
        x_val, y_val, r_val, dx_val, dy_val, c_val = -1.0, -1.0, -1.0, 0.0, 0.0, 0.0
        parts = raw.split()
        for p in parts:
            try:
                if p.startswith('x:'):
                    x_val = float(p.split(':')[1])
                elif p.startswith('y:'):
                    y_val = float(p.split(':')[1])
                elif p.startswith('r:'):
                    r_val = float(p.split(':')[1])
                elif p.startswith('dx:'):
                    dx_val = float(p.split(':')[1])
                elif p.startswith('dy:'):
                    dy_val = float(p.split(':')[1])
                elif p.startswith('c:'):
                    c_val = float(p.split(':')[1])
            except (IndexError, ValueError):
                pass

        ring_x = x_val
        ring_y = y_val
        ring_radius = r_val
        ring_dx = dx_val
        ring_dy = dy_val
        ring_confidence = c_val if c_val > 0 else 0.85
        ring_found = (x_val > 0 and y_val > 0)

    except Exception as e:
        rospy.logwarn(f"[vision_bridge] 圆环回调异常: {e}")
        ring_x = -1.0
        ring_y = -1.0
        ring_radius = -1.0
        ring_dx = 0.0
        ring_dy = 0.0
        ring_found = False
        ring_confidence = 0.0


# ========== 初始化函数 (在tt.py中调用一次) ==========

def init_vision_subscribers():
    """
    初始化视觉相关的话题订阅
    在tt.py中 drone = Action_t(...) 之后调用本函数
    """
    rospy.Subscriber('/qr_decode_result', String, _qr_callback, queue_size=10)
    rospy.loginfo("[vision_bridge] 已订阅 /qr_decode_result")

    rospy.Subscriber('/ring_detect_info', String, _ring_callback, queue_size=10)
    rospy.loginfo("[vision_bridge] 已订阅 /ring_detect_info")
