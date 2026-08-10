#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR码解码节点
功能: 订阅摄像头图像, 检测并解码QR码,
      将结果发布到 /qr_decode_result 话题

QR码格式(按赛规): "word1,word2,left" 或 "word1,word2,right"
  示例: "man,apple,left"
  规格: 版本1, 纠错等级M(15%), 大小200mm×200mm

依赖: pip install opencv-contrib-python pyzbar (可选)
"""

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class QRDecoder:
    def __init__(self):
        self.bridge = CvBridge()

        # 发布器: QR解码结果
        self.qr_pub = rospy.Publisher(
            '/qr_decode_result', String, queue_size=10
        )

        # 订阅器: 摄像头图像
        self.image_sub = rospy.Subscriber(
            '/iris_0/camera_1/camera/image_raw_down', Image, self.image_callback, queue_size=2
        )

        # 使用OpenCV的QRCodeDetector (OpenCV 4.5+)
        self.qr_detector = cv2.QRCodeDetector()

        # 防重复发布: 同一QR码内容不重复发送
        self.last_qr_text = ""

        # 可选调试图像发布
        self.debug_pub = rospy.Publisher(
            '/qr_detect_debug', Image, queue_size=2
        )

        rospy.loginfo("QRDecoder 初始化完成")
        rospy.loginfo("  话题: /qr_decode_result")

    def image_callback(self, msg):
        """接收图像, 检测QR码, 发布解码结果"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {e}")
            return

        # OpenCV QR检测
        data, points, _ = self.qr_detector.detectAndDecode(cv_image)

        if data and data != self.last_qr_text:
            rospy.loginfo(f"QR码检测到: '{data}'")

            # 验证格式: 应为3个逗号分隔的单词
            parts = [p.strip() for p in data.split(',')]
            if len(parts) >= 3:
                self.last_qr_text = data
                self.qr_pub.publish(String(data=data))
                rospy.loginfo(f"QR解码成功, 已发布: {parts}")
            else:
                rospy.logwarn(f"QR码格式不匹配(期望3个单词): '{data}'")

            # 发布调试图像
            if points is not None and len(points) > 0:
                pts = points.astype(np.int32)
                for i in range(len(pts)):
                    cv2.line(cv_image,
                             tuple(pts[i][0]), tuple(pts[(i+1) % len(pts)][0]),
                             (0, 255, 0), 2)
                cv2.putText(cv_image, data, tuple(pts[0][0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('qr_decoder', anonymous=True)
    decoder = QRDecoder()
    rospy.spin()
