# #!/usr/bin/env python3
# """最简圆环检测——霍夫圆找最大最中心的那个"""
# import rospy, cv2, numpy as np
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# class Detector:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.pub = rospy.Publisher('/ring_detect_info', String, queue_size=10)
#         self.dpub = rospy.Publisher('/ring_detect_debug', Image, queue_size=2)
#         # self.sub = rospy.Subscriber('/usb_cam6/image_raw', Image, self.cb, queue_size=2)#real
#         self.sub = rospy.Subscriber('/iris_0/realsense/depth_camera/color/image_raw', Image, self.cb, queue_size=2)#sim
#         rospy.loginfo("RingDetector MINI 启动")
#     def cb(self, msg):
#         img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         h, w = img.shape[:2]
#         cx_img, cy_img = w // 2, h // 2

#         blur = cv2.GaussianBlur(gray, (7, 7), 0)
#         circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,
#                                     dp=1.2, minDist=60, param1=150, param2=45,
#                                     minRadius=100, maxRadius=min(h, w) // 2)

#         dbg = img.copy()
#         cv2.line(dbg, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (200, 200, 200), 1)
#         cv2.line(dbg, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (200, 200, 200), 1)

#         if circles is None:
#             self.pub.publish(String(data="not_found"))
#             self.dpub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
#             return

#         best, best_s = None, 0.0
#         for x, y, r in np.uint16(np.around(circles[0])):
#             x, y, r = int(x), int(y), int(r)
#             if r < 80: continue
#             dist = np.sqrt((x - cx_img)**2 + (y - cy_img)**2)
#             score = float(r) / 150.0 - dist / 400.0
#             if score > best_s:
#                 best_s = score; best = (x, y, r)
#             cv2.circle(dbg, (x, y), r, (255, 0, 0), 1)  # 所有候选圆

#         if best is None:
#             self.pub.publish(String(data="not_found"))
#             self.dpub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
#             return

#         x, y, r = best
#         scale = 1.2 / (2.0 * r)*1.7 if r > 10 else 0
#         dx = (x - cx_img) * scale
#         dy = (y - cy_img) * scale
#         self.pub.publish(String(data=f"circle: x:{x} y:{y} r:{r} dx:{dx:.3f} dy:{dy:.3f} c:{best_s:.2f}"))

#         cv2.circle(dbg, (x, y), r, (0, 255, 0), 3)  # 选中的圆
#         cv2.circle(dbg, (x, y), 4, (0, 0, 255), -1)
#         cv2.line(dbg, (cx_img, cy_img), (x, y), (255, 100, 0), 2)
#         cv2.putText(dbg, f"r={r} dx={dx:.2f}m dy={dy:.2f}m", (10, 20),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
#         self.dpub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
#         rospy.loginfo_throttle(1.0, f"环({x},{y}) r={r} dx={dx:.3f} dy={dy:.3f}")

# if __name__ == '__main__':
#     rospy.init_node('ring_mini')
#     Detector()
#     rospy.spin()



#!/usr/bin/env python3
"""最简圆环检测——霍夫圆 + 指数平滑，减少跳变"""
import rospy, cv2, numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/ring_detect_info', String, queue_size=10)
        self.dpub = rospy.Publisher('/ring_detect_debug', Image, queue_size=2)
        # self.sub = rospy.Subscriber('/usb_cam6/image_raw', Image, self.cb, queue_size=2)#real
        self.sub = rospy.Subscriber('/iris_0/realsense/depth_camera/color/image_raw', Image, self.cb, queue_size=2)#sim
        rospy.loginfo("RingDetector MINI (平滑版) 启动")

        # ====== 平滑状态 ======
        self.smooth_x = None
        self.smooth_y = None
        self.smooth_r = None
        self.alpha = 0.3   # 平滑系数：0~1，越小越平滑但响应慢

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = img.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,
                                    dp=1.0, minDist=60, param1=150, param2=45,
                                    minRadius=100, maxRadius=min(h, w) // 2)

        dbg = img.copy()
        cv2.line(dbg, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (200, 200, 200), 1)
        cv2.line(dbg, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (200, 200, 200), 1)

        if circles is None:
            self.pub.publish(String(data="not_found"))
            self.dpub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
            return

        best, best_s = None, 0.0
        for x, y, r in np.uint16(np.around(circles[0])):
            x, y, r = int(x), int(y), int(r)
            if r < 80: continue
            dist = np.sqrt((x - cx_img)**2 + (y - cy_img)**2)
            score = float(r) / 150.0 - dist / 400.0
            if score > best_s:
                best_s = score; best = (x, y, r)
            cv2.circle(dbg, (x, y), r, (255, 0, 0), 1)  # 所有候选圆

        if best is None:
            self.pub.publish(String(data="not_found"))
            self.dpub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
            return

        # ----- 平滑处理（仅新增部分） -----
        x, y, r = best
        if self.smooth_x is None:  # 第一帧直接采用
            self.smooth_x, self.smooth_y, self.smooth_r = float(x), float(y), float(r)
        else:
            self.smooth_x = self.alpha * x + (1 - self.alpha) * self.smooth_x
            self.smooth_y = self.alpha * y + (1 - self.alpha) * self.smooth_y
            self.smooth_r = self.alpha * r + (1 - self.alpha) * self.smooth_r
        # 使用平滑后的值进行偏移计算
        sx, sy, sr = int(self.smooth_x), int(self.smooth_y), int(self.smooth_r)
        # --------------------------------

        scale = 1.2 / (2.0 * sr) if sr > 10 else 0
        dx = (sx - cx_img) * scale
        dy = (sy - cy_img) * scale
        self.pub.publish(String(data=f"circle: x:{sx} y:{sy} r:{sr} dx:{dx:.3f} dy:{dy:.3f} c:{best_s:.2f}"))

        cv2.circle(dbg, (sx, sy), sr, (0, 255, 0), 3)  # 选中的圆（平滑后）
        cv2.circle(dbg, (sx, sy), 4, (0, 0, 255), -1)
        cv2.line(dbg, (cx_img, cy_img), (sx, sy), (255, 100, 0), 2)
        cv2.putText(dbg, f"r={sr} dx={dx:.2f}m dy={dy:.2f}m", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.dpub.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
        rospy.loginfo_throttle(1.0, f"平滑环({sx},{sy}) r={sr} dx={dx:.3f} dy={dy:.3f}")

if __name__ == '__main__':
    rospy.init_node('ring_mini')
    Detector()
    rospy.spin()



