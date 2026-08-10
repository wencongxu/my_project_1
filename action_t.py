#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
by:杨锦成
vx:18873833517
time:2025.5.29
file_name:action_t.py
说明:mavros_mask 0b010111000011 //选择需要的功能掩码，需要的给0,不需要的给1,从右往左对应PX PY PZ VX VY VZ AFX AFY AFZ FORCE YAW YAW_RATE
'''

from ros_initialization import init_ros_publishers, init_ros_services, init_ros_subscribers, init_ros_node, yolo_callback, odom_callback, sim_init_ros_publishers, sim_init_ros_services, sim_init_ros_subscribers
from nav_position_pkg import publish_nav_goal_x_y_z_yaw_t_frame, publish_nav_goal_x_y_z_yaw_tol_frame
from ros_initialization import current_position_x, current_position_y, current_position_z
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Path
from mavros_msgs.srv import CommandLongRequest
from pykalman import KalmanFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import rospy
import time
import math
from nav_msgs.msg import Odometry  # 导入 Odometry 消息类型
import tf
from std_msgs.msg import String
import math  # 添加此行
import threading
from std_msgs.msg import Int32, UInt16  # 添加UInt16用于舵机状态回调
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class Action_t:
    """
    类名称: Action_t
    类功能: 用于控制无人机的各种操作，包括解锁、上锁、陀螺仪热初始化和OFFBOARD模式切换
    使用案例:
    - 创建对象: action = Action_t("real")  # 初始化实际模式控制对象
    - 发送位置目标: action.send_position_x_y_z_t(0, 0, 5, 10)  # 在10秒内将无人机移动到(0, 0, 5)的位置
    - 解锁无人机: action.unlock()  # 解锁无人机并进入OFFBOARD模式
    - 上锁无人机: action.lock()  # 上锁无人机
    - 获取版本信息: version = action.version()  # 获取版本信息
    - 降落无人机: action.land_vz_lock_t(-0.4, 10)  # 以-0.4 m/s的速度降落10秒
    """

    def __init__(self, ExecuteType):
        """
        函数名称: __init__
        函数功能: 初始化Action_t对象，根据执行类型设置ROS服务和订阅
        参数:
        - ExecuteType: 执行类型（"cheak"或"real"）
        使用案例:
        - 创建实际模式对象: action = Action_t("real")
        - 创建检查当前页面语法模式对象: action = Action_t("cheak")
        """
        versioninfo = 30.5  # 版本信息
        print("版本信息:" + str(versioninfo))

        self.ActionExecuteType = ExecuteType  # 设置执行类型（仿真或真实）
        self.state = State()  # 初始化状态
        self.last_command = None  # 存储最后一次的控制命令
        self.offboard_mask = False  # OFFBOARD模式标志
        self.current_position_z = 0.0  # 当前高度
        self.current_position_x = 0.0  # 初始化当前位置X
        self.current_position_y = 0.0  # 初始化当前位置Y
        self.current_orientation = None  # 初始化为 None 以确保类型正确
        self.current_yaw = 0.0
        self.overtime = 0
        self.count = 0  # 添加count属性，用于周期性日志输出

        # --------------- 多线程初始化 -----------------#
        self.CONTROL_COMPLETE = False
        self.send_position_thread_running = False
        self.hover_thread_running = False
        self.unlock_thread_running = False
        self.lock_thread_running = False
        self.takeoff_thread_running = False
        self.land_auto_thread_running = False
        self.nav_goal_thread_running = False
        self.control_yaw_thread_running = False
        self.velocity_thread_running = False
        self.land_lock_thread_running = False
        self.control_points_thread_running = False
        self.position_yaw_thread_running = False
        self.circle_thread_running = False
        self.track_velocity_thread_running = False
        self.time_sleep_thread_running = None
        self.tracking_thread_running = False  # 添加tracking_thread_running属性
        self.control_servo_thread_running = False  # 添加舵机控制线程状态标志

        self.send_position_thread = None
        self.hover_thread = None
        self.unlock_thread = None
        self.lock_thread = None
        self.takeoff_thread = None
        self.land_auto_thread = None
        self.nav_goal_thread = None
        self.control_yaw_thread = None
        self.velocity_thread = None
        self.land_lock_thread = None
        self.control_points_thread = None
        self.position_yaw_thread = None
        self.circle_thread = None
        self.track_velocity_thread = None
        self.time_sleep_thread = None
        self.tracking_thread = None  # 添加tracking_thread属性
        self.servo_control_thread = None  # 添加舵机控制线程
        
        self.stop_thread_flag = threading.Event()  # 终止标志
        # ---------------------------------------------#

        if self.ActionExecuteType == "real" or self.ActionExecuteType == "real_nolog":
            # 设置不使用仿真时间
            rospy.set_param('/use_sim_time', False)

            # 初始化ROS节点（若节点尚未初始化）
            init_ros_node('drone_controller')
            print("ROS node initialized.")

            # 初始化发布器与服务
            self.setpoint_pub, self.nav_goal_pub = init_ros_publishers()
            self.arming_service, self.set_mode_service, self.command_service = init_ros_services()

            # 初始化订阅
            init_ros_subscribers(self)
            rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback, callback_args=self, queue_size=10)
            rospy.Subscriber('/ai_detect_info', String, yolo_callback, callback_args=self, queue_size=30)
            
            # 初始化舵机控制发布者和状态相关属性
            self.servo_pub = rospy.Publisher('/control_servo', String, queue_size=10)
            # 舵机状态订阅已移至ros_initialization.py中处理
            self.servo_status_received = False
            self.servo_status = None
            self.last_servo_status = None
            
        elif self.ActionExecuteType == "sim" or self.ActionExecuteType == "sim_nolog":
             # 设置使用仿真时间
            rospy.set_param('/use_sim_time', True)

            # 初始化ROS节点（若节点尚未初始化）
            init_ros_node('drone_controller')
            print("ROS node initialized.")

            # 初始化发布器与服务
            self.setpoint_pub, self.nav_goal_pub = sim_init_ros_publishers()
            self.arming_service, self.set_mode_service, self.command_service = sim_init_ros_services()

            # 初始化订阅
            sim_init_ros_subscribers(self)
            rospy.Subscriber('/iris_0/mavros/local_position/odom', Odometry, odom_callback, callback_args=self, queue_size=10)
            rospy.Subscriber('/ai_detect_info', String, yolo_callback, callback_args=self, queue_size=30)
            
            # 初始化舵机控制发布者和状态相关属性
            self.servo_pub = rospy.Publisher('/control_servo', String, queue_size=10)
            # 舵机状态订阅已移至ros_initialization.py中处理
            self.servo_status_received = False
            self.servo_status = None
            self.last_servo_status = None
            
        elif self.ActionExecuteType == "cheak":
            print("仿真/检查模式: 不使用实际的ROS服务与话题订阅。")
            self.servo_pub = None
            self.servo_status_received = False
            self.last_servo_status = None


    # ----------------- 舵机控制函数 ----------------- #
    def _control_servo_id_angle_t(self, servo_id, angle, duration):
        """
        函数名称: _control_servo_id_angle_t
        函数功能: 控制指定ID的舵机旋转到指定角度，并等待指定时间
        参数:
        - servo_id: 舵机ID (1-6)
        - angle: 目标角度 (0-180)
        - duration: 等待时间（秒）
        返回值:
        - bool: 控制是否成功
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            # 检查参数有效性
            if servo_id < 1 or servo_id > 6:
                self.log_err(f"舵机ID无效，应为1-6，当前值: {servo_id}")
                self.control_complete(True)
                return False
                
            if angle < 0 or angle > 180:
                self.log_err(f"角度无效，应为0-180，当前值: {angle}")
                self.control_complete(True)
                return False
            
            # 格式化命令：舵机号 + 三位角度值（补零）
            command = f"{servo_id}{angle:03d}"
            
            # 重置状态接收标志
            self.servo_status_received = False
            
            # 发布命令
            msg = String()
            msg.data = command
            self.servo_pub.publish(msg)
            
            self.log_info(f"已发送舵机命令: {command} (舵机{servo_id}旋转到{angle}度)")
            
            # 等待状态回复，最多等待1秒
            timeout = rospy.Time.now() + rospy.Duration(1.0)
            while not self.servo_status_received and rospy.Time.now() < timeout:
                if self.stop_thread_flag.is_set():
                    self.log_warn("舵机控制线程被中断")
                    self.control_complete(True)
                    return False
                rospy.sleep(0.01)
                
            if not self.servo_status_received:
                self.log_warn("未收到舵机控制状态回复")
            
            # 等待指定的持续时间
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    self.log_warn("舵机控制线程被中断")
                    break
                rospy.sleep(0.1)
            
            self.control_complete(True)
            return self.servo_status_received and self.last_servo_status == 1
            
        elif self.ActionExecuteType == "cheak":
            self.log_info(f"检查当前页面语法模式: 控制舵机{servo_id}旋转到{angle}度，持续{duration}秒")
            return True
    
    def thread_control_servo_id_angle_t(self, servo_id, angle, duration, use_thread=False):
        """舵机控制线程函数
        
        Args:
            servo_id: 舵机ID
            angle: 目标角度
            duration: 持续时间
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                # 启动线程
                if self.servo_thread_running:
                    rospy.logwarn("舵机控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.servo_thread and self.servo_thread.is_alive():
                        self.servo_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.servo_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.servo_thread = threading.Thread(
                        target=self._control_servo_id_angle_t,
                        args=(servo_id, angle, duration)
                    )
                    self.servo_thread.daemon = True  # 设为守护线程
                    self.servo_thread.start()
                    rospy.loginfo(f"舵机控制线程已启动: ID={servo_id}, 角度={angle}, 持续={duration}s")
                except Exception as e:
                    rospy.logerr(f"启动舵机控制线程失败: {str(e)}")
                    self.servo_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.servo_thread_running:
                    self.servo_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止舵机控制线程...")
                    if self.servo_thread and self.servo_thread.is_alive():
                        self.servo_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("舵机控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的舵机控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.servo_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def control_servo_id_angle_t(self, servo_id, angle, duration, use_thread=False):
        """
        函数名称: control_servo_id_angle_t
        函数功能: 控制指定ID的舵机旋转到指定角度，并等待指定时间
        参数:
        - servo_id: 舵机ID (1-6)
        - angle: 目标角度 (0-180)
        - duration: 等待时间（秒）
        - use_thread: 是否使用线程方式执行（默认为False）
        返回值:
        - bool: 控制是否成功
        使用案例:
        - 控制1号舵机旋转到90度，等待2秒: action.control_servo_id_angle_t(1, 90, 2)
        - 使用线程方式控制: action.control_servo_id_angle_t(1, 90, 2, use_thread=True)
        """
        try:
            if use_thread:
                # 使用线程方式执行
                self.thread_control_servo_id_angle_t(servo_id, angle, duration, use_thread=True)
                if self.control_complete():
                    self.stop_thread_flag.set()
                    self.thread_control_servo_id_angle_t(servo_id, angle, duration, use_thread=False)
                return True
            else:
                # 非线程方式：如果已有线程在运行，先停止它
                if self.servo_control_thread_running:
                    self.stop_thread_flag.set()  # 设置中断标志
                    self.log_warn("强制中断舵机控制线程")
                    self.thread_control_servo_id_angle_t(servo_id, angle, duration, use_thread=False)
                # 直接执行控制函数
                return self._control_servo_id_angle_t(servo_id, angle, duration)
        except Exception as e:
            self.log_err(f"舵机控制过程中发生错误: {str(e)}")
            self.servo_control_thread_running = False
            self.control_complete(True)  # 确保出错时设置完成标志
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    # ----------------- 日志输出封装，用于在 "nolog" 时屏蔽 ----------------- #
    def log_info(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            print(self.ActionExecuteType)
            rospy.loginfo(msg)

    def log_warn(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            rospy.logwarn(msg)

    def log_err(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            rospy.logerr(msg)

    def log_debug(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            rospy.logdebug(msg)
    # --------------------------------------------------------------------- #

    # ----------------- 无人机数据回调 ----------------- #
        
    def control_complete(self, value=None):
        if value is not None:
            self.CONTROL_COMPLETE = value
        return self.CONTROL_COMPLETE
    
    def get_current_xyz(self):
        """
        函数名称: get_current_xyz
        函数功能: 获取无人机当前的X和Y坐标。
        返回值:
        - (x, y, z): 当前的X和Y坐标。
        """
        # 等待回调函数获取位置数据
        rospy.sleep(0.1)  # 等待数据更新
        # 确保数据正确性
        if hasattr(self, 'current_position_x') and hasattr(self, 'current_position_y') and hasattr(self, 'current_position_z'):
            self.log_debug(f"当前坐标: x={self.current_position_x}, y={self.current_position_y}, z={self.current_position_z}")
            return self.current_position_x, self.current_position_y, self.current_position_z
        
    def get_current_yaw(self):
        """
        函数名称: get_current_yaw
        函数功能: 获取无人机当前的 yaw 角度（欧拉角形式）。
        返回值:
        - yaw: 当前 yaw 角度，单位为弧度。
        使用案例:
        - yaw = action.get_current_yaw()  # 获取当前 yaw 角度
        """
        # 等待回调函数获取姿态数据
        rospy.sleep(0.1)  # 等待数据更新

        # 确保 yaw 角度已经被更新
        if hasattr(self, 'current_yaw'):
            self.log_debug(f"当前 yaw 角度: {self.current_yaw} (弧度)")
            return self.current_yaw
        else:
            self.log_warn("当前未获取到 yaw 角度，默认返回 yaw=0.0")
            return 0.0
    
    def get_current_yolo_xy(self):
        """
        函数名称: get_current_yolo_xy
        函数功能: 获取当前目标的X和Y坐标
        返回值:
        - (yolo_x, yolo_y): 目标的X和Y坐标，未检测到目标时返回(-1.0, -1.0)
        """
        # 检查属性是否存在，不存在则初始化为-1
        if not hasattr(self, 'yolo_x'):
            self.yolo_x = -1.0
        if not hasattr(self, 'yolo_y'):
            self.yolo_y = -1.0
        
        return self.yolo_x, self.yolo_y
    
    def get_current_yolo_class(self):
        """
        获取当前检测到的目标类别。
        返回值:
        - 类别字符串，如 'circle'，未检测到则返回 'none'
        """
        if not hasattr(self, 'yolo_class'):
            self.yolo_class = 'none'
        return self.yolo_class

    def get_yolo_result_width(self):
        """
        函数名称: get_yolo_result_width
        函数功能: 获取当前目标的宽度
        返回值:
        - (width): 目标的宽度，未检测到目标时返回-1.0
        """
        # 检查属性是否存在，不存在则初始化为-1
        if not hasattr(self, 'yolo_result_width'):
            self.yolo_result_width = -1.0
        
        return self.yolo_result_width
    
    def get_yolo_result_height(self):
        """
        函数名称: get_yolo_result_height
        函数功能: 获取当前目标的高度
        返回值:
        - (height): 目标的高度，未检测到目标时返回-1.0
        """
        # 检查属性是否存在，不存在则初始化为-1
        if not hasattr(self, 'yolo_result_height'):
            self.yolo_result_height = -1.0
        
        return self.yolo_result_height

    # ------------------------------------------------- #

    # ----------------- 无人机控制的返回标志集合函数 ----------------- #
    def found_target(self):
        """
        函数名称: found_target
        函数功能: 飞行过程中这个函数会被循环调用，用于检查是否检测到目标。
                  若当前获取的目标坐标 (yolo_x, yolo_y) 都为 -1.0，则表示未检测到目标，返回 False；
                  否则，表示检测到目标，返回 True。
        返回值:
        - bool: 若当前目标坐标为 (-1.0, -1.0) 则返回 False，否则返回 True。
        """
        yolo_x, yolo_y = self.get_current_yolo_xy()
        if yolo_x != -1.0 and yolo_y != -1.0:
            self.log_info(f"检测到目标: x={yolo_x}, y={yolo_y}")
            return True
        return False

    # ---------------------------------------------------------------#
    def _time_sleep(self, duration):
        """
        使用 rospy.Rate 实现类似 time.sleep 的功能。

        参数:
        - duration: 延迟时间，单位秒。
        """
        if duration <= 0:
            return
        
        rate = rospy.Rate(1.0 / duration)  # 设置频率为 1/duration Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.stop_thread_flag.is_set():
                rospy.logwarn("_time_sleep 线程被中断")
                break
            rate.sleep()  # 等待一个周期

    def _send_position_x_y_z_t(self, x, y, z, duration):
        """
        函数名称: send_position_x_y_z_t
        函数功能: 在指定时间内循环发送位置目标
        参数:
        - x: 目标位置X坐标
        - y: 目标位置Y坐标
        - z: 目标位置Z坐标
        - duration: 发送位置目标的时间持续时长（秒）
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)

            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            hold_yaw = self.get_current_yaw()  # 保存当前yaw角度
            hold_x_y_z = self.get_current_xyz()
            target = PositionTarget()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = 0b101111111000  # 设置功能掩码
            target.position.x = x
            target.position.y = y
            target.position.z = z
            target.yaw = hold_yaw
            self.last_command = target        
            start_time = rospy.Time.now()
            rate = rospy.Rate(20)
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                   rospy.logwarn("send_position_x_y_z_t 线程被中断")
                   break
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                # self.log_info(f"Real模式: 发布位置指令到 ({x}, {y}, {z}), Yaw: {hold_yaw}，x_y_z:{hold_x_y_z}")
                rate.sleep()
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式: 发送位置 ({x}, {y}, {z})")

    def _hover_delay_t(self, delay_s):
        """
        函数名称: hover_delay_t
        函数功能: 在指定延迟时间内保持当前位置和旋转角度，持续发布控制指令
        参数:
        - delay_s: 延迟时间，单位秒
        使用案例:
        - 延时10秒，并在此期间保持当前位置和航向角: action.hover_delay_t(10)
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            start_time = rospy.Time.now() + rospy.Duration(delay_s)  # 计算结束时间
            # 在函数开始时读取一次当前位置和yaw角度
            current_x, current_y, current_z = self.get_current_xyz()  # 获取当前X和Y坐标
            current_z = self.current_position_z  # 获取当前高度
            hold_yaw = self.get_current_yaw()  # 获取当前yaw角度

            # 在延时期间持续发布当前位置和当前yaw角度
            while rospy.Time.now() < start_time:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("hover_delay_t 线程被中断")
                    break
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                # 设置功能掩码，仅使用位置和yaw角度控制
                target.type_mask = 0b101111111000  
                target.position.x = current_x  # 使用当前X坐标保持位置
                target.position.y = current_y  # 使用当前Y坐标保持位置
                target.position.z = current_z  # 使用当前高度保持位置
                target.yaw = hold_yaw  # 使用当前yaw角度保持旋转角

                # 发布位置和yaw角度控制指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)

                rate.sleep()  # 保持20Hz的频率
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式: 延时 {delay_s} 秒")

    def _takeoff(self, height):
        """
        函数名称: takeoff
        函数功能: 起飞到指定高度
        参数:
        - height: 起飞高度，单位米
        使用案例:
        - 起飞到1米高度: action.takeoff(1)
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            # 等待无人机解锁
            if not self.state.armed:
                self.log_warn("警告: 无人机未解锁")
                # 尝试解锁无人机
                if not self.state.armed:
                    rospy.logerr("超时: 无人机未能解锁")
                    return
            # 等待进入OFFBOARD模式
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return

            # 获取当前坐标
            current_x, current_y, current_z = self.get_current_xyz()

            # 发送位置指令，起飞到指定高度
            self._send_position_x_y_z_t(current_x, current_y, height, 5)  # 起飞到指定高度

            # 等待无人机达到指定高度
            rate = rospy.Rate(10)  # 设置频率为10Hz
            while not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("takeoff 线程被中断")
                    break
                current_x, current_y, current_z = self.get_current_xyz()
                if current_z >= height - 0.1:  # 容忍0.1米的误差
                    self.log_info(f"无人机已达到指定高度: {height} 米")
                    break
                self.log_info(f"当前高度: {current_z} 米, 目标高度: {height} 米")
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            self.log_info(f"检查当前页面语法模式: 起飞到高度 {height} 米")
            
    def _unlock(self):
        """
        函数名称: unlock
        函数功能: 解锁无人机，并在解锁时执行陀螺仪热初始化和OFFBOARD模式切换
        使用案例:
        - 解锁无人机并准备进入OFFBOARD模式: action.unlock()
        """
        ros_time_limit = 3  # 解锁操作的ROS时间限制，设置为3秒

        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            try:
                # 进行陀螺仪热校准
                command = CommandLongRequest()
                command.broadcast = False
                command.command = 241  # MAV_CMD_PREFLIGHT_CALIBRATION
                command.confirmation = 0
                command.param1 = 1  # Gyro calibration
                command.param2 = 0
                command.param3 = 0
                command.param4 = 0
                command.param5 = 0
                command.param6 = 0
                command.param7 = 0

                # 调用服务
                result = self.command_service(command)
                if result.success:
                    print("陀螺仪热初始化成功")  # 打印成功消息
                else:
                    print("陀螺仪热初始化失败")  # 打印失败消息

                # 等待2秒以确保热初始化过程完成
                rospy.sleep(2)

                # 尝试进入OFFBOARD模式
                target = PositionTarget()  # 创建位置目标消息
                target.type_mask = 0b101111111000  # 设置功能掩码
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.position.x = 0  # 设置X坐标
                target.position.y = 0  # 设置Y坐标
                target.position.z = 0  # 设置Z坐标
                target.yaw = 0  # 设置目标yaw角度
                self.last_command = target  # 存储当前命令

                # 发布目标点以激活OFFBOARD模式
                rate = rospy.Rate(5)  # 频率设置为20Hz
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("unlock 线程被中断")
                        break
                    if self.setpoint_pub:
                        self.setpoint_pub.publish(target)  # 发布最后一次的命令
                    if self.state.mode != "OFFBOARD":
                        if self.set_mode_service:
                            self.set_mode_service(custom_mode="OFFBOARD")  # 设置模式为OFFBOARD
                        rospy.sleep(0.1)  # 等待0.2秒
                    if self.state.mode == "OFFBOARD":
                        print("成功进入OFFBOARD模式")  # 打印成功消息
                        self.offboard_mask = True
                        break
                    rate.sleep()  # 等待以保持频率

                # 解锁无人机
                start_time = rospy.Time.now()  # 获取当前时间
                while (rospy.Time.now() - start_time).to_sec() < ros_time_limit:
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("unlock 线程被中断")
                        break
                    self.arming_service(True)  # 发送解锁指令
                    rospy.sleep(0.5)  # 等待0.5秒以减少时间浪费
                print("解锁完成")  # 打印完成消息
                
                self.control_complete(True)  # 设置控制完成标志

            except rospy.ServiceException as e:
                print("服务调用失败: %s" % e)  # 打印服务调用失败的错误消息
                
                self.control_complete(True)  # 设置控制完成标志

        elif self.ActionExecuteType == "cheak":
            # 在检查当前页面语法模式下，直接打印解锁操作信息
            print("检查当前页面语法模式: 执行解锁操作及OFFBOARD模式切换")
            self.offboard_mask = True  # 检查当前页面语法模式下直接设置为True

    def _lock(self):
        """
        函数名称: lock
        函数功能: 上锁无人机，添加高度检查
        使用案例:
        - 上锁无人机: action.lock()
        - 说明:无人机高度高于0.2米上锁功能无效
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            if self.current_position_z > 0.5:
                print("警告: 无人机高度高于0.6米，上锁失败")
                return
            # 在实际模式下，执行上锁操作
            print("正在上锁无人机...")
            
            # 自动强制上锁，有高空坠落风险
            disarm_cmd_long = CommandLongRequest()
            disarm_cmd_long.broadcast = False
            disarm_cmd_long.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
            disarm_cmd_long.param1 = 0  # Disarm
            disarm_cmd_long.param2 = 21196  # Kill no check landed

            rate = rospy.Rate(1.0)
            while not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("lock 线程被中断")
                    break
                result = self.command_service(disarm_cmd_long)
                if result.success:
                    print("无人机已解除武装 (disarmed)")
                    break
                else:
                    print("解除武装失败，重试中...")
                rate.sleep()
            
            self.control_complete(True)  # 设置控制完成标志
            print("紧急解除武装完成，继续发布设定点。")

        elif self.ActionExecuteType == "cheak":
            # 在检查当前页面语法模式下，直接打印上锁操作信息
            print("检查当前页面语法模式: 上锁操作")

    def _land_auto(self):
        """
        函数名称: land_auto
        函数功能: 自动控制无人机降落，根据当前高度决定何时终止降落并上锁。
        使用案例:
        - 自动降落：action.land_auto()
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return

            rate = rospy.Rate(20)  # 设置频率为20Hz
            hold_yaw = self.get_current_yaw()  # 保存当前yaw角度

            # 步骤1：设置目标高度为0.1米
            setpoint_position = PositionTarget()
            setpoint_position.header.stamp = rospy.Time.now()
            setpoint_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_position.type_mask = 0b010111111000  # 使用位置控制，忽略速度和加速度
            current_x, current_y, _ = self.get_current_xyz()
            setpoint_position.position.x = current_x  # 保持当前x位置
            setpoint_position.position.y = current_y  # 保持当前y位置
            setpoint_position.position.z = 0.1        # 设置目标z位置为0.1米
            setpoint_position.yaw = hold_yaw          # 保持当前yaw角度
            
            # 持续发送位置指令，直到当前高度 <= 0.25米
            while not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("land_auto 线程被中断")
                    break
                current_x, current_y, current_z = self.get_current_xyz()

                # 检查当前高度是否低于0.25米
                if current_z <= 0.25:
                    break

                # 发布位置控制指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(setpoint_position)

                self.log_debug(f"发布位置指令: x={current_x}, y={current_y}, z=0.1, yaw={hold_yaw}")
                rate.sleep()

            # 步骤2：设置目标高度为0米
            setpoint_position.position.z = -1  # 修改目标高度为-1米
            setpoint_position.position.x = current_x  # 保持当前x位置
            setpoint_position.position.y = current_y  # 保持当前y位置
            setpoint_position.yaw = hold_yaw

            start_time = rospy.Time.now()
            duration = 0.4  # 持续0.4秒
            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("land_auto 线程被中断")
                    break
                # 发布位置控制指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(setpoint_position)

                self.log_debug(f"发布位置指令: x={current_x}, y={current_y}, z=-1.0, yaw={hold_yaw}")
                rate.sleep()

            # 步骤3：设置目标高度为1米
            setpoint_position.position.z = 0.02 # 修改目标高度为0.02米
            setpoint_position.position.x = current_x  # 保持当前x位置
            setpoint_position.position.y = current_y  # 保持当前y位置
            setpoint_position.yaw = hold_yaw

            start_time = rospy.Time.now()
            duration = 0.02  # 持续0.02秒
            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("land_auto 线程被中断")
                    break
                # 发布位置控制指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(setpoint_position)

                self.log_debug(f"发布位置指令: x={current_x}, y={current_y}, z=1.0, yaw={hold_yaw}")
                rate.sleep()

            # 步骤4：强制上锁
            self._lock()
            self.log_info("自动降落完成，执行上锁操作")
        
        elif self.ActionExecuteType == "cheak":
            self.log_info("检查当前页面语法模式: 执行自动降落操作")

    def _publish_nav_goal_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame):
        """
        函数名称: publish_nav_goal_x_y_z_yaw_t_frame
        函数功能: 发布导航目标到 'nav_goal' 话题，并在指定时间内以 20Hz 发布 mavros/setpoint_raw/local 消息
        参数:
        - x: 目标位置的X坐标
        - y: 目标位置的Y坐标
        - z: 目标位置的Z坐标
        - yaw: 目标航向角（弧度）
        - 偏航角说明(无人机到达容差范围内，也就是到达目标点后开始旋转):
        - 正值: 顺时针旋转，角度范围为 0 到 360 度。
        - 负值: 逆时针旋转，角度范围为 0 到 -360 度。
        - t: 持续时间（秒）
        - frame: 坐标系（1表示'map'，2表示'base_link'）
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                if "nolog" not in self.ActionExecuteType:
                    self.log_info(f"Publishing nav goal: x={x}, y={y}, z={z}, yaw={yaw}, frame={frame}, duration={t}s")

                publish_nav_goal_x_y_z_yaw_t_frame(
                    x, y, z, yaw, t, frame, self.setpoint_pub, self.nav_goal_pub, self.log_info ,self.ActionExecuteType ,stop_thread_flag=self.stop_thread_flag
                )
                
                self.control_complete(True)  # 设置控制完成标志
            elif self.ActionExecuteType == "cheak":
                print(f"检查当前页面语法模式: 发布导航目标到 (x: {x}, y: {y}, z: {z}, yaw: {yaw}, frame: {frame})，持续时间 {t} 秒")
        except Exception as e:
            self.log_err(f"导航过程中发生错误: {str(e)}")
            raise
        finally:
            self.control_complete(True)
            if self.nav_goal_thread_running:
                self.nav_goal_thread_running = False

    def _control_yaw_z_t(self, yaw, z, t):
        """
        函数名称: control_yaw_z_t
        函数功能: 控制无人机在指定高度并在指定时间内进行偏航控制，保持当前位置的X和Y不变。
        
        参数:
        - z: 目标高度
        - yaw: 相对偏航角，范围为 0 到 360 度 或 0 到 -360 度。
        - t: 持续时间（秒）

        偏航角说明:
        - 正值: 顺时针旋转，角度范围为 0 到 360 度。
        - 负值: 逆时针旋转，角度范围为 0 到 -360 度。
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            # 获取当前X、Y坐标和当前yaw角度，只读取一次
            current_x, current_y, current_z = self.get_current_xyz()
            hold_yaw = self.get_current_yaw()  # 获取当前yaw角度

            # 将目标yaw角度转换为弧度并反转正负符号
            relative_yaw = -yaw * (math.pi / 180.0)
            # 获取当前时间
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < t:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("control_yaw_z_t 线程被中断")
                    break
                # 设置目标的yaw和保持当前位置的x, y
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # 使用本地NED坐标系
                target.type_mask = 0b101111111000  # 使用位置控制并控制yaw
                target.position.x = current_x  # 保持当前位置的X坐标
                target.position.y = current_y  # 保持当前位置的Y坐标
                target.position.z = z  # 目标高度
                target.yaw = hold_yaw + relative_yaw  # 使用相对yaw值
                # 发布目标点指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                    self.log_info(f"发布偏航控制: yaw={yaw} 度, 高度={z}, 当前x={current_x}, y={current_y}")
                rate.sleep()  # 保持20Hz的循环频率
            self.log_info("偏航控制完成")
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            # 检查当前页面语法模式下只输出一行日志并打印传入的时间
            print(f"检查当前页面语法模式 - yaw控制: yaw={yaw} 度, 高度={z}, 传入时间={t} 秒")

    def _send_velocity_vx_vy_z_t(self, vx, vy, z, duration):
        """
        函数名称: send_velocity_vx_vy_z_t
        函数功能: 在局部坐标系下以指定的速度控制无人机移动一段时间
        参数:
        - vx: X方向速度
        - vy: Y方向速度
        - z: 目标高度
        - duration: 持续时间（秒）
        使用案例:
        - 控制无人机以速度(vx, vy)在高度z飞行duration秒: action.send_velocity_vx_vy_z_t(1, 1, 5, 10)
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            start_time = rospy.Time.now()
            
            # 保存当前位置和yaw角度，仅在初次进入循环时保存一次
            current_x, current_y,current_z = self.get_current_xyz()  # 获取当前X和Y坐标
            hold_yaw = self.get_current_yaw()  # 保存初始yaw角度

            # 开始控制逻辑
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("send_velocity_vx_vy_z_t 线程被中断")
                    break
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

                # 设置目标速度和位置控制
                if vx == 0 and vy != 0:
                    target.type_mask = 0b101111000000  # 保持X位置
                    target.position.x = current_x
                    target.velocity.x = 0
                    target.velocity.y = vy
                    target.velocity.z = 0
                elif vy == 0 and vx != 0:
                    target.type_mask = 0b101111000000  # 保持Y位置
                    target.position.y = current_y
                    target.velocity.y = 0
                    target.velocity.x = vx
                    target.velocity.z = 0
                elif vx == 0 and vy == 0:
                    target.type_mask = 0b101111000000  # 保持X和Y位置
                    target.position.x = current_x
                    target.velocity.x = 0
                    target.position.y = current_y
                    target.velocity.y = 0
                    target.velocity.z = 0
                else:
                    target.type_mask = 0b101111000011  # 使用速度控制
                    target.velocity.x = vx
                    target.velocity.y = vy
                    target.velocity.z = 0
                
                # 设置高度和yaw角度
                target.position.z = z  # 保持目标高度
                target.yaw = hold_yaw  # 设置初始保存的yaw角度

                # 发布目标指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)

                # ROS 日志打印输出当前状态
                self.log_info(f"Real模式 - 发布速度控制: 速度(vx={vx}, vy={vy}), 位置保持(X={current_x if vx == 0 else 'N/A'}, Y={current_y if vy == 0 else 'N/A'}), 高度={z}, Yaw={hold_yaw}")

                rate.sleep()  # 保持20Hz的循环频率
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式 - 发布目标指令: 速度(vx={vx}, vy={vy}), 位置保持(X={self.current_position_x if vx == 0 else 'N/A'}, Y={self.current_position_y if vy == 0 else 'N/A'}), 高度={z}, Yaw={self.current_yaw}")

    

    
            
    def _land_lock_vz_t(self, vz, duration):
        """
        函数名称: land_vz_t
        函数功能: 控制无人机降落
        参数:
        - vz: 下降速度（米/秒）
        - duration: 降落时间（秒）
        使用案例:
        - 以-0.4 m/s的速度降落10秒: action.land_lock_vz_t(-0.4, 10)
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(20)  # 设置频率为20Hz
                state_start_time = rospy.Time.now()
                while (rospy.Time.now() - state_start_time).to_sec() < duration:
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("land_lock_vz_t 线程被中断")
                        break
                    setpoint_raw = PositionTarget()
                    setpoint_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    setpoint_raw.type_mask = 0b101111000011  # 设置功能掩码
                    setpoint_raw.position.z = 0
                    setpoint_raw.velocity.x = 0
                    setpoint_raw.velocity.y = 0
                    setpoint_raw.velocity.z = vz
                    self.setpoint_pub.publish(setpoint_raw)
                    if (rospy.Time.now() - state_start_time).to_sec() > 3:
                        setpoint_raw.type_mask = 0b101111000011  # 设置功能掩码
                        self.log_info("Landing complete")
                        # 自动强制上锁，有高空坠落风险
                        disarm_cmd_long = CommandLongRequest()
                        disarm_cmd_long.broadcast = False
                        disarm_cmd_long.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
                        disarm_cmd_long.param1 = 0  # Disarm
                        disarm_cmd_long.param2 = 21196  # Kill no check landed
                        rate1 = rospy.Rate(1.0)
                        while not rospy.is_shutdown():
                            if self.stop_thread_flag.is_set():
                                rospy.logwarn("land_lock_vz_t 线程被中断")
                                break
                            result = self.command_service(disarm_cmd_long)
                            if result.success:
                                print("无人机已解除武装 (disarmed)")
                                break
                            else:
                                print("解除武装失败，重试中...")
                            rate1.sleep()
                        print("紧急解除武装完成，继续发布设定点。")
                        break
                    rospy.sleep(0.05)
                    rate.sleep()
            elif self.ActionExecuteType == "cheak":
                print("检查当前页面语法模式: 执行降落操作")
        except Exception as e:
            self.log_err(f"降落过程中发生错误: {str(e)}")
            raise  # 向上传递异常以在调用函数中处理
        finally:
            self.control_complete(True)  # 确保在任何情况下都设置控制完成标志
            if hasattr(self, 'land_lock_thread_running') and self.land_lock_thread_running:
                self.land_lock_thread_running = False  # 重置线程状态

    

    

    def _control_points_x_y_z_stepsize(self, x_target, y_target, z, stepsize=0.5):
        """
        函数功能: 控制无人机沿直线轨迹从当前位置飞往目标位置，生成每隔指定步长的采样点逐步移动到目标位置。
                当某个轴（X或Y）的剩余距离小于0.1米时，直接将该轴的采样点设置为目标位置。
        
        参数:
        - x_target: 目标位置的X坐标
        - y_target: 目标位置的Y坐标
        - z: 目标高度
        - stepsize: 每个采样点之间的步长（默认0.5米），通过调整步长控制飞行激进度。
        """
        tolerance = 0.03  # 设置容差为3厘米
        axis_tolerance = 0.1  # 轴的容差为0.1米
        
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            hold_yaw = self.get_current_yaw()  # 获取当前的yaw角度

            while True:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("control_points_x_y_z_stepsize 线程被中断")
                    break
                # 在每次循环中获取当前无人机的位置
                current_x, current_y, current_z = self.get_current_xyz()  # 获取实时的X和Y坐标
                self.log_info(f"当前无人机位置: x={current_x}, y={current_y}")
                # 计算无人机到目标点X轴和Y轴的剩余距离
                x_distance = abs(x_target - current_x)
                y_distance = abs(y_target - current_y)

                # 如果X轴或Y轴的剩余距离小于0.1米，直接设置该轴为目标位置
                if x_distance <= axis_tolerance:
                    next_x = x_target
                    self.log_info("X轴距离小于0.1米，直接设置X轴到目标位置")
                else:
                    # 计算X轴方向的下一个采样点，根据步长调整
                    direction_x = (x_target - current_x) / x_distance
                    next_x = current_x + direction_x * min(stepsize, x_distance)

                if y_distance <= axis_tolerance:
                    next_y = y_target
                    self.log_info("Y轴距离小于0.1米，直接设置Y轴到目标位置")
                else:
                    # 计算Y轴方向的下一个采样点，根据步长调整
                    direction_y = (y_target - current_y) / y_distance
                    next_y = current_y + direction_y * min(stepsize, y_distance)

                # 如果无人机已接近目标位置，退出循环
                distance_to_target = math.sqrt((x_target - current_x) ** 2 + (y_target - current_y) ** 2)
                if distance_to_target < tolerance:
                    self.log_info("目标位置接近完毕，无人机停止移动")
                    break

                # 设置并发布下一个目标点
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # 使用本地NED坐标系
                target.type_mask = 0b101111111000  # 使用位置控制，不控制速度
                target.position.x = next_x  # 目标X位置
                target.position.y = next_y  # 目标Y位置
                target.position.z = z  # 目标高度Z
                target.yaw = hold_yaw  # 设置目标yaw角度

                # 发布目标点指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                    self.log_info(f"发布下一个采样点: ({next_x}, {next_y}, {z})")

                rate.sleep()  # 保持20Hz的循环频率，允许其他ROS回调执行
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            # 检查当前页面语法模式下只打印目标位置的信息
            self.log_info(f"检查当前页面语法模式 - 目标位置: X={x_target}, Y={y_target}, Z={z}")


    def _send_position_x_y_z_t_yaw(self, x, y, z, yaw_radians, duration):
        """
        函数名称: send_position_x_y_z_t_yaw
        函数功能: 在指定时间内不断发送目标位置和偏航角度指令，使无人机移动到指定位置并保持朝向。

        参数:
        - x: 目标位置的 X 坐标，单位为米。
        - y: 目标位置的 Y 坐标，单位为米。
        - z: 目标位置的 Z 坐标（高度），单位为米。
        - yaw_radians: 无人机的偏航角，单位为弧度。
        - duration: 指令发送的持续时间，单位为秒。在此时间段内会重复发送位置和偏航角指令以确保无人机执行。

        使用案例:
        - drone.send_position_x_y_z_t_yaw(x=1.0, y=2.0, z=1.5, yaw_radians=math.radians(90), duration=2)

        注意事项:
        - 确保无人机已进入 OFFBOARD 模式，否则指令可能无法生效。
        - 在 `self.ActionExecuteType` 为 `"real"` 的情况下，代码会不断发布指令到 `setpoint_pub`，每 20Hz 发送一次，持续 `duration` 秒。
        - 使用 `duration` 参数设置指令发送的持续时间，以确保无人机有足够时间响应并达到目标位置。
        - 如果无人机不在 OFFBOARD 模式，将输出警告信息。
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            target = PositionTarget()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = 0b101111111000  # 设置功能掩码
            target.position.x = x
            target.position.y = y
            target.position.z = z
            target.yaw = yaw_radians
            self.last_command = target         
            start_time = rospy.Time.now()
            rate = rospy.Rate(20)
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("send_position_x_y_z_t_yaw 线程被中断")
                    break
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                self.log_info(f"Real模式: 发布位置指令到 ({x}, {y}, {z})")
                rate.sleep()
                
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式: 发送位置 ({x}, {y}, {z})")

    
    def _circle_r_z_numpoints(self, radius, z, numpoints=16, tolerance=0.1):
        """
        函数名称: circle_r_z_numpoints
        函数功能: 控制无人机围绕一个柱子飞行，飞行过程中实时获取当前位置，并在一圈结束后返回起始点。

        参数:
        - radius: 无人机与柱子的距离（半径），单位为米。确保这个距离足够大，以免碰撞。
        - z: 飞行高度，单位为米。
        - numpoints: 采样点的数量，默认为16。采样点越多，环绕越细致。
        - tolerance: 容差，用于判断无人机是否返回到起点，单位为米。

        使用案例:
        - drone.circle_r_z_numpoints(radius=1.0, z=1.0, numpoints=16, tolerance=0.1)

        说明:
        - 无人机将围绕柱子飞行，飞行过程中会实时获取当前位置 (x, y)，并确保在一圈飞行完成后回到起点。
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
    
            rate = rospy.Rate(20)  # 设定循环频率为20Hz

            # 获取起点的 x 和 y 坐标
            start_x, start_y ,start_z= self.get_current_xyz()  # 初次调用 get_current_xyz() 获取起点坐标
            current_yaw = self.get_current_yaw()  # 获取当前的 yaw 角

            # 计算柱子的位置，假设柱子位于无人机当前前方
            pillar_x = start_x + radius * math.cos(current_yaw)
            pillar_y = start_y + radius * math.sin(current_yaw)

            self.log_info(f"柱子位置: ({pillar_x:.2f}, {pillar_y:.2f}, {z})")

            # 每个采样点之间的角度增量，2π 代表一整圈，分为 numpoints 个点
            angle_increment = 2 * math.pi / numpoints
            # 获取初始角度，无人机当前方向指向柱子的角度
            initial_theta = math.atan2(start_y - pillar_y, start_x - pillar_x)

            theta = initial_theta  # 初始角度
            point_counter = 1  # 采样点计数器

            while True:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("circle_r_z_numpoints 线程被中断")
                    break
                # 每次移动前，实时获取当前位置
                current_x, current_y,current_z = self.get_current_xyz()  # 循环中实时获取当前坐标

                # 计算当前采样点的位置
                target_x = pillar_x + radius * math.cos(theta)
                target_y = pillar_y + radius * math.sin(theta)
                yaw_radians = math.atan2(pillar_y - target_y, pillar_x - target_x)  # 计算偏航角

                # 打印日志，输出当前采样点的信息
                self.log_info(f"移动到采样点 {point_counter} 位置: ({target_x:.3f}, {target_y:.3f}, {z})")
                self.log_info(f"调整偏航角至 {math.degrees(yaw_radians) % 360:.2f}°")

                # 发送当前位置和偏航角指令，移动到目标点并调整偏航角
                self.send_position_x_y_z_t_yaw(target_x, target_y, z, 0.4, yaw_radians)
                self.log_info(f"环绕柱子采样点 {point_counter} 完成")
                point_counter += 1

                # 更新角度，用于下一个采样点的计算
                theta += angle_increment

                # 判断是否完成了所有的采样点
                if point_counter > numpoints - 1:
                    # 计算当前坐标与起点坐标的差值
                    delta_x = abs(current_x - start_x)
                    delta_y = abs(current_y - start_y)

                    # 打印当前位置与起点的差值信息
                    self.log_info(f"当前与起点的 X 轴差值: {delta_x:.4f} 米，Y 轴差值: {delta_y:.4f} 米")

                    # 如果 x 或 y 轴上的差值小于等于容差，则判定为完成一圈飞行
                    if delta_x <= tolerance or delta_y <= tolerance:
                        self.log_info("已返回起点的 X 或 Y 位置，环绕飞行完成")
                        break  # 完成一圈后退出循环

                # 如果所有采样点已完成但未回到起点，则继续生成采样点
                if point_counter > numpoints:
                    self.log_info("初始采样点已完成，继续生成新的采样点以完成环绕飞行")
                    # 这里可以根据需要调整 angle_increment，以控制新的采样点密度

            self.log_info("环绕飞行过程完成")

            # 在环绕飞行结束后，返回到起点并调整偏航角为起始角度
            self.log_info("返回起点坐标，并调整偏航角为起始偏航角")
            self.send_position_x_y_z_t_yaw(start_x, start_y, z, 0.8, current_yaw)
            
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType != "cheak":
            self.log_info("检查当前页面语法模式下不执行围绕柱子的飞行")
            return


    def _track_velocity_z_centertol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20):
        """
        视觉追踪：xy速度控制，z高度采用速度控制
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                if not hasattr(self, 'count'):
                    self.count = 0
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m")
                consecutive_centered_frames = 0
                required_centered_frames = 30
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_centertol 线程被中断")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    target.type_mask = 0b110111000011  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 3.0, -0.5, 0.5)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        if abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance:
                            consecutive_centered_frames += 1
                            self.log_info(f"目标在中心范围内，已持续 {consecutive_centered_frames} 帧")
                            if consecutive_centered_frames >= required_centered_frames:
                                self.log_info("目标已稳定在中心范围内，任务完成")
                                break
                        else:
                            consecutive_centered_frames = 0
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        consecutive_centered_frames = 0
                    rate.sleep()
                # 结束时发送停止命令
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                target.type_mask = 0b110111000011
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0  # 停止垂直运动
                target.yaw = 0
                self.setpoint_pub.publish(target)
                _, _, final_z = self.get_current_xyz()
                rospy.loginfo(f"追踪控制结束: 最终高度={final_z:.2f}m, 目标高度={z}m")
            elif self.ActionExecuteType == "cheak":
                self.log_info("检查当前页面语法模式下不执行追踪目标")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
        finally:
            self.control_complete(True)

    def _track_velocity_z_centertol_ttol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, timeout=3):
        """
        视觉追踪（带超时）：xy速度控制，z高度采用速度控制
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                if not hasattr(self, 'count'):
                    self.count = 0
                self.control_complete(False)
                # 初始化超时标志
                self.track_timeout_flag = False
                
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制(带超时)，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m")
                consecutive_centered_frames = 0
                required_centered_frames = 30
                target_lost_start_time = None
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_centertol_ttol 线程被中断")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    target.type_mask = 0b110111000011  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 3.0, -0.5, 0.5)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        target_lost_start_time = None
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        if abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance:
                            consecutive_centered_frames += 1
                            self.log_info(f"目标在中心范围内，已持续 {consecutive_centered_frames} 帧")
                            if consecutive_centered_frames >= required_centered_frames:
                                self.log_info("目标已稳定在中心范围内，任务完成")
                                self.track_timeout_flag = False  # 成功追踪，清除超时标志
                                break
                        else:
                            consecutive_centered_frames = 0
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        if target_lost_start_time is None:
                            target_lost_start_time = rospy.Time.now()
                            self.log_warn("目标丢失，开始计时")
                        elif (rospy.Time.now() - target_lost_start_time).to_sec() > timeout:
                            self.log_warn(f"目标丢失超过{timeout}秒，中断追踪")
                            self.track_timeout_flag = True  # 设置超时标志
                            break
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        consecutive_centered_frames = 0
                    rate.sleep()
                # 结束时发送停止命令
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                target.type_mask = 0b110111000011
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0  # 停止垂直运动
                target.yaw = 0
                self.setpoint_pub.publish(target)
                _, _, final_z = self.get_current_xyz()
                rospy.loginfo(f"追踪控制结束: 最终高度={final_z:.2f}m, 目标高度={z}m, 是否超时={self.track_timeout_flag}")
            elif self.ActionExecuteType == "cheak":
                self.log_info("检查当前页面语法模式下不执行追踪目标")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
            self.track_timeout_flag = True  # 发生错误时也设置超时标志
        finally:
            self.control_complete(True)

    

    

    def _track_velocity_z_t(self, z, duration, MAX_VEL=0.5, MIN_VEL=0, Kp=0.001):
        """
        视觉追踪：xy速度控制，z高度采用速度控制
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m")
                start_time = rospy.Time.now()
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_t 线程被中断")
                        break
                    if (rospy.Time.now() - start_time).to_sec() > duration:
                        rospy.loginfo("追踪超时，结束追踪")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.type_mask = 0b110111000011  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 3.0, -0.5, 0.5)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = hold_yaw
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = hold_yaw
                        self.setpoint_pub.publish(target)
                    rate.sleep()
                # 结束时发送停止命令
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b110111000011
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0  # 停止垂直运动
                target.yaw = hold_yaw
                self.setpoint_pub.publish(target)
                _, _, final_z = self.get_current_xyz()
                rospy.loginfo(f"追踪控制结束: 最终高度={final_z:.2f}m, 目标高度={z}m")
            elif self.ActionExecuteType == "cheak":
                self.log_info("检查当前页面语法模式下不执行追踪目标")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
        finally:
            self.control_complete(True)

    def _publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame):
        """
        函数名称: publish_nav_goal_x_y_z_yaw_tol_frame
        函数功能: 发布导航目标，并等待无人机到达目标位置的容差范围内（默认0.05米）
        参数:
        - x: 目标位置的X坐标
        - y: 目标位置的Y坐标
        - z: 目标位置的Z坐标
        - yaw: 目标偏航角（度）
        - frame: 坐标系（1表示'map'，2表示'base_link'）
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                if "nolog" not in self.ActionExecuteType:
                    self.log_info(f"Publishing nav goal with tolerance: x={x}, y={y}, z={z}, yaw={yaw}, frame={frame}")

                publish_nav_goal_x_y_z_yaw_tol_frame(
                    x, y, z, yaw, tolerance, frame, self.setpoint_pub, self.nav_goal_pub, self.log_info ,self.ActionExecuteType ,stop_thread_flag=self.stop_thread_flag
                )
                
                self.control_complete(True)  # 设置控制完成标志
            elif self.ActionExecuteType == "cheak":
                print(f"检查当前页面语法模式: 发布带容差的导航目标到 (x: {x}, y: {y}, z: {z}, yaw: {yaw}, frame: {frame})，持续时间 秒")
        except Exception as e:
            self.log_err(f"导航过程中发生错误: {str(e)}")
            raise
        finally:
            self.control_complete(True)  # 确保在任何情况下都设置控制完成标志
            if self.nav_goal_thread_running:
                self.nav_goal_thread_running = False  # 重置线程运行标志

    def _control_yaw_z_t(self, yaw, z, t):
        """
        函数名称: control_yaw_z_t
        函数功能: 控制无人机在指定高度并在指定时间内进行偏航控制，保持当前位置的X和Y不变。
        
        参数:
        - z: 目标高度
        - yaw: 相对偏航角，范围为 0 到 360 度 或 0 到 -360 度。
        - t: 持续时间（秒）

        偏航角说明:
        - 正值: 顺时针旋转，角度范围为 0 到 360 度。
        - 负值: 逆时针旋转，角度范围为 0 到 -360 度。
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            # 获取当前X、Y坐标和当前yaw角度，只读取一次
            current_x, current_y, current_z = self.get_current_xyz()
            hold_yaw = self.get_current_yaw()  # 获取当前yaw角度

            # 将目标yaw角度转换为弧度并反转正负符号
            relative_yaw = -yaw * (math.pi / 180.0)
            # 获取当前时间
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < t:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("control_yaw_z_t 线程被中断")
                    break
                # 设置目标的yaw和保持当前位置的x, y
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # 使用本地NED坐标系
                target.type_mask = 0b101111111000  # 使用位置控制并控制yaw
                target.position.x = current_x  # 保持当前位置的X坐标
                target.position.y = current_y  # 保持当前位置的Y坐标
                target.position.z = z  # 目标高度
                target.yaw = hold_yaw + relative_yaw  # 使用相对yaw值
                # 发布目标点指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                    self.log_info(f"发布偏航控制: yaw={yaw} 度, 高度={z}, 当前x={current_x}, y={current_y}")
                rate.sleep()  # 保持20Hz的循环频率
            self.log_info("偏航控制完成")
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            # 检查当前页面语法模式下只输出一行日志并打印传入的时间
            print(f"检查当前页面语法模式 - yaw控制: yaw={yaw} 度, 高度={z}, 传入时间={t} 秒")

    def _send_velocity_vx_vy_z_t(self, vx, vy, z, duration):
        """
        函数名称: send_velocity_vx_vy_z_t
        函数功能: 在局部坐标系下以指定的速度控制无人机移动一段时间
        参数:
        - vx: X方向速度
        - vy: Y方向速度
        - z: 目标高度
        - duration: 持续时间（秒）
        使用案例:
        - 控制无人机以速度(vx, vy)在高度z飞行duration秒: action.send_velocity_vx_vy_z_t(1, 1, 5, 10)
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            start_time = rospy.Time.now()
            
            # 保存当前位置和yaw角度，仅在初次进入循环时保存一次
            current_x, current_y,current_z = self.get_current_xyz()  # 获取当前X和Y坐标
            hold_yaw = self.get_current_yaw()  # 保存初始yaw角度

            # 开始控制逻辑
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("send_velocity_vx_vy_z_t 线程被中断")
                    break
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

                # 设置目标速度和位置控制
                if vx == 0 and vy != 0:
                    target.type_mask = 0b101111000000  # 保持X位置
                    target.position.x = current_x
                    target.velocity.x = 0
                    target.velocity.y = vy
                    target.velocity.z = 0
                elif vy == 0 and vx != 0:
                    target.type_mask = 0b101111000000  # 保持Y位置
                    target.position.y = current_y
                    target.velocity.y = 0
                    target.velocity.x = vx
                    target.velocity.z = 0
                elif vx == 0 and vy == 0:
                    target.type_mask = 0b101111000000  # 保持X和Y位置
                    target.position.x = current_x
                    target.velocity.x = 0
                    target.position.y = current_y
                    target.velocity.y = 0
                    target.velocity.z = 0
                else:
                    target.type_mask = 0b101111000011  # 使用速度控制
                    target.velocity.x = vx
                    target.velocity.y = vy
                    target.velocity.z = 0
                
                # 设置高度和yaw角度
                target.position.z = z  # 保持目标高度
                target.yaw = hold_yaw  # 设置初始保存的yaw角度

                # 发布目标指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)

                # ROS 日志打印输出当前状态
                self.log_info(f"Real模式 - 发布速度控制: 速度(vx={vx}, vy={vy}), 位置保持(X={current_x if vx == 0 else 'N/A'}, Y={current_y if vy == 0 else 'N/A'}), 高度={z}, Yaw={hold_yaw}")

                rate.sleep()  # 保持20Hz的循环频率
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式 - 发布目标指令: 速度(vx={vx}, vy={vy}), 位置保持(X={self.current_position_x if vx == 0 else 'N/A'}, Y={self.current_position_y if vy == 0 else 'N/A'}), 高度={z}, Yaw={self.current_yaw}")

    

    
            
    def _land_lock_vz_t(self, vz, duration):
        """
        函数名称: land_vz_t
        函数功能: 控制无人机降落
        参数:
        - vz: 下降速度（米/秒）
        - duration: 降落时间（秒）
        使用案例:
        - 以-0.4 m/s的速度降落10秒: action.land_lock_vz_t(-0.4, 10)
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(20)  # 设置频率为20Hz
                state_start_time = rospy.Time.now()
                while (rospy.Time.now() - state_start_time).to_sec() < duration:
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("land_lock_vz_t 线程被中断")
                        break
                    setpoint_raw = PositionTarget()
                    setpoint_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    setpoint_raw.type_mask = 0b101111000011  # 设置功能掩码
                    setpoint_raw.position.z = 0
                    setpoint_raw.velocity.x = 0
                    setpoint_raw.velocity.y = 0
                    setpoint_raw.velocity.z = vz
                    self.setpoint_pub.publish(setpoint_raw)
                    if (rospy.Time.now() - state_start_time).to_sec() > 3:
                        setpoint_raw.type_mask = 0b101111000011  # 设置功能掩码
                        self.log_info("Landing complete")
                        # 自动强制上锁，有高空坠落风险
                        disarm_cmd_long = CommandLongRequest()
                        disarm_cmd_long.broadcast = False
                        disarm_cmd_long.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
                        disarm_cmd_long.param1 = 0  # Disarm
                        disarm_cmd_long.param2 = 21196  # Kill no check landed
                        rate1 = rospy.Rate(1.0)
                        while not rospy.is_shutdown():
                            if self.stop_thread_flag.is_set():
                                rospy.logwarn("land_lock_vz_t 线程被中断")
                                break
                            result = self.command_service(disarm_cmd_long)
                            if result.success:
                                print("无人机已解除武装 (disarmed)")
                                break
                            else:
                                print("解除武装失败，重试中...")
                            rate1.sleep()
                        print("紧急解除武装完成，继续发布设定点。")
                        break
                    rospy.sleep(0.05)
                    rate.sleep()
            elif self.ActionExecuteType == "cheak":
                print("检查当前页面语法模式: 执行降落操作")
        except Exception as e:
            self.log_err(f"降落过程中发生错误: {str(e)}")
            raise  # 向上传递异常以在调用函数中处理
        finally:
            self.control_complete(True)  # 确保在任何情况下都设置控制完成标志
            if hasattr(self, 'land_lock_thread_running') and self.land_lock_thread_running:
                self.land_lock_thread_running = False  # 重置线程状态

    

    

    def _control_points_x_y_z_stepsize(self, x_target, y_target, z, stepsize=0.5):
        """
        函数功能: 控制无人机沿直线轨迹从当前位置飞往目标位置，生成每隔指定步长的采样点逐步移动到目标位置。
                当某个轴（X或Y）的剩余距离小于0.1米时，直接将该轴的采样点设置为目标位置。
        
        参数:
        - x_target: 目标位置的X坐标
        - y_target: 目标位置的Y坐标
        - z: 目标高度
        - stepsize: 每个采样点之间的步长（默认0.5米），通过调整步长控制飞行激进度。
        """
        tolerance = 0.03  # 设置容差为3厘米
        axis_tolerance = 0.1  # 轴的容差为0.1米
        
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            rate = rospy.Rate(20)  # 设置频率为20Hz
            hold_yaw = self.get_current_yaw()  # 获取当前的yaw角度

            while True:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("control_points_x_y_z_stepsize 线程被中断")
                    break
                # 在每次循环中获取当前无人机的位置
                current_x, current_y, current_z = self.get_current_xyz()  # 获取实时的X和Y坐标
                self.log_info(f"当前无人机位置: x={current_x}, y={current_y}")
                # 计算无人机到目标点X轴和Y轴的剩余距离
                x_distance = abs(x_target - current_x)
                y_distance = abs(y_target - current_y)

                # 如果X轴或Y轴的剩余距离小于0.1米，直接设置该轴为目标位置
                if x_distance <= axis_tolerance:
                    next_x = x_target
                    self.log_info("X轴距离小于0.1米，直接设置X轴到目标位置")
                else:
                    # 计算X轴方向的下一个采样点，根据步长调整
                    direction_x = (x_target - current_x) / x_distance
                    next_x = current_x + direction_x * min(stepsize, x_distance)

                if y_distance <= axis_tolerance:
                    next_y = y_target
                    self.log_info("Y轴距离小于0.1米，直接设置Y轴到目标位置")
                else:
                    # 计算Y轴方向的下一个采样点，根据步长调整
                    direction_y = (y_target - current_y) / y_distance
                    next_y = current_y + direction_y * min(stepsize, y_distance)

                # 如果无人机已接近目标位置，退出循环
                distance_to_target = math.sqrt((x_target - current_x) ** 2 + (y_target - current_y) ** 2)
                if distance_to_target < tolerance:
                    self.log_info("目标位置接近完毕，无人机停止移动")
                    break

                # 设置并发布下一个目标点
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # 使用本地NED坐标系
                target.type_mask = 0b101111111000  # 使用位置控制，不控制速度
                target.position.x = next_x  # 目标X位置
                target.position.y = next_y  # 目标Y位置
                target.position.z = z  # 目标高度Z
                target.yaw = hold_yaw  # 设置目标yaw角度

                # 发布目标点指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                    self.log_info(f"发布下一个采样点: ({next_x}, {next_y}, {z})")

                rate.sleep()  # 保持20Hz的循环频率，允许其他ROS回调执行
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            # 检查当前页面语法模式下只打印目标位置的信息
            self.log_info(f"检查当前页面语法模式 - 目标位置: X={x_target}, Y={y_target}, Z={z}")


    def _send_position_x_y_z_t_yaw(self, x, y, z, yaw_radians, duration):
        """
        函数名称: send_position_x_y_z_t_yaw
        函数功能: 在指定时间内不断发送目标位置和偏航角度指令，使无人机移动到指定位置并保持朝向。

        参数:
        - x: 目标位置的 X 坐标，单位为米。
        - y: 目标位置的 Y 坐标，单位为米。
        - z: 目标位置的 Z 坐标（高度），单位为米。
        - yaw_radians: 无人机的偏航角，单位为弧度。
        - duration: 指令发送的持续时间，单位为秒。在此时间段内会重复发送位置和偏航角指令以确保无人机执行。

        使用案例:
        - drone.send_position_x_y_z_t_yaw(x=1.0, y=2.0, z=1.5, yaw_radians=math.radians(90), duration=2)

        注意事项:
        - 确保无人机已进入 OFFBOARD 模式，否则指令可能无法生效。
        - 在 `self.ActionExecuteType` 为 `"real"` 的情况下，代码会不断发布指令到 `setpoint_pub`，每 20Hz 发送一次，持续 `duration` 秒。
        - 使用 `duration` 参数设置指令发送的持续时间，以确保无人机有足够时间响应并达到目标位置。
        - 如果无人机不在 OFFBOARD 模式，将输出警告信息。
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
            target = PositionTarget()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = 0b101111111000  # 设置功能掩码
            target.position.x = x
            target.position.y = y
            target.position.z = z
            target.yaw = yaw_radians
            self.last_command = target         
            start_time = rospy.Time.now()
            rate = rospy.Rate(20)
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("send_position_x_y_z_t_yaw 线程被中断")
                    break
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                # self.log_info(f"Real模式: 发布位置指令到 ({x}, {y}, {z})")
                rate.sleep()
                
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式: 发送位置 ({x}, {y}, {z})")

    
    def _circle_r_z_numpoints(self, radius, z, numpoints=16, tolerance=0.1):
        """
        函数名称: circle_r_z_numpoints
        函数功能: 控制无人机围绕一个柱子飞行，飞行过程中实时获取当前位置，并在一圈结束后返回起始点。

        参数:
        - radius: 无人机与柱子的距离（半径），单位为米。确保这个距离足够大，以免碰撞。
        - z: 飞行高度，单位为米。
        - numpoints: 采样点的数量，默认为16。采样点越多，环绕越细致。
        - tolerance: 容差，用于判断无人机是否返回到起点，单位为米。

        使用案例:
        - drone.circle_r_z_numpoints(radius=1.0, z=1.0, numpoints=16, tolerance=0.1)

        说明:
        - 无人机将围绕柱子飞行，飞行过程中会实时获取当前位置 (x, y)，并确保在一圈飞行完成后回到起点。
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    
                    return
    
            rate = rospy.Rate(20)  # 设定循环频率为20Hz

            # 获取起点的 x 和 y 坐标
            start_x, start_y ,start_z= self.get_current_xyz()  # 初次调用 get_current_xyz() 获取起点坐标
            current_yaw = self.get_current_yaw()  # 获取当前的 yaw 角

            # 计算柱子的位置，假设柱子位于无人机当前前方
            pillar_x = start_x + radius * math.cos(current_yaw)
            pillar_y = start_y + radius * math.sin(current_yaw)

            self.log_info(f"柱子位置: ({pillar_x:.2f}, {pillar_y:.2f}, {z})")

            # 每个采样点之间的角度增量，2π 代表一整圈，分为 numpoints 个点
            angle_increment = 2 * math.pi / numpoints
            # 获取初始角度，无人机当前方向指向柱子的角度
            initial_theta = math.atan2(start_y - pillar_y, start_x - pillar_x)

            theta = initial_theta  # 初始角度
            point_counter = 1  # 采样点计数器

            while True:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("circle_r_z_numpoints 线程被中断")
                    break
                # 每次移动前，实时获取当前位置
                current_x, current_y,current_z = self.get_current_xyz()  # 循环中实时获取当前坐标

                # 计算当前采样点的位置
                target_x = pillar_x + radius * math.cos(theta)
                target_y = pillar_y + radius * math.sin(theta)
                yaw_radians = math.atan2(pillar_y - target_y, pillar_x - target_x)  # 计算偏航角

                # 打印日志，输出当前采样点的信息
                self.log_info(f"移动到采样点 {point_counter} 位置: ({target_x:.3f}, {target_y:.3f}, {z})")
                self.log_info(f"调整偏航角至 {math.degrees(yaw_radians) % 360:.2f}°")

                # 发送当前位置和偏航角指令，移动到目标点并调整偏航角
                self.send_position_x_y_z_t_yaw(target_x, target_y, z, 0.4,yaw_radians)
                self.log_info(f"环绕柱子采样点 {point_counter} 完成")
                point_counter += 1

                # 更新角度，用于下一个采样点的计算
                theta += angle_increment

                # 判断是否完成了所有的采样点
                if point_counter > numpoints - 1:
                    # 计算当前坐标与起点坐标的差值
                    delta_x = abs(current_x - start_x)
                    delta_y = abs(current_y - start_y)

                    # 打印当前位置与起点的差值信息
                    self.log_info(f"当前与起点的 X 轴差值: {delta_x:.4f} 米，Y 轴差值: {delta_y:.4f} 米")

                    # 如果 x 或 y 轴上的差值小于等于容差，则判定为完成一圈飞行
                    if delta_x <= tolerance or delta_y <= tolerance:
                        self.log_info("已返回起点的 X 或 Y 位置，环绕飞行完成")
                        break  # 完成一圈后退出循环

                # 如果所有采样点已完成但未回到起点，则继续生成采样点
                if point_counter > numpoints:
                    self.log_info("初始采样点已完成，继续生成新的采样点以完成环绕飞行")
                    # 这里可以根据需要调整 angle_increment，以控制新的采样点密度

            self.log_info("环绕飞行过程完成")

            # 在环绕飞行结束后，返回到起点并调整偏航角为起始角度
            self.log_info("返回起点坐标，并调整偏航角为起始偏航角")
            self.send_position_x_y_z_t_yaw(start_x, start_y, z, 0.8, current_yaw)
            
            
            self.control_complete(True)  # 设置控制完成标志
        elif self.ActionExecuteType != "cheak":
            self.log_info("检查当前页面语法模式下不执行围绕柱子的飞行")
            return


    def _track_velocity_z_centertol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20):
        """
        视觉追踪：xy速度控制，z高度采用速度控制
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                if not hasattr(self, 'count'):
                    self.count = 0
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m")
                consecutive_centered_frames = 0
                required_centered_frames = 30
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_centertol 线程被中断")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    target.type_mask = 0b110111000011  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 3.0, -0.5, 0.5)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        if abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance:
                            consecutive_centered_frames += 1
                            self.log_info(f"目标在中心范围内，已持续 {consecutive_centered_frames} 帧")
                            if consecutive_centered_frames >= required_centered_frames:
                                self.log_info("目标已稳定在中心范围内，任务完成")
                                break
                        else:
                            consecutive_centered_frames = 0
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        consecutive_centered_frames = 0
                    rate.sleep()
                # 结束时发送停止命令
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                target.type_mask = 0b110111000011
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0  # 停止垂直运动
                target.yaw = 0
                self.setpoint_pub.publish(target)
                _, _, final_z = self.get_current_xyz()
                rospy.loginfo(f"追踪控制结束: 最终高度={final_z:.2f}m, 目标高度={z}m")
            elif self.ActionExecuteType == "cheak":
                self.log_info("检查当前页面语法模式下不执行追踪目标")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
        finally:
            self.control_complete(True)

    def _track_velocity_z_centertol_ttol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, timeout=3):
        """
        视觉追踪（带超时）：xy速度控制，z高度采用速度控制
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                if not hasattr(self, 'count'):
                    self.count = 0
                self.control_complete(False)
                # 初始化超时标志
                self.track_timeout_flag = False
                
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制(带超时)，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m")
                consecutive_centered_frames = 0
                required_centered_frames = 30
                target_lost_start_time = None
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_centertol_ttol 线程被中断")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    target.type_mask = 0b110111000011  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 3.0, -0.5, 0.5)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        target_lost_start_time = None
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        if abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance:
                            consecutive_centered_frames += 1
                            self.log_info(f"目标在中心范围内，已持续 {consecutive_centered_frames} 帧")
                            if consecutive_centered_frames >= required_centered_frames:
                                self.log_info("目标已稳定在中心范围内，任务完成")
                                self.track_timeout_flag = False  # 成功追踪，清除超时标志
                                break
                        else:
                            consecutive_centered_frames = 0
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        if target_lost_start_time is None:
                            target_lost_start_time = rospy.Time.now()
                            self.log_warn("目标丢失，开始计时")
                        elif (rospy.Time.now() - target_lost_start_time).to_sec() > timeout:
                            self.log_warn(f"目标丢失超过{timeout}秒，中断追踪")
                            self.track_timeout_flag = True  # 设置超时标志
                            break
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = 0
                        self.setpoint_pub.publish(target)
                        consecutive_centered_frames = 0
                    rate.sleep()
                # 结束时发送停止命令
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                target.type_mask = 0b110111000011
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0  # 停止垂直运动
                target.yaw = 0
                self.setpoint_pub.publish(target)
                _, _, final_z = self.get_current_xyz()
                rospy.loginfo(f"追踪控制结束: 最终高度={final_z:.2f}m, 目标高度={z}m, 是否超时={self.track_timeout_flag}")
            elif self.ActionExecuteType == "cheak":
                self.log_info("检查当前页面语法模式下不执行追踪目标")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
            self.track_timeout_flag = True  # 发生错误时也设置超时标志
        finally:
            self.control_complete(True)

    

    

    def _track_velocity_z_t(self, z, duration, MAX_VEL=0.5, MIN_VEL=0, Kp=0.001):
        """
        视觉追踪：xy速度控制，z高度采用速度控制
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m")
                start_time = rospy.Time.now()
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_t 线程被中断")
                        break
                    if (rospy.Time.now() - start_time).to_sec() > duration:
                        rospy.loginfo("追踪超时，结束追踪")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.type_mask = 0b110111000011  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 3.0, -0.5, 0.5)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = hold_yaw
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = hold_yaw
                        self.setpoint_pub.publish(target)
                    rate.sleep()
                # 结束时发送停止命令
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b110111000011
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0  # 停止垂直运动
                target.yaw = hold_yaw
                self.setpoint_pub.publish(target)
                _, _, final_z = self.get_current_xyz()
                rospy.loginfo(f"追踪控制结束: 最终高度={final_z:.2f}m, 目标高度={z}m")
            elif self.ActionExecuteType == "cheak":
                self.log_info("检查当前页面语法模式下不执行追踪目标")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
        finally:
            self.control_complete(True)

    def _publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame):
        """
        函数名称: publish_nav_goal_x_y_z_yaw_tol_frame
        函数功能: 发布导航目标，并等待无人机到达目标位置的容差范围内（默认0.05米）
        参数:
        - x: 目标位置的X坐标
        - y: 目标位置的Y坐标
        - z: 目标位置的Z坐标
        - yaw: 目标偏航角（度）
        - frame: 坐标系（1表示'map'，2表示'base_link'）
        """
        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return

                if "nolog" not in self.ActionExecuteType:
                    self.log_info(f"Publishing nav goal with tolerance: x={x}, y={y}, z={z}, yaw={yaw}, frame={frame}")

                publish_nav_goal_x_y_z_yaw_tol_frame(
                    x, y, z, yaw, tolerance, frame,
                    self.setpoint_pub, self.nav_goal_pub,
                    self.log_info,
                    self.ActionExecuteType,
                    stop_thread_flag=self.stop_thread_flag
                )
                
            elif self.ActionExecuteType == "cheak":
                print(f"检查当前页面语法模式: 发布带容差的导航目标到 (x: {x}, y: {y}, z: {z}, yaw: {yaw}, frame: {frame})")
        except Exception as e:
            self.log_err(f"导航过程中发生错误: {str(e)}")
            raise
        finally:
            self.control_complete(True)  # 确保在任何情况下都设置控制完成标志
            if self.nav_goal_thread_running:
                self.nav_goal_thread_running = False  # 重置线程运行标志

    def _control_servo_id_angle_t(self, servo_id, angle, duration):
        """
        函数名称: _control_servo_id_angle_t
        函数功能: 控制指定ID的舵机旋转到指定角度，并等待指定时间
        参数:
        - servo_id: 舵机ID号，范围1-6
        - angle: 目标角度，范围0-180度
        - duration: 等待时间，单位秒
        返回值:
        - bool: 控制是否成功
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            # 检查参数有效性
            if servo_id < 1 or servo_id > 6:
                self.log_err(f"舵机ID无效，应为1-6，当前值: {servo_id}")
                self.control_complete(True)
                return False
                
            if angle < 0 or angle > 180:
                self.log_err(f"角度无效，应为0-180，当前值: {angle}")
                self.control_complete(True)
                return False
            
            # 格式化命令：舵机号 + 三位角度值（补零）
            command = f"{servo_id}{angle:03d}"
            
            # 重置状态接收标志
            self.servo_status_received = False
            
            # 发布命令
            msg = String()
            msg.data = command
            self.servo_pub.publish(msg)
            
            self.log_info(f"已发送命令: {command} (舵机{servo_id}旋转到{angle}度)")
            
            # 等待状态回复，最多等待1秒
            timeout = rospy.Time.now() + rospy.Duration(1.0)
            while not self.servo_status_received and rospy.Time.now() < timeout:
                if self.stop_thread_flag.is_set():
                    self.log_warn("control_servo_id_angle_t 线程被中断")
                    self.control_complete(True)
                    return False
                rospy.sleep(0.01)
                
            if not self.servo_status_received:
                self.log_warn("未收到舵机控制状态回复")
            
            # 等待指定的持续时间
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    self.log_warn("control_servo_id_angle_t 线程被中断")
                    break
                rospy.sleep(0.1)  # 降低CPU使用率
            
            self.control_complete(True)
            return self.servo_status == 1  # 返回控制是否成功
            
        elif self.ActionExecuteType == "cheak":
            self.log_info(f"检查当前页面语法模式: 控制舵机{servo_id}旋转到{angle}度，等待{duration}秒")
            return True

##########################################################################################################################################
#
#        
#
#
#       # ---------------多线程管理----------------- #
#
#
#
#
#
#
#多线程管理#########################################################################################################################################

    def thread_track_velocity_z_centertol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, use_thread=False):
        """
        线程管理函数，用于启动或停止带中心点容差的追踪控制线程
        """
        try:
            if use_thread:
                # 如果有旧线程在运行，先停止它
                if self.track_velocity_thread_running:
                    self.stop_thread_flag.set()
                    if self.track_velocity_thread and self.track_velocity_thread.is_alive():
                        self.track_velocity_thread.join(timeout=2.0)
                    self.log_info("旧的追踪线程已停止")
                # 重置状态并启动新线程
                self.track_velocity_thread_running = True
                self.stop_thread_flag.clear()  # 确保终止标志未设置
                self.control_complete(False)  # 确保控制标志设置为未完成
                self.track_velocity_thread = threading.Thread(
                    target=self._track_velocity_z_centertol,
                    args=(z, MAX_VEL, MIN_VEL, Kp, center_tolerance)
                )
                self.track_velocity_thread.daemon = True
                self.track_velocity_thread.start()
                self.log_info(f"追踪线程已启动: z={z}, 中心容差={center_tolerance}像素")
            else:
                # 停止线程
                if self.track_velocity_thread_running:
                    self.track_velocity_thread_running = False
                    self.stop_thread_flag.set()  # 设置终止标志
                    if self.track_velocity_thread and self.track_velocity_thread.is_alive():
                        self.track_velocity_thread.join(timeout=0.3)  # 等待最多2秒
                    self.log_warn("追踪线程已终止")
                else:
                    self.log_warn("追踪线程未在运行")
                # 确保控制标志被设置为完成
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.track_velocity_thread_running = False
            self.control_complete(True)

    def thread_track_velocity_z_centertol_ttol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, timeout=3, use_thread=False):
        """
        线程管理函数，用于启动或停止带超时的追踪控制线程
        """
        try:
            if use_thread:
                # 如果有旧线程在运行，先停止它
                if self.track_velocity_thread_running:
                    self.stop_thread_flag.set()
                    if self.track_velocity_thread and self.track_velocity_thread.is_alive():
                        self.track_velocity_thread.join(timeout=2.0)
                    self.log_info("旧的追踪线程已停止")
                # 重置状态并启动新线程
                self.track_velocity_thread_running = True
                self.stop_thread_flag.clear()  # 确保终止标志未设置
                self.control_complete(False)  # 确保控制标志设置为未完成
                self.track_velocity_thread = threading.Thread(
                    target=self._track_velocity_z_centertol_ttol,
                    args=(z, MAX_VEL, MIN_VEL, Kp, center_tolerance, timeout)
                )
                self.track_velocity_thread.daemon = True
                self.track_velocity_thread.start()
                self.log_info(f"追踪线程已启动: z={z}, 中心容差={center_tolerance}像素, 超时={timeout}秒")
            else:
                # 停止线程
                if self.track_velocity_thread_running:
                    self.track_velocity_thread_running = False
                    self.stop_thread_flag.set()  # 设置终止标志
                    if self.track_velocity_thread and self.track_velocity_thread.is_alive():
                        self.track_velocity_thread.join(timeout=2.0)  # 等待最多2秒
                    self.log_warn("追踪线程已终止")
                else:
                    self.log_warn("追踪线程未在运行")
                # 确保控制标志被设置为完成
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.track_velocity_thread_running = False
            self.control_complete(True)
    
    def thread_send_velocity_vx_vy_z_t(self, vx, vy, z, duration, use_thread=False):
        """
        线程管理函数，用于启动或停止速度控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.velocity_thread_running:
                    rospy.logwarn("速度控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.velocity_thread and self.velocity_thread.is_alive():
                        self.velocity_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.velocity_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.velocity_thread = threading.Thread(
                        target=self._send_velocity_vx_vy_z_t,
                        args=(vx, vy, z, duration)
                    )
                    self.velocity_thread.daemon = True  # 设为守护线程
                    self.velocity_thread.start()
                    rospy.loginfo(f"速度控制线程已启动: vx={vx}, vy={vy}, z={z}, 持续={duration}s")
                except Exception as e:
                    rospy.logerr(f"启动速度控制线程失败: {str(e)}")
                    self.velocity_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.velocity_thread_running:
                    self.velocity_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止速度控制线程...")
                    if self.velocity_thread and self.velocity_thread.is_alive():
                        self.velocity_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("速度控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的速度控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.velocity_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
            
    def thread_land_lock_vz_t(self, vz, duration, use_thread=False):
        """
        线程管理函数，用于启动或停止降落控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.land_lock_thread_running:
                    rospy.logwarn("降落线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if hasattr(self, 'land_lock_thread') and self.land_lock_thread and self.land_lock_thread.is_alive():
                        self.land_lock_thread.join(timeout=2.0)  # 等待线程结束，最多2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.land_lock_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                self.land_lock_thread = threading.Thread(
                    target=self._land_lock_vz_t,
                    args=(vz, duration)
                )
                self.land_lock_thread.daemon = True  # 设为守护线程
                self.land_lock_thread.start()
                self.log_info(f"降落控制线程已启动: vz={vz}, 持续={duration}s")
            else:
                # 停止线程
                if self.land_lock_thread_running:
                    self.land_lock_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    self.log_info("正在停止降落控制线程...")
                    if hasattr(self, 'land_lock_thread') and self.land_lock_thread and self.land_lock_thread.is_alive():
                        self.land_lock_thread.join(timeout=2.0)  # 等待线程结束，最多2秒
                    self.log_info("降落控制线程已停止")
                else:
                    self.log_warn("没有运行中的降落控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"降落线程管理过程中发生错误: {str(e)}")
            self.land_lock_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_time_sleep(self, duration, use_thread=False):
        """
        线程管理函数，用于启动或停止延时线程
        Args:
            duration: 延迟时间（秒）
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                if not self.time_sleep_thread_running:
                    self.time_sleep_thread_running = True
                    self.stop_thread_flag.clear()  # 确保终止标志未设置
                    self.time_sleep_thread = threading.Thread(
                        target=self._time_sleep,
                        args=(duration,)
                    )
                    self.time_sleep_thread.daemon = True
                    self.time_sleep_thread.start()
                    return True
                else:
                    pass  # 如果线程已在运行，什么都不做，防止空代码块
            else:
                if self.time_sleep_thread_running:
                    self.time_sleep_thread_running = False
                    self.stop_thread_flag.set()  # 设置终止标志
                    if self.time_sleep_thread and self.time_sleep_thread.is_alive():
                        self.time_sleep_thread.join(timeout=2.0)  # 等待最多2秒
                    self.log_warn("time_sleep 线程已终止")
                else:
                    self.log_warn("time_sleep 线程未在运行")
        except Exception as e:
            self.log_err(f"time_sleep 线程函数错误: {str(e)}")
        finally:
            if not use_thread:
                self.time_sleep_thread_running = False
                self.stop_thread_flag.clear()

    def thread_send_position_x_y_z_t(self, x, y, z, duration, use_thread=False):
        """
        线程管理函数，用于启动或停止位置控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.send_position_thread_running:
                    rospy.logwarn("位置控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.send_position_thread and self.send_position_thread.is_alive():
                        self.send_position_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.send_position_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.send_position_thread = threading.Thread(
                        target=self._send_position_x_y_z_t,
                        args=(x, y, z, duration)
                    )
                    self.send_position_thread.daemon = True  # 设为守护线程
                    self.send_position_thread.start()
                    rospy.loginfo(f"位置控制线程已启动: x={x}, y={y}, z={z}, 持续={duration}s")
                except Exception as e:
                    rospy.logerr(f"启动位置控制线程失败: {str(e)}")
                    self.send_position_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.send_position_thread_running:
                    self.send_position_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止位置控制线程...")
                    if self.send_position_thread and self.send_position_thread.is_alive():
                        self.send_position_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("位置控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的位置控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.send_position_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_hover_delay_t(self, delay_s, use_thread=False):
        """
        线程管理函数，用于启动或停止悬停控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.hover_thread_running:
                    rospy.logwarn("悬停控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.hover_thread and self.hover_thread.is_alive():
                        self.hover_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.hover_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.hover_thread = threading.Thread(
                        target=self._hover_delay_t,
                        args=(delay_s,)
                    )
                    self.hover_thread.daemon = True  # 设为守护线程
                    self.hover_thread.start()
                    rospy.loginfo(f"悬停控制线程已启动: 延迟={delay_s}s")
                except Exception as e:
                    rospy.logerr(f"启动悬停控制线程失败: {str(e)}")
                    self.hover_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.hover_thread_running:
                    self.hover_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止悬停控制线程...")
                    if self.hover_thread and self.hover_thread.is_alive():
                        self.hover_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("悬停控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的悬停控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.hover_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_unlock(self, use_thread=False):
        """
        线程管理函数，用于启动或停止解锁控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.unlock_thread_running:
                    rospy.logwarn("解锁控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.unlock_thread and self.unlock_thread.is_alive():
                        self.unlock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.unlock_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.unlock_thread = threading.Thread(
                        target=self._unlock
                    )
                    self.unlock_thread.daemon = True  # 设为守护线程
                    self.unlock_thread.start()
                    rospy.loginfo("解锁控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动解锁控制线程失败: {str(e)}")
                    self.unlock_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.unlock_thread_running:
                    self.unlock_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止解锁控制线程...")
                    if self.unlock_thread and self.unlock_thread.is_alive():
                        self.unlock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("解锁控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的解锁控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.unlock_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_lock(self, use_thread=False):
        """
        线程管理函数，用于启动或停止锁定控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.lock_thread_running:
                    rospy.logwarn("锁定控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.lock_thread and self.lock_thread.is_alive():
                        self.lock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.lock_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.lock_thread = threading.Thread(
                        target=self._lock
                    )
                    self.lock_thread.daemon = True  # 设为守护线程
                    self.lock_thread.start()
                    rospy.loginfo("锁定控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动锁定控制线程失败: {str(e)}")
                    self.lock_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.lock_thread_running:
                    self.lock_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止锁定控制线程...")
                    if self.lock_thread and self.lock_thread.is_alive():
                        self.lock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("锁定控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的锁定控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.lock_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_takeoff(self, use_thread=False):
        """
        线程管理函数，用于启动或停止起飞控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.takeoff_thread_running:
                    rospy.logwarn("起飞控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.takeoff_thread and self.takeoff_thread.is_alive():
                        self.takeoff_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.takeoff_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.takeoff_thread = threading.Thread(
                        target=self._takeoff
                    )
                    self.takeoff_thread.daemon = True  # 设为守护线程
                    self.takeoff_thread.start()
                    rospy.loginfo("起飞控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动起飞控制线程失败: {str(e)}")
                    self.takeoff_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.takeoff_thread_running:
                    self.takeoff_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止起飞控制线程...")
                    if self.takeoff_thread and self.takeoff_thread.is_alive():
                        self.takeoff_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("起飞控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的起飞控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.takeoff_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_land_auto(self, use_thread=False):
        """
        线程管理函数，用于启动或停止自动降落控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.land_auto_thread_running:
                    rospy.logwarn("自动降落控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.land_auto_thread and self.land_auto_thread.is_alive():
                        self.land_auto_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.land_auto_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.land_auto_thread = threading.Thread(
                        target=self._land_auto
                    )
                    self.land_auto_thread.daemon = True  # 设为守护线程
                    self.land_auto_thread.start()
                    rospy.loginfo("自动降落控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动自动降落控制线程失败: {str(e)}")
                    self.land_auto_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.land_auto_thread_running:
                    self.land_auto_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止自动降落控制线程...")
                    if self.land_auto_thread and self.land_auto_thread.is_alive():
                        self.land_auto_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("自动降落控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的自动降落控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.land_auto_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_publish_nav_goal_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame, use_thread=False):
        """发布导航目标点线程函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            t: 时间
            frame: 坐标系
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                # 启动线程
                if self.nav_goal_thread_running:
                    rospy.logwarn("导航目标点线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.nav_goal_thread and self.nav_goal_thread.is_alive():
                        self.nav_goal_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.nav_goal_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.nav_goal_thread = threading.Thread(
                        target=self._publish_nav_goal_x_y_z_yaw_t_frame,
                        args=(x, y, z, yaw, t, frame)
                    )
                    self.nav_goal_thread.daemon = True  # 设为守护线程
                    self.nav_goal_thread.start()
                    rospy.loginfo(f"导航目标点线程已启动: x={x}, y={y}, z={z}, yaw={yaw}")
                except Exception as e:
                    rospy.logerr(f"启动导航目标点线程失败: {str(e)}")
                    self.nav_goal_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.nav_goal_thread_running:
                    self.nav_goal_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止导航目标点线程...")
                    if self.nav_goal_thread and self.nav_goal_thread.is_alive():
                        self.nav_goal_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("导航目标点线程已停止")
                else:
                    rospy.logwarn("没有运行中的导航目标点线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.nav_goal_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_publish_nav_points_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame, use_thread=False):
        """发布导航点线程函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            t: 时间
            frame: 坐标系
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                # 启动线程
                if self.nav_points_thread_running:
                    rospy.logwarn("导航点线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.nav_points_thread and self.nav_points_thread.is_alive():
                        self.nav_points_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.nav_points_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.nav_points_thread = threading.Thread(
                        target=self._publish_nav_points_x_y_z_yaw_t_frame,
                        args=(x, y, z, yaw, t, frame)
                    )
                    self.nav_points_thread.daemon = True  # 设为守护线程
                    self.nav_points_thread.start()
                    rospy.loginfo(f"导航点线程已启动: x={x}, y={y}, z={z}, yaw={yaw}")
                except Exception as e:
                    rospy.logerr(f"启动导航点线程失败: {str(e)}")
                    self.nav_points_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.nav_points_thread_running:
                    self.nav_points_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止导航点线程...")
                    if self.nav_points_thread and self.nav_points_thread.is_alive():
                        self.nav_points_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("导航点线程已停止")
                else:
                    rospy.logwarn("没有运行中的导航点线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.nav_points_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_control_yaw_z_t(self, yaw, z, t, use_thread=False):
        """
        线程管理函数，用于启动或停止偏航角控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.control_yaw_thread_running:
                    rospy.logwarn("偏航角控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.control_yaw_thread and self.control_yaw_thread.is_alive():
                        self.control_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.control_yaw_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.control_yaw_thread = threading.Thread(
                        target=self._control_yaw_z_t,
                        args=(yaw, z, t)
                    )
                    self.control_yaw_thread.daemon = True  # 设为守护线程
                    self.control_yaw_thread.start()
                    rospy.loginfo(f"偏航角控制线程已启动: yaw={yaw}, z={z}, t={t}")
                except Exception as e:
                    rospy.logerr(f"启动偏航角控制线程失败: {str(e)}")
                    self.control_yaw_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.control_yaw_thread_running:
                    self.control_yaw_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止偏航角控制线程...")
                    if self.control_yaw_thread and self.control_yaw_thread.is_alive():
                        self.control_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("偏航角控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的偏航角控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.control_yaw_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_send_velocity_vx_vy_z_t(self, vx, vy, z, duration, use_thread=False):
        if use_thread:
            if not self.velocity_thread_running:
                self.velocity_thread_running = True
                self.stop_thread_flag.clear()  # 确保终止标志未设置
                self.velocity_thread = threading.Thread(target=self._send_velocity_vx_vy_z_t, args=(vx, vy, z, duration))
                self.velocity_thread.start()
        else:
            if self.velocity_thread_running:
                
                self.velocity_thread_running = False
                if self.velocity_thread:
                    self.velocity_thread.join()  # 等待线程结束
                self.log_warn("send_velocity_vx_vy_z_t 线程已终止")
            else:
                self.log_warn("send_velocity_vx_vy_z_t 线程未在运行")


    def thread_send_position_x_y_z_t_yaw(self, x, y, z, duration, yaw, use_thread=False):
        """
        线程管理函数，用于启动或停止带偏航角的位置控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.send_position_yaw_thread_running:
                    rospy.logwarn("带偏航角的位置控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.send_position_yaw_thread and self.send_position_yaw_thread.is_alive():
                        self.send_position_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.send_position_yaw_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.send_position_yaw_thread = threading.Thread(
                        target=self._send_position_x_y_z_t_yaw,
                        args=(x, y, z, duration, yaw)
                    )
                    self.send_position_yaw_thread.daemon = True  # 设为守护线程
                    self.send_position_yaw_thread.start()
                    rospy.loginfo(f"带偏航角的位置控制线程已启动: x={x}, y={y}, z={z}, yaw={yaw}, 持续={duration}s")
                except Exception as e:
                    rospy.logerr(f"启动带偏航角的位置控制线程失败: {str(e)}")
                    self.send_position_yaw_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.send_position_yaw_thread_running:
                    self.send_position_yaw_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止带偏航角的位置控制线程...")
                    if self.send_position_yaw_thread and self.send_position_yaw_thread.is_alive():
                        self.send_position_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("带偏航角的位置控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的带偏航角的位置控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.send_position_yaw_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_circle_r_z_numpoints_t(self, r, z, numpoints, t, use_thread=False):
        """
        线程管理函数，用于启动或停止圆形轨迹控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.circle_thread_running:
                    rospy.logwarn("圆形轨迹控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.circle_thread and self.circle_thread.is_alive():
                        self.circle_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.circle_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.circle_thread = threading.Thread(
                        target=self._circle_r_z_numpoints,
                        args=(r, z, numpoints, t)
                    )
                    self.circle_thread.daemon = True  # 设为守护线程
                    self.circle_thread.start()
                    rospy.loginfo(f"圆形轨迹控制线程已启动: 半径={r}, 高度={z}, 点数={numpoints}, 时间={t}s")
                except Exception as e:
                    rospy.logerr(f"启动圆形轨迹控制线程失败: {str(e)}")
                    self.circle_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.circle_thread_running:
                    self.circle_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止圆形轨迹控制线程...")
                    if self.circle_thread and self.circle_thread.is_alive():
                        self.circle_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("圆形轨迹控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的圆形轨迹控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.circle_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def thread_track_velocity_z_t(self, z, duration, MAX_VEL=0.5, MIN_VEL=0, Kp=0.001, use_thread=False):
        """跟踪速度z轴线程函数
        
        Args:
            z: 目标z坐标
            duration: 持续时间
            MAX_VEL: 最大速度
            MIN_VEL: 最小速度
            Kp: 比例系数
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                if not self.track_velocity_thread_running:
                    self.track_velocity_thread_running = True
                    self.stop_thread_flag.clear()  # 确保终止标志未设置
                    self.thread_track_velocity_z_t_thread = threading.Thread(
                        target=self._track_velocity_z_t,
                        args=(z, duration, MAX_VEL, MIN_VEL, Kp)
                    )
                    self.thread_track_velocity_z_t_thread.start()
                    return True
            else:
                if self.track_velocity_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 track_velocity_z_t 线程")
                    self._track_velocity_z_t(z, duration, MAX_VEL, MIN_VEL, Kp)
                else:
                    self.log_warn("track_velocity_z_t 线程未在运行，无法中断")
        except Exception as e:
            self.log_err(f"track_velocity_z_t 线程函数错误: {str(e)}")
            self.track_velocity_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常


    def thread_control_points_x_y_z_stepsize(self, x, y, z, stepsize, use_thread=False):
        """点对点控制线程函数
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            stepsize: 步长
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                self.thread_control_points_x_y_z_stepsize_running = True
                self.thread_control_points_x_y_z_stepsize_stop = False
                self.thread_control_points_x_y_z_stepsize_thread = threading.Thread(
                    target=self._control_points_x_y_z_stepsize,
                    args=(x, y, z, stepsize)
                )
                self.thread_control_points_x_y_z_stepsize_thread.start()
                return True
            else:
                if self.thread_control_points_x_y_z_stepsize_running:
                    self.thread_control_points_x_y_z_stepsize_stop = True
                    self.log_warn("点对点控制线程正在运行，已设置停止标志")
                else:
                    self.thread_control_points_x_y_z_stepsize_stop = False
                    self._control_points_x_y_z_stepsize(x, y, z, stepsize)
        except Exception as e:
            self.log_err(f"点对点控制线程函数错误: {str(e)}")
        finally:
            if not use_thread:
                self.thread_control_points_x_y_z_stepsize_stop = False

    def thread_publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame, use_thread=False):
        """发布导航目标点线程函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            tolerance: 容差
            frame: 坐标系
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_running = True
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = False
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_thread = threading.Thread(
                    target=self._publish_nav_goal_x_y_z_yaw_tol_frame,
                    args=(x, y, z, yaw, tolerance, frame)
                )
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_thread.start()
                return True
            else:
                if self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_running:
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = True
                    self.log_warn("发布导航目标点线程正在运行，已设置停止标志")
                else:
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = False
                    self._publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame)
        except Exception as e:
            self.log_err(f"发布导航目标点线程函数错误: {str(e)}")
        finally:
            if not use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = False

    def thread_publish_nav_goal_x_y_z_yaw_t(self, x, y, z, yaw, t, use_thread=False):
        """发布导航目标点线程函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            t: 持续时间
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                # 启动线程
                if self.nav_goal_thread_running:
                    rospy.logwarn("导航目标点线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.nav_goal_thread and self.nav_goal_thread.is_alive():
                        self.nav_goal_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.nav_goal_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.nav_goal_thread = threading.Thread(
                        target=self._publish_nav_goal_x_y_z_yaw_t,
                        args=(x, y, z, yaw, t)
                    )
                    self.nav_goal_thread.daemon = True  # 设为守护线程
                    self.nav_goal_thread.start()
                    rospy.loginfo(f"导航目标点线程已启动: x={x}, y={y}, z={z}, yaw={yaw}, 持续={t}s")
                except Exception as e:
                    rospy.logerr(f"启动导航目标点线程失败: {str(e)}")
                    self.nav_goal_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.nav_goal_thread_running:
                    self.nav_goal_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止导航目标点线程...")
                    if self.nav_goal_thread and self.nav_goal_thread.is_alive():
                        self.nav_goal_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("导航目标点线程已停止")
                else:
                    rospy.logwarn("没有运行中的导航目标点线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.nav_goal_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常

    def _publish_nav_goal_x_y_z_yaw_t(self, x, y, z, yaw, t):
        """发布导航目标点函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            t: 持续时间
        """
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return

            if "nolog" not in self.ActionExecuteType:
                self.log_info(f"发布导航目标点: x={x}, y={y}, z={z}, yaw={yaw}, 持续={t}s")

            # 创建导航目标点消息
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z
            
            # 将偏航角转换为四元数
            q = quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            # 发布导航目标点
            start_time = rospy.Time.now()
            rate = rospy.Rate(20)  # 20Hz
            while (rospy.Time.now() - start_time).to_sec() < t:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("发布导航目标点线程被中断")
                    break
                if self.nav_goal_pub:
                    self.nav_goal_pub.publish(goal)
                rate.sleep()

            self.control_complete(True)
        elif self.ActionExecuteType == "cheak":
            print(f"检查当前页面语法模式: 发布导航目标点 (x: {x}, y: {y}, z: {z}, yaw: {yaw}, 持续: {t}s)")

    def publish_nav_points_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame, use_thread=False):
        """发布导航点序列函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            t: 持续时间
            frame: 坐标系
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                self.thread_publish_nav_points_x_y_z_yaw_t_frame(x, y, z, yaw, t, frame, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.nav_points_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断导航点序列线程")
                    self.thread_publish_nav_points_x_y_z_yaw_t_frame(x, y, z, yaw, t, frame, use_thread=False)
            
            # 直接执行导航点序列发布函数
            return self._publish_nav_points_x_y_z_yaw_t_frame(x, y, z, yaw, t, frame)
        except Exception as e:
            self.log_err(f"发布导航点序列过程中发生错误: {str(e)}")
            self.nav_points_thread_running = False
            self.control_complete(True)
            return False

    def circle_r_z_numpoints_t(self, r, z, numpoints, t, use_thread=False):
        """圆形轨迹控制函数
        
        Args:
            r: 半径
            z: 高度
            numpoints: 点数
            t: 持续时间
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                self.thread_circle_r_z_numpoints_t(r, z, numpoints, t, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.circle_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断圆形轨迹线程")
                    self.thread_circle_r_z_numpoints_t(r, z, numpoints, t, use_thread=False)
            
            # 直接执行圆形轨迹控制函数
            return self._circle_r_z_numpoints(r, z, numpoints, t)
        except Exception as e:
            self.log_err(f"圆形轨迹控制过程中发生错误: {str(e)}")
            self.circle_thread_running = False
            self.control_complete(True)
            return False

##########################################################################################################################################
#
#        
#
#
#       # ---------------用户调用区域----------------- #
#
#
#
#
#
#
#用户调用区域#########################################################################################################################################        
    def ensure_count_exists(self):
        """
        确保count属性存在，如果不存在则初始化为0
        """
        if not hasattr(self, 'count'):
            self.count = 0
        return self.count
    

    def control_servo_id_angle_t(self, servo_id, angle, duration, use_thread=False):
        """
        函数名称: control_servo_id_angle_t
        函数功能: 控制指定ID的舵机旋转到指定角度，并等待指定时间
        参数:
        - servo_id: 舵机ID号，范围1-6
        - angle: 目标角度，范围0-180度
        - duration: 等待时间，单位秒
        - use_thread: 是否使用线程方式执行，默认为False
        返回值:
        - bool: 控制是否成功
        使用案例:
        - 控制1号舵机旋转到90度，等待2秒: action.control_servo_id_angle_t(1, 90, 2)
        - 使用线程方式控制2号舵机旋转到45度，等待1秒: action.control_servo_id_angle_t(2, 45, 1, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_control_servo_id_angle_t(servo_id, angle, duration, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.control_servo_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 control_servo_id_angle_t 线程")
                    self.thread_control_servo_id_angle_t(servo_id, angle, duration, use_thread=False)
                
                # 直接执行控制函数
                return self._control_servo_id_angle_t(servo_id, angle, duration)
        except Exception as e:
            self.log_err(f"舵机控制过程中发生错误: {str(e)}")
            self.control_servo_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def unlock(self, use_thread=False):
        """
        函数名称: unlock
        函数功能: 解锁无人机，并在解锁时执行陀螺仪热初始化和OFFBOARD模式切换
        参数:
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 解锁无人机并准备进入OFFBOARD模式: action.unlock()
        - 使用线程方式解锁: action.unlock(use_thread=True)
        """
        try:
            if use_thread:
                self.thread_unlock(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.unlock_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 unlock 线程")
                    self.thread_unlock(use_thread=False)
                
                # 直接执行解锁函数
                return self._unlock()
        except Exception as e:
            self.log_err(f"解锁过程中发生错误: {str(e)}")
            self.unlock_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def stop_all_threads(self):
        """
        函数名称: stop_all_threads
        函数功能: 停止所有正在运行的线程
        使用案例:
        - 在发生错误时停止所有线程: action.stop_all_threads()
        """
        try:
            # 设置终止标志
            self.stop_thread_flag.set()
            
            # 停止所有线程
            thread_list = [
                (self.send_position_thread_running, self.send_position_thread, "位置控制"),
                (self.hover_thread_running, self.hover_thread, "悬停控制"),
                (self.unlock_thread_running, self.unlock_thread, "解锁控制"),
                (self.lock_thread_running, self.lock_thread, "锁定控制"),
                (self.takeoff_thread_running, self.takeoff_thread, "起飞控制"),
                (self.land_auto_thread_running, self.land_auto_thread, "自动降落"),
                (self.nav_goal_thread_running, self.nav_goal_thread, "导航控制"),
                (self.control_yaw_thread_running, self.control_yaw_thread, "偏航角控制"),
                (self.velocity_thread_running, self.velocity_thread, "速度控制"),
                (self.land_lock_thread_running, self.land_lock_thread, "降落锁定"),
                (self.control_points_thread_running, self.control_points_thread, "点控制"),
                (self.position_yaw_thread_running, self.position_yaw_thread, "位置偏航"),
                (self.circle_thread_running, self.circle_thread, "圆形轨迹"),
                (self.track_velocity_thread_running, self.track_velocity_thread, "速度追踪"),
                (self.time_sleep_thread_running, self.time_sleep_thread, "延时"),
                (self.tracking_thread_running, self.tracking_thread, "追踪"),
                (self.control_servo_thread_running, self.servo_control_thread, "舵机控制")
            ]
            
            for running_flag, thread, name in thread_list:
                if running_flag and thread and thread.is_alive():
                    self.log_warn(f"正在停止{name}线程...")
                    thread.join(timeout=2.0)  # 等待最多2秒
                    self.log_warn(f"{name}线程已停止")
            
            # 重置所有运行标志
            self.send_position_thread_running = False
            self.hover_thread_running = False
            self.unlock_thread_running = False
            self.lock_thread_running = False
            self.takeoff_thread_running = False
            self.land_auto_thread_running = False
            self.nav_goal_thread_running = False
            self.control_yaw_thread_running = False
            self.velocity_thread_running = False
            self.land_lock_thread_running = False
            self.control_points_thread_running = False
            self.position_yaw_thread_running = False
            self.circle_thread_running = False
            self.track_velocity_thread_running = False
            self.time_sleep_thread_running = False
            self.tracking_thread_running = False
            self.control_servo_thread_running = False
            
            # 清除终止标志
            self.stop_thread_flag.clear()
            
            # 设置控制完成标志
            self.control_complete(True)
            
            self.log_info("所有线程已停止")
        except Exception as e:
            self.log_err(f"停止线程过程中发生错误: {str(e)}")
            raise

    def send_position_x_y_z_t(self, x, y, z, duration, use_thread=False):
        """
        函数名称: send_position_x_y_z_t
        函数功能: 在指定时间内将无人机移动到指定位置
        参数:
        - x: 目标位置X坐标
        - y: 目标位置Y坐标
        - z: 目标位置Z坐标
        - duration: 移动持续时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 移动到(0, 0, 5)位置，持续10秒: action.send_position_x_y_z_t(0, 0, 5, 10)
        - 使用线程方式移动: action.send_position_x_y_z_t(0, 0, 5, 10, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_send_position_x_y_z_t(x, y, z, duration, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.send_position_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 send_position_x_y_z_t 线程")
                    self.thread_send_position_x_y_z_t(x, y, z, duration, use_thread=False)
                
                # 直接执行位置控制函数
                return self._send_position_x_y_z_t(x, y, z, duration)
        except Exception as e:
            self.log_err(f"位置控制过程中发生错误: {str(e)}")
            self.send_position_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def takeoff(self, height=1.2, use_thread=False):
        """
        函数名称: takeoff
        函数功能: 控制无人机起飞到指定高度
        参数:
        - height: 目标高度，单位米，默认为1.2米
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 起飞到1.2米高度: action.takeoff()
        - 起飞到指定高度: action.takeoff(2.0)
        - 使用线程方式起飞: action.takeoff(1.5, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_takeoff(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.takeoff_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 takeoff 线程")
                    self.thread_takeoff(use_thread=False)
                
                # 直接执行起飞函数
                return self._takeoff(height)
        except Exception as e:
            self.log_err(f"起飞过程中发生错误: {str(e)}")
            self.takeoff_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def lock(self, use_thread=False):
        """
        函数名称: lock
        函数功能: 锁定无人机（上锁）
        参数:
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 锁定无人机: action.lock()
        - 使用线程方式锁定: action.lock(use_thread=True)
        """
        try:
            if use_thread:
                self.thread_lock(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.lock_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 lock 线程")
                    self.thread_lock(use_thread=False)
                
                # 直接执行锁定函数
                return self._lock()
        except Exception as e:
            self.log_err(f"锁定过程中发生错误: {str(e)}")
            self.lock_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def land_auto(self, use_thread=False):
        """
        函数名称: land_auto
        函数功能: 控制无人机自动降落
        参数:
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 自动降落: action.land_auto()
        - 使用线程方式自动降落: action.land_auto(use_thread=True)
        """
        try:
            if use_thread:
                self.thread_land_auto(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.land_auto_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 land_auto 线程")
                    self.thread_land_auto(use_thread=False)
                
                # 直接执行自动降落函数
                return self._land_auto()
        except Exception as e:
            self.log_err(f"自动降落过程中发生错误: {str(e)}")
            self.land_auto_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def hover_delay_t(self, delay_s, use_thread=False):
        """
        函数名称: hover_delay_t
        函数功能: 在当前位置悬停指定时间
        参数:
        - delay_s: 悬停时间，单位秒
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 悬停5秒: action.hover_delay_t(5)
        - 使用线程方式悬停: action.hover_delay_t(5, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_hover_delay_t(delay_s, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.hover_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 hover_delay_t 线程")
                    self.thread_hover_delay_t(delay_s, use_thread=False)
                
                # 直接执行悬停函数
                return self._hover_delay_t(delay_s)
        except Exception as e:
            self.log_err(f"悬停过程中发生错误: {str(e)}")
            self.hover_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def send_position_x_y_z_t_yaw(self, x, y, z, duration, yaw, use_thread=False):
        """
        函数名称: send_position_x_y_z_t_yaw
        函数功能: 在指定时间内将无人机移动到指定位置和偏航角
        参数:
        - x: 目标位置X坐标
        - y: 目标位置Y坐标
        - z: 目标位置Z坐标
        - duration: 移动持续时间（秒）
        - yaw: 目标偏航角（弧度）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 移动到(0, 0, 5)位置，偏航角为0，持续10秒: action.send_position_x_y_z_t_yaw(0, 0, 5, 10, 0)
        - 使用线程方式移动: action.send_position_x_y_z_t_yaw(0, 0, 5, 10, 0, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_send_position_x_y_z_t_yaw(x, y, z, duration, yaw, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.position_yaw_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 send_position_x_y_z_t_yaw 线程")
                    self.thread_send_position_x_y_z_t_yaw(x, y, z, duration, yaw, use_thread=False)
                
                # 直接执行位置和偏航角控制函数
                return self._send_position_x_y_z_t_yaw(x, y, z, yaw, duration)
        except Exception as e:
            self.log_err(f"位置和偏航角控制过程中发生错误: {str(e)}")
            self.position_yaw_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def land_lock_vz_t(self, vz, duration, use_thread=False):
        """
        函数名称: land_lock_vz_t
        函数功能: 以指定速度降落指定时间
        参数:
        - vz: 降落速度，单位米/秒（负值表示下降）
        - duration: 降落持续时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 以-0.4m/s的速度降落10秒: action.land_lock_vz_t(-0.4, 10)
        - 使用线程方式降落: action.land_lock_vz_t(-0.4, 10, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_land_lock_vz_t(vz, duration, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.land_lock_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 land_lock_vz_t 线程")
                    self.thread_land_lock_vz_t(vz, duration, use_thread=False)
                
                # 直接执行降落函数
                return self._land_lock_vz_t(vz, duration)
        except Exception as e:
            self.log_err(f"降落过程中发生错误: {str(e)}")
            self.land_lock_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame, use_thread=False):
        """
        主控制函数，用于管理带容差的导航控制过程
        """
        try:
            if use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=True)
                if self.control_complete():
                    self.stop_thread_flag.set()
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=False)
            else:
                if self.nav_goal_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 publish_nav_goal_x_y_z_yaw_tol_frame 线程")
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=False)
                else:
                    self.log_warn("publish_nav_goal_x_y_z_yaw_tol_frame 线程未在运行，无法中断")
        except Exception as e:
            self.log_err(f"导航控制过程中发生错误: {str(e)}")
            self.nav_goal_thread_running = False
            self.control_complete(True)
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志
                
    def send_velocity_vx_vy_z_t(self, vx, vy, z, duration, use_thread=False):
        """
        函数名称: send_velocity_vx_vy_z_t
        函数功能: 控制无人机以指定速度飞行指定时间
        参数:
        - vx: x方向速度（米/秒）
        - vy: y方向速度（米/秒）
        - z: 目标高度（米）
        - duration: 持续时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 以0.5m/s的速度向前飞行10秒: action.send_velocity_vx_vy_z_t(0.5, 0, 1.5, 10)
        - 使用线程方式: action.send_velocity_vx_vy_z_t(0.5, 0, 1.5, 10, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_send_velocity_vx_vy_z_t(vx, vy, z, duration, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.send_velocity_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 send_velocity_vx_vy_z_t 线程")
                    self.thread_send_velocity_vx_vy_z_t(vx, vy, z, duration, use_thread=False)
                
                # 直接执行速度控制函数
                return self._send_velocity_vx_vy_z_t(vx, vy, z, duration)
        except Exception as e:
            self.log_err(f"速度控制过程中发生错误: {str(e)}")
            self.send_velocity_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def control_yaw_z_t(self, yaw, z, t, use_thread=False):
        """
        函数名称: control_yaw_z_t
        函数功能: 控制无人机的偏航角和高度
        参数:
        - yaw: 目标偏航角（弧度）
        - z: 目标高度（米）
        - t: 持续时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 控制偏航角和高度: action.control_yaw_z_t(1.57, 1.5, 10)  # 旋转90度并保持1.5米高度10秒
        - 使用线程方式: action.control_yaw_z_t(1.57, 1.5, 10, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_control_yaw_z_t(yaw, z, t, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.control_yaw_z_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 control_yaw_z_t 线程")
                    self.thread_control_yaw_z_t(yaw, z, t, use_thread=False)
                
                # 直接执行偏航角和高度控制函数
                return self._control_yaw_z_t(yaw, z, t)
        except Exception as e:
            self.log_err(f"偏航角和高度控制过程中发生错误: {str(e)}")
            self.control_yaw_z_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def track_velocity_z_t(self, z, duration, MAX_VEL=0.5, MIN_VEL=0, Kp=0.001, use_thread=False):
        """
        函数名称: track_velocity_z_t
        函数功能: 控制无人机追踪指定高度
        参数:
        - z: 目标高度（米）
        - duration: 持续时间（秒）
        - MAX_VEL: 最大速度限制（米/秒）
        - MIN_VEL: 最小速度限制（米/秒）
        - Kp: 比例系数
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 追踪1.5米高度: action.track_velocity_z_t(1.5, 10)
        - 使用线程方式: action.track_velocity_z_t(1.5, 10, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_track_velocity_z_t(z, duration, MAX_VEL, MIN_VEL, Kp, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.track_velocity_z_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 track_velocity_z_t 线程")
                    self.thread_track_velocity_z_t(z, duration, MAX_VEL, MIN_VEL, Kp, use_thread=False)
                
                # 直接执行高度追踪函数
                return self._track_velocity_z_t(z, duration, MAX_VEL, MIN_VEL, Kp)
        except Exception as e:
            self.log_err(f"高度追踪过程中发生错误: {str(e)}")
            self.track_velocity_z_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def track_velocity_z_centertol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, use_thread=False):
        """
        函数名称: track_velocity_z_centertol
        函数功能: 控制无人机追踪指定高度，带中心点容差
        参数:
        - z: 目标高度（米）
        - MAX_VEL: 最大速度限制（米/秒）
        - MIN_VEL: 最小速度限制（米/秒）
        - Kp: 比例系数
        - center_tolerance: 中心点容差（像素）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 追踪1.5米高度: action.track_velocity_z_centertol(1.5)
        - 使用线程方式: action.track_velocity_z_centertol(1.5, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_track_velocity_z_centertol(z, MAX_VEL, MIN_VEL, Kp, center_tolerance, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.track_velocity_z_centertol_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 track_velocity_z_centertol 线程")
                    self.thread_track_velocity_z_centertol(z, MAX_VEL, MIN_VEL, Kp, center_tolerance, use_thread=False)
                
                # 直接执行高度追踪函数
                return self._track_velocity_z_centertol(z, MAX_VEL, MIN_VEL, Kp, center_tolerance)
        except Exception as e:
            self.log_err(f"高度追踪过程中发生错误: {str(e)}")
            self.track_velocity_z_centertol_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def track_velocity_z_centertol_ttol(self, z, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, timeout=3, use_thread=False):
        """
        函数名称: track_velocity_z_centertol_ttol
        函数功能: 控制无人机追踪指定高度，带中心点容差和超时限制
        参数:
        - z: 目标高度（米）
        - MAX_VEL: 最大速度限制（米/秒）
        - MIN_VEL: 最小速度限制（米/秒）
        - Kp: 比例系数
        - center_tolerance: 中心点容差（像素）
        - timeout: 超时时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 追踪1.5米高度: action.track_velocity_z_centertol_ttol(1.5)
        - 使用线程方式: action.track_velocity_z_centertol_ttol(1.5, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_track_velocity_z_centertol_ttol(z, MAX_VEL, MIN_VEL, Kp, center_tolerance, timeout, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.track_velocity_z_centertol_ttol_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 track_velocity_z_centertol_ttol 线程")
                    self.thread_track_velocity_z_centertol_ttol(z, MAX_VEL, MIN_VEL, Kp, center_tolerance, timeout, use_thread=False)
                
                # 直接执行高度追踪函数
                return self._track_velocity_z_centertol_ttol(z, MAX_VEL, MIN_VEL, Kp, center_tolerance, timeout)
        except Exception as e:
            self.log_err(f"高度追踪过程中发生错误: {str(e)}")
            self.track_velocity_z_centertol_ttol_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def control_points_x_y_z_stepsize(self, x, y, z, stepsize, use_thread=False):
        """
        主控制函数，用于管理点对点控制过程
        """
        try:
            if use_thread:
                self.thread_control_points_x_y_z_stepsize(x, y, z, stepsize, use_thread=True)
                if self.control_complete():
                    self.stop_thread_flag.set()
                    self.thread_control_points_x_y_z_stepsize(x, y, z, stepsize, use_thread=False)
            else:
                if self.control_points_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 control_points_x_y_z_stepsize 线程")
                    self.thread_control_points_x_y_z_stepsize(x, y, z, stepsize, use_thread=False)
                else:
                    self.log_warn("control_points_x_y_z_stepsize 线程未在运行，无法中断")
        except Exception as e:
            self.log_err(f"点对点控制过程中发生错误: {str(e)}")
            self.control_points_thread_running = False
            self.control_complete(True)
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志






