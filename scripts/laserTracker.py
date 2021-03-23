#!/usr/bin/env python
# coding=utf-8
import rospy
import thread
import threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from simple_follower.msg import position as PositionMsg


# laserTracker类定义
class laserTracker:
    # 类属性初始化
    def __init__(self):
        self.lastScan = None
        self.winSize = rospy.get_param('~winSize')      # winSize   :   窗口尺寸（单边）
        self.deltaDist = rospy.get_param('~deltaDist')  # deltaDist :   Δx
        self.lidar_type = rospy.get_param('~lidarType') # lidarType :   雷达类型-XAS
        # scanSubscriber订阅激光雷达/scan数据
        self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        # positionPublisher发布/object_tracker/current_position跟随物体位置信息（角度和距离）
        self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=3)
        # infoPublisher发布ROS系统中断信息
        self.infoPublisher = rospy.Publisher('/object_tracker/info', StringMsg, queue_size=3)

    # scanSubscriber订阅器回调函数
    def registerScan(self, scan_data):

        # 将雷达距离数据按照从小到大排序
        ranges = np.array(scan_data.ranges)
        sortedIndices = np.argsort(ranges)
        minDistanceID = None
        minDistance = float('inf')

        # 将当前的激光雷达数据与上一次激光雷达数据做对比，判断是不是噪声
        if (not (self.lastScan is None)):
            for i in sortedIndices:
                tempMinDistance = ranges[i]

                # 以当前位置开一个宽度为2*winSize+1的窗口
                windowIndex = np.clip([i - self.winSize, i + self.winSize + 1], 0, len(self.lastScan))
                window = self.lastScan[windowIndex[0]:windowIndex[1]]

                with np.errstate(invalid='ignore'):
                    # 检查窗口中的任何扫描（上一次扫描）是否与当前扫描距离足够近
                    if (np.any(abs(window - tempMinDistance) <= self.deltaDist)):
                        # 满足条件，说明当前距离处不是噪声，找到最近物体


                        # 此处可添加人腿特征提取函数


                        minDistanceID = i
                        minDistance = ranges[minDistanceID]
                        break

        self.lastScan = ranges

        if (minDistance > scan_data.range_max):
            # 发布警告，没有找到合适的物体
            rospy.logwarn('laser no object found')
            self.infoPublisher.publish(StringMsg('laser:nothing found'))

        else:
            # 本机激光雷达型号为XAS，此处是为了适配不同型号的激光雷达
            if (self.lidar_type == 'XAS'):
                minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
            elif (self.lidar_type == 'XT1'):
                minDistanceAngle = - (scan_data.angle_min + (minDistanceID - 250) * scan_data.angle_increment)
            elif (self.lidar_type == 'YT1'):
                minDistanceAngle = - (scan_data.angle_min + (minDistanceID - 500) * scan_data.angle_increment)
            # 发布跟随物体的位置信息（包含角度和距离）
            # 这里我们只有一个x角，所以y是任意设置的
            self.positionPublisher.publish(PositionMsg(minDistanceAngle, 42, minDistance))


if __name__ == '__main__':
    # 提示laser_tracker节点开始运行
    print('laser_tracker starting...')
    # 初始化laser_tracker节点
    rospy.init_node('laser_tracker')
    # 实例化laserTracker
    tracker = laserTracker()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('ROSInterruptException')
