#!/usr/bin/env python3



import rospy
from std_msgs.msg import Header
from std_msgs.msg import Int32, Bool
from std_msgs.msg import String
from mavros_msgs import msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from grid_map_msgs.msg import GridMap
import csv
from datetime import datetime


class RcRecorder(object):
    MAX_NUM_STONES = 10

    world_state = {}


    def VehiclePositionCB(self,stamped_pose):
        rospy.logdebug("I was called here")
        interesting_data = stamped_pose.pose.pose
        #   type(stamped_pose)
        self.world_state['VehiclePosition'] = interesting_data
        rospy.logdebug('position is:' + str(interesting_data))

    def ShovelPositionCB(self, stamped_pose):
        interesting_data = stamped_pose.pose.pose
        self.world_state['ShovelPosition'] = interesting_data
        rospy.logdebug('velocity is:' + str(interesting_data))

    def ArmHeightCB(self, data):
        height = data.data
        self.world_state['ArmHeight'] = height
        rospy.logdebug('arm height is:' + str(height))

    def ArmShortHeightCB(self, data):
        height = data.data
        self.world_state['ArmShortHeight'] = height
        rospy.logdebug('arm short height is:' + str(height))

    def BladeImuCB(self, imu):
        short_imu = {}
        short_imu['orientation'] = imu.orientation
        short_imu['angular_velocity'] = imu.angular_velocity
        short_imu['linear_acceleration'] = imu.linear_acceleration
        self.world_state['BladeImu'] = short_imu
        rospy.logdebug('blade imu is:' + str(short_imu))

    def VehicleImuCB(self, imu):
        short_imu = {}
        short_imu['orientation'] = imu.orientation
        short_imu['angular_velocity'] = imu.angular_velocity
        short_imu['linear_acceleration'] = imu.linear_acceleration
        self.world_state['VehicleImu'] = short_imu
        rospy.logdebug('vehicle imu is:' + str(short_imu))

    def MapCB(self, data):
        gridmap = data
        self.world_state['GridMap'] = gridmap
        rospy.logdebug('GridMap is:' + str(gridmap))

    def JoyActionSubCB(self, data):
        joydata = data.axes
        self.world_state['JoyActions'] = joydata

    def RCOverrideActionSubCB(self, data):
        rcoverrideData = data.channels

        self.world_state['Date&Time'] = datetime.now().strftime("%d-%m,%H:%M:%S")
        self.world_state['RCActions'] = rcoverrideData
        self.world_state['Grade'] = "N/A"
        # print("joydata" + joydata.__str__())
        rospy.logdebug("actions" + rcoverrideData.__str__())
        with open('rcrecorder.csv', 'a', newline='') as csvfile:
            fieldnames = ['Date&Time', 'RCActions','JoyActions', 'VehiclePosition', 'VehicleVelocity', 'ArmHeight',
                          'ArmShortHeight', 'BladeImu',
                          'VehicleImu', 'GridMap', 'Grade']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow(self.world_state)

    def __init__(self):
        rospy.init_node('rcrecorder', anonymous=False)
        # rospy.init_node('slagent', anonymous=False,log_level=rospy.DEBUG)
        # Define Subscribers
        self.vehiclePositionSub = rospy.Subscriber('/sl_pose', PoseWithCovarianceStamped, self.VehiclePositionCB)
        self.shovelPositionSub = rospy.Subscriber('/shovel_pose', PoseWithCovarianceStamped, self.ShovelPositionCB)
        self.heightSub = rospy.Subscriber('/arm/height', Int32, self.ArmHeightCB)
        self.heightSub = rospy.Subscriber('/arm/shortHeight', Int32, self.ArmShortHeightCB)
        self.bladeImuSub = rospy.Subscriber('/arm/blade/Imu', Imu, self.BladeImuCB)
        self.vehicleImuSub = rospy.Subscriber('/mavros/imu/data', Imu, self.VehicleImuCB)
        self.joyActionSub = rospy.Subscriber('/joy', Joy, self.JoyActionSubCB)
        self.gridMapSub = rospy.Subscriber('/sl_map', GridMap, self.MapCB)

        self.rcOverrideActionSub = rospy.Subscriber('/mavros/rc/override', msg.OverrideRCIn, self.RCOverrideActionSubCB)
        with open('rcrecorder.csv', 'w', newline='') as csvfile:
            fieldnames = ['Date&Time', 'RCActions','JoyActions', 'VehiclePosition', 'ShovelPosition','ArmHeight', 'ArmShortHeight', 'BladeImu',
                          'VehicleImu', 'GridMap', 'Grade']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

        rospy.spin()

if __name__ == '__main__':
    node = RcRecorder()
