#!/usr/bin/env python3



import rospy
from std_msgs.msg import Header
from std_msgs.msg import Int32, Bool
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
import csv
from datetime import datetime


class OpRecorder(object):
    MAX_NUM_STONES = 10

    world_state = {}

    def VehiclePositionCB(self,stamped_pose):
        rospy.logdebug("I was called here")
        type(stamped_pose)
        #   pose = stamped_pose.pose
        #   position = pose.position
        #   rospy.loginfo("I was called...")
        self.world_state['VehiclePosition'] = stamped_pose

        rospy.loginfo('position is:' + str(stamped_pose))

    def VehicleVelocityCB(self, stamped_twist):
        twist = stamped_twist.twist
        self.world_state['VehicleVelocity'] = stamped_twist
        rospy.logdebug('velocity is:' + str(twist))

    def ArmHeightCB(self, data):
        height = data.data
        self.world_state['ArmHeight'] = height
        rospy.logdebug('arm height is:' + str(height))

    def BladeImuCB(self, imu):
        orientation = imu.orientation
        angular_velocity = imu.angular_velocity
        linear_acceleration = imu.linear_acceleration
        self.world_state['BladeImu'] = imu
        rospy.logdebug('blade imu is:' + str(imu))

    def VehicleImuCB(self, imu):
        orientation = imu.orientation
        angular_velocity = imu.angular_velocity
        linear_acceleration = imu.linear_acceleration
        self.world_state['VehicleImu'] = imu
        rospy.logdebug('vehicle imu is:' + str(imu))

    def StonePositionCB(self, data, arg):
        position = data.pose
        stone = arg
        self.world_state['StonePosition' + str(stone)] = position
        rospy.logdebug('stone ' + str(stone) + ' position is:' + str(position))

    def StoneIsLoadedCB(self, data, arg):
        question = data.data
        stone = arg
        self.world_state['StoneIsLoaded' + str(stone)] = question
        rospy.logdebug('Is stone ' + str(stone) + ' loaded? ' + str(question))

    def JoyActionSubCB(self, data):
        joydata = data.axes

        self.world_state['Date&Time'] = datetime.now().strftime("%d-%m,%H:%M:%S")
        self.world_state['Buttons'] = joydata
        self.world_state['Grade'] = "N/A"
        # print("joydata" + joydata.__str__())
        rospy.logdebug("actions" + joydata.__str__())
        with open('oprecorder.csv', 'a', newline='') as csvfile:
            fieldnames = ['Date&Time', 'Buttons', 'VehiclePosition', 'VehicleVelocity', 'ArmHeight', 'BladeImu',
                          'VehicleImu', 'StonePosition1', 'StonePosition2', 'Grade']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow(self.world_state)

    def __init__(self):
        rospy.init_node('oprecorder', anonymous=False)
        # rospy.init_node('slagent', anonymous=False,log_level=rospy.DEBUG)

        # Define Subscribers
        self.vehiclePositionSub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.VehiclePositionCB)
        self.vehicleVelocitySub = rospy.Subscriber('mavros/local_position/velocity', TwistStamped, self.VehicleVelocityCB)
        self.heightSub = rospy.Subscriber('arm/height', Int32, self.ArmHeightCB)
        self.bladeImuSub = rospy.Subscriber('arm/blade/Imu', Imu, self.BladeImuCB)
        self.vehicleImuSub = rospy.Subscriber('mavros/imu/data', Imu, self.VehicleImuCB)

        self.stoneIsLoadedSubList = []
        self.stonePoseSubList = []
        for i in range(self.MAX_NUM_STONES):
            topicName = 'stone/' + str(i) + '/Pose'
            self.stonePoseSubList.append(rospy.Subscriber(topicName, PoseStamped, self.StonePositionCB, i))
            # stonePoseSub[i] = rospy.Subscriber(topicName, PoseStamped, StonePositionCB, i)
            topicName = 'stone/' + str(i) + '/IsLoaded'
            self.stoneIsLoadedSubList.append(rospy.Subscriber(topicName, Bool, self.StoneIsLoadedCB, i))

        self.joyActionSub = rospy.Subscriber('/joy', Joy, self.JoyActionSubCB)
        with open('oprecorder.csv', 'w', newline='') as csvfile:
            fieldnames = ['Date&Time', 'Buttons', 'VehiclePosition', 'VehicleVelocity','ArmHeight', 'BladeImu',
                          'VehicleImu', 'StonePosition1', 'StonePosition2', 'Grade']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

        rospy.spin()

if __name__ == '__main__':
    node = OpRecorder()
