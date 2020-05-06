#!/usr/bin/env python3



import rospy
from std_msgs.msg import Header
from std_msgs.msg import Int32, Bool
from std_msgs.msg import String
from mavros_msgs import msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, PoseStamped
from grid_map_msgs.msg import GridMap
import csv
from datetime import datetime
import os, time, sys
import _thread
import numpy, ros_numpy

def deleteFileIfExists(filename):
    if os.path.exists(filename):
        os.remove(filename)
    else:
        print("The file does not exist")

def find(name, path):
    for root, dirs, files in os.walk(path):
        if name in files or name in dirs:
            return os.path.join(root, name)

def determinePathToRecording():
    user=os.getenv("HOME")
    recloc = find("recordings", user)
    if recloc is None:
        os.mkdir(user+"/recordings")
    recpath = recloc+"/"
    return recpath


class periodicrecorder(object):
    world_state = {}
    store_file_name ="rcrecorderDefault.csv"
    store_point_cloud = "rcrpointCloudDefault.gz"
    currentPointCloud= None
    #PointCloud2()

    def VelodynePointCloudCB(self,msg):
        rospy.logdebug("I was called here")
        data = ros_numpy.numpify(msg)
        self.currentPointCloud = data
        #   type(stamped_pose)
        #self.world_state['PCloudFile'] = interesting_data

    def VehiclePositionCB(self,stamped_pose):
        rospy.logdebug("I was called here")
        interesting_data = stamped_pose.pose.pose
        #   type(stamped_pose)
        self.world_state['VehiclePosition'] = interesting_data
        rospy.logdebug('position is:' + str(interesting_data))

    def ShovelPositionCB(self, stamped_pose):
        interesting_data = stamped_pose.pose.pose
        self.world_state['ShovelPosition'] = interesting_data
        rospy.logdebug('ShovelPosition is:' + str(interesting_data))

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

    def GridMapCB(self, data):
        gridmap = data.data
        gridmap2 = gridmap[1]
        self.world_state['GridMap'] = gridmap2
        rospy.logdebug('GridMap is:' + str(gridmap2))

    def JoyActionSubCB(self, data):
        joydata = data.axes
        self.world_state['JoyActions'] = joydata

    def SimPositionCB(self,pose):
        interesting_data = pose
        self.world_state['SimulationPosition'] = interesting_data
        rospy.logdebug('position is:' + str(interesting_data))

    def RCOverrideActionSubCB(self, data):
        rcoverrideData = data.channels
        rospy.logdebug("RCOverrideActionSubCB " + rcoverrideData.__str__())
        self.world_state['RCActions'] = rcoverrideData



    def SaveData(self):
        identifier = datetime.now().strftime("%d-%m-H%H:%M:%S.%f")[:-3]+"-"+self.store_number.__str__()
        self.world_state['Date&Time'] = identifier
        if self.currentPointCloud != None:
            self.store_point_cloud = "PointCloud"+identifier+".gzip"
            fullpath = self.store_rec_path+"/"+self.store_point_cloud
            xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.currentPointCloud)
            numpy.savez_compressed(fullpath, xyz_array)
            self.world_state['PCloudFile'] = self.store_point_cloud
        self.world_state['Grade'] = "N/A"
        # print("joydata" + joydata.__str__())
        with open(self.store_file_name, 'a', newline='') as csvfile:
            fieldnames = ['Date&Time', 'RCActions','JoyActions', 'VehiclePosition', 'ShovelPosition', 'ArmHeight',
                          'ArmShortHeight', 'BladeImu',
                          'VehicleImu', 'GridMap', 'SimulationPosition', 'PCloudFile', 'Grade']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow(self.world_state)
        self.store_number = self.store_number + 1

    def startBagRecord(self, command):
        os.system(command)

    def __init__(self, freq=10):

        if (len(sys.argv)>1):
            freq = int(sys.argv[1])

        rospy.init_node('periodicrecorder', anonymous=False)
        # rospy.init_node('rcrecorder', anonymous=False,log_level=rospy.DEBUG)
        # Define Subscribers
        self.vehiclePositionSub = rospy.Subscriber('/sl_pose', PoseWithCovarianceStamped, self.VehiclePositionCB)
        self.shovelPositionSub = rospy.Subscriber('/shovel_pose', PoseWithCovarianceStamped, self.ShovelPositionCB)
        self.heightSub = rospy.Subscriber('/arm/height', Int32, self.ArmHeightCB)
        self.heightSub = rospy.Subscriber('/arm/shortHeight', Int32, self.ArmShortHeightCB)
        self.bladeImuSub = rospy.Subscriber('/arm/blade/Imu', Imu, self.BladeImuCB)
        self.vehicleImuSub = rospy.Subscriber('/mavros/imu/data', Imu, self.VehicleImuCB)
        self.simPositionSub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.SimPositionCB)

        self.joyActionSub = rospy.Subscriber('/joy', Joy, self.JoyActionSubCB)
        self.gridMapSub = rospy.Subscriber('/sl_map', GridMap, self.GridMapCB)
        self.pointCloud = rospy.Subscriber('/velodyne_points', PointCloud2, self.VelodynePointCloudCB)

        self.rcOverrideActionSub = rospy.Subscriber('/mavros/rc/override', msg.OverrideRCIn, self.RCOverrideActionSubCB)

        self.store_number = 1
        identifier = "rec"+datetime.now().strftime("%d-%m-H%H:%M:%S")
        path = determinePathToRecording()+ identifier + "/"
        self.store_rec_path = path
        os.mkdir(self.store_rec_path)
        self.store_file_name = self.store_rec_path+identifier+".csv"
        bagfilename = self.store_rec_path+identifier+".bag"
        bagrecordcommand = "rosbag record --duration=2h --node=/periodicrecorder --bz2 -O "+bagfilename

        with open(self.store_file_name, 'w', newline='') as csvfile:
            fieldnames = ['Date&Time', 'RCActions','JoyActions', 'VehiclePosition', 'ShovelPosition','ArmHeight',
                          'ArmShortHeight', 'BladeImu',
                          'VehicleImu', 'GridMap', 'SimulationPosition', 'PCloudFile', 'Grade']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
        try:
            _thread.start_new_thread(self.startBagRecord, (bagrecordcommand,) )
        except:
            print("Error: unable to start bagfile")
     #   startbagprocess.start()
#        rospy.spin()
        time.sleep(3)
        r = rospy.Rate(freq)  # 10hz
        while not rospy.is_shutdown():
            self.SaveData()
            r.sleep()

if __name__ == '__main__':
    node = periodicrecorder()

