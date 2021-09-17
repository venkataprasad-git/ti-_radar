'''boson robotics radar based obstacle detection based on the 180 degrees FOV
and works only with 2d point cloud cfg file '''

import math
import ros_numpy
import rospy
from sensor_msgs.msg import PointCloud2 
from std_msgs.msg import Bool
import numpy as np


class RadarObjDetct:
    def __init__(self,x_min,x_max,y_min,y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        sub = rospy.Subscriber("/ti_mmwave/radar_scan_pcl", PointCloud2, self.radar_callback)
        self.radar_pub = rospy.Publisher('radar_obj_status',Bool,queue_size=10)


    def radar_callback(self, pc2_msg):
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
        fil=[]
        for i,(x,y,z) in enumerate(xyz_array):
            if x > self.x_min and self.y_min < y < self.y_max:
                fil.append(xyz_array[i])
        val = 0
        for x,y,z in fil:
            if self.x_min < x < self.x_max:
                val +=1
        bool_status = Bool()
        if val >0 :
            bool_status =  True
        else:
            bool_status =  False
        self.radar_pub.publish(bool_status)

    def run(self):
        rospy.spin()

if __name__=="__main__":
    rospy.init_node('rdar')
    x_min = rospy.get_param('radar/x_min',1)
    x_max = rospy.get_param('radar/x_max',4)
    y_min = rospy.get_param('radar/y_min',-1.5)
    y_max = rospy.get_param('radar/y_max',1.5)


    radar_obj = RadarObjDetct(x_min, x_max, y_min, y_max)
    radar_obj.run()
