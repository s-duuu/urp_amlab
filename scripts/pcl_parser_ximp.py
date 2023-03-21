#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
import math

sys.path.append('/home/ximp/catkin_ws/src/urp_amlab/scripts/lidar_processing')

import pcl_helper

#ROI setting
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    segmenter.set_max_iterations(1000)
    segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)

    return indices, inliers, outliers

#noise filtering
def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k: number of neighboring points to analyze for any given point
    :param tresh: Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    : https://github.com/fouliex/RoboticPerception
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)    
    print("Input :", cloud, type(cloud))
    
    # 실행 코드 부분 - filtering by axis
    filter_axis = 'x'
    axis_min = 1.0
    axis_max = 20.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -7.0
    axis_max = 5.5
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    _, _, cloud = do_ransac_plane_normal_segmentation(cloud, 0.05)

    # 실행 코드 부분 - noise filtering
    cloud = pcl_helper.XYZRGB_to_XYZ(cloud)

    mean_k = 10
    tresh = 0.001
    cloud = do_statistical_outlier_filtering(cloud,mean_k,tresh)

    color = pcl_helper.random_color_gen()
    cloud = pcl_helper.XYZ_to_XYZRGB(cloud,color)
    print("Output :", cloud, type(cloud))
    print("")

    #PCL을 ROS 메시지로 변경
    cloud_new = pcl_helper.pcl_to_ros(cloud)      
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('ximp_trial', anonymous=True)
    rospy.Subscriber('/pointcloud/os1_pc2', PointCloud2, callback)
    pub = rospy.Publisher("/ximp_pcl", PointCloud2, queue_size=1)

    rospy.spin()



