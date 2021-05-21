#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys

from numpy.lib.function_base import piecewise
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError

import tf_conversions
import tf

import pcl_ros

import struct

import numpy as np

from sensor_msgs.msg import PointCloud2
import message_filters
import time

import os

# TODO: Add transform

LABELS = { 
  "0" : "unlabeled",
  "1" : "outlier",
  "10": "car",
  "11": "bicycle",
  "13": "bus",
  "15": "motorcycle",
  "16": "on-rails",
  "18": "truck",
  "20": "other-vehicle",
  "30": "person",
  "31": "bicyclist",
  "32": "motorcyclist",
  "40": "road",
  "44": "parking",
  "48": "sidewalk",
  "49": "other-ground",
  "50": "building",
  "51": "fence",
  "52": "other-structure",
  "60": "lane-marking",
  "70": "vegetation",
  "71": "trunk",
  "72": "terrain",
  "80": "pole",
  "81": "traffic-sign",
  "99": "other-object",
  "252": "moving-car",
  "253": "moving-bicyclist",
  "254": "moving-person",
  "255": "moving-motorcyclist",
  "256": "moving-on-rails",
  "257": "moving-bus",
  "258": "moving-truck",
  "259": "moving-other-vehicle"}

COLOR_MAP = { # bgr
  "0" : [0, 0, 0],
  "1" : [0, 0, 255],
  "10": [245, 150, 100],
  "11": [245, 230, 100],
  "13": [250, 80, 100],
  "15": [150, 60, 30],
  "16": [255, 0, 0],
  "18": [180, 30, 80],
  "20": [255, 0, 0],
  "30": [30, 30, 255],
  "31": [200, 40, 255],
  "32": [90, 30, 150],
  "40": [255, 0, 255],
  "44": [255, 150, 255],
  "48": [75, 0, 75],
  "49": [75, 0, 175],
  "50": [0, 200, 255],
  "51": [50, 120, 255],
  "52": [0, 150, 255],
  "60": [170, 255, 150],
  "70": [0, 175, 0],
  "71": [0, 60, 135],
  "72": [80, 240, 150],
  "80": [150, 240, 255],
  "81": [0, 0, 255],
  "99": [255, 255, 50],
  "252": [245, 150, 100],
  "256": [255, 0, 0],
  "253": [200, 40, 255],
  "254": [30, 30, 255],
  "255": [90, 30, 150],
  "257": [250, 80, 100],
  "258": [180, 30, 80],
  "259": [255, 0, 0]}

def color_map(N=256, normalized=False):
    """
    Return Color Map in PASCAL VOC format (rgb)
    \param N (int) number of classes
    \param normalized (bool) whether colors are normalized (float 0-1)
    \return (Nx3 numpy array) a color map
    """
    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)
    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7-j)
            g = g | (bitget(c, 1) << 7-j)
            b = b | (bitget(c, 2) << 7-j)
            c = c >> 3
        cmap[i] = np.array([r, g, b])
    cmap = cmap/255.0 if normalized else cmap
    return cmap


class SemanticGen:
    def __init__(self, classes=34, H=64, W=1024):

        #rospy.init_node('sem_gen')

        self.num_classes = classes
        self.frame_id = "odom"
        
        self.width = W
        self.height = H

        self.colors = color_map(classes) 



    def read_kitti_PCD(self, filename):
        """ Open raw scan and fill in attributes
        """
        # reset just in case there was an open structure
        # self.reset()

        # check filename is string
        if not isinstance(filename, str):
            raise TypeError("Filename should be string type, "
                            "but was {type}".format(type=str(type(filename))))

        # check extension is a laserscan
        if not any(filename.endswith(ext) for ext in ['bin']):
            raise RuntimeError("Filename extension is not valid scan file.")

        # if all goes well, open pointcloud
        scan = np.fromfile(filename, dtype=np.float32)
        scan = scan.reshape((-1, 4))

        return scan

        #print(scan)

        # put in attribute
        #points = scan[:, 0:3]    # get xyz
        #remissions = scan[:, 3]  # get remission // Intensity?

        #return self.generate_cloud(scan[:, 0:4], use_kitti=True)

    
    def read_label(self, filename):
        """ Open raw scan and fill in attributes
        """
        # check filename is string
        if not isinstance(filename, str):
            raise TypeError("Filename should be string type, "
                        "but was {type}".format(type=str(type(filename))))

        # check extension is a laserscan
        if not any(filename.endswith(ext) for ext in [".label"]):
            raise RuntimeError("Filename extension is not valid label file.")

        # if all goes well, open label
        label = np.fromfile(filename, dtype=np.uint32)
        label = label.reshape((-1))
        self.set_label(label)    

    def set_label(self, label):
        """ Set points for label not from file but from np
        """
        # check label makes sense
        if not isinstance(label, np.ndarray):
            raise TypeError("Label should be numpy array")

        # only fill in attribute if the right size
        if True:#label.shape[0] == self.points.shape[0]:
            self.sem_label = label & 0xFFFF  # semantic label in lower half
            self.inst_label = label >> 16    # instance id in upper half
        else:
            print("Points shape: ", self.points.shape)
            print("Label shape: ", label.shape)
            raise ValueError("Scan and Label don't contain same number of points")

        # sanity check
        assert((self.sem_label + (self.inst_label << 16) == label).all())


    def sort_confidence(self):
        """Sort labels by confidence"""
        pass


    def generate_cloud(self, cloud, label, use_kitti=False):

        fields = []
        fields.append(PointField(
             name = "x",
             offset = 0,
             datatype = PointField.FLOAT32, count = 1))
        fields.append(PointField(
                            name = "y",
                            offset = 4,
                            datatype = PointField.FLOAT32, count = 1))
        fields.append(PointField(
             name = "z",
             offset = 8,
             datatype = PointField.FLOAT32, count = 1))
        fields.append(PointField(
             name = "intensity",
             offset = 12,
             datatype = PointField.FLOAT32, count = 1))
        fields.append(PointField(
             name = "semantic_color",
             offset = 16,
             datatype = PointField.FLOAT32, count = 1))
        fields.append(PointField(
                            name = "confidence",
                            offset = 24,
                            datatype = PointField.FLOAT32, count = 1))

        if use_kitti:
            pass

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id


        for i in range(len(self.sem_label)):
            b,g,r = COLOR_MAP[str(self.sem_label[i])]
            fl_color = (r<<16) | (g<<8) | b            
            self.sem_label[i] = fl_color

        scan_data = np.zeros((cloud.shape[0],6))
        scan_data[:,0:4] = cloud
        scan_data[:,4] = self.sem_label
        scan_data[:,5] = np.ones(self.sem_label.shape)*0.6

        cl = point_cloud2.create_cloud(header, fields, scan_data)

        return cl

    def parse_calibration(self, filename):
        """ read calibration file with given filename
            Returns
            -------
            dict
                Calibration matrices as 4x4 numpy arrays.
        """
        calib = {}

        calib_file = open(filename)
        for line in calib_file:
            key, content = line.strip().split(":")
            values = [float(v) for v in content.strip().split()]

            pose = np.zeros((4, 4))
            pose[0, 0:4] = values[0:4]
            pose[1, 0:4] = values[4:8]
            pose[2, 0:4] = values[8:12]
            pose[3, 3] = 1.0

            calib[key] = pose

        calib_file.close()

        self.calib = calib

    def parse_pose(self,line):
        """ read poses file with per-scan poses from given filename
            Returns
            -------
            list
                list of poses as 4x4 numpy arrays.
        """
        Tr = self.calib["Tr"]
        Tr_inv = np.linalg.inv(Tr)
        values = [float(v) for v in line.strip().split()]

        pose = np.zeros((4, 4))
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        return np.matmul(Tr_inv, np.matmul(pose, Tr))


if __name__=="__main__":
    rospy.init_node('sem_gen', anonymous=True)

    n = SemanticGen()
    print("Reading bin...")

    file_dir = "/home/abbas/Downloads/06/"

    bin_files = os.listdir(file_dir+"velodyne/")
    labels = os.listdir(file_dir+"labels/")
    bin_files.sort()
    labels.sort()

    poses_file = open(file_dir+"poses.txt")
    poses = poses_file.readlines()

    calibration = n.parse_calibration(file_dir+"calib.txt")


    cloud_pub = rospy.Publisher("cloud_out", PointCloud2, queue_size = 5)
    while cloud_pub.get_num_connections() < 1:
        print("waiting...")
        time.sleep(1)
    
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    loop_rate = rospy.Rate(10)

    for p,l,s in zip(poses,labels,bin_files):
        pose = n.parse_pose(p)

        t = tf.transformations.translation_from_matrix(pose)
        q = tf.transformations.quaternion_from_matrix(pose)

        transform = tf.TransformerROS().fromTranslationRotation(t,q)
        
        br.sendTransform(t,q,rospy.Time.now(), "odom","world")

        scan = n.read_kitti_PCD(file_dir+"/velodyne/"+s)
        label = n.read_label(file_dir+"/labels/"+l)
        

        cl = n.generate_cloud(scan, label)

        cloud_pub.publish(cl)
        
        if rospy.is_shutdown():
            break

        loop_rate.sleep()

    rospy.spin()
