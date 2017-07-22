#!/usr/bin/env python
import argparse
import tools.find_mxnet
import mxnet as mx
import os
import importlib
import sys
from detect.camdector import Detector

import rospy

CLASSES = ('aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')

def get_detector(net, prefix, epoch, data_shape, mean_pixels, ctx,
                 nms_thresh=0.5, force_nms=True):
    
    sys.path.append(os.path.join(sys.path[1], 'symbol'))

    net = importlib.import_module("symbol_" + net) \
            .get_symbol(len(CLASSES), nms_thresh, force_nms)
    detector = Detector(net, prefix + "_" + str(data_shape), epoch, \
        data_shape, mean_pixels, ctx=ctx)
    return detector


if __name__ == '__main__':

    ctx = mx.gpu(0)
    network = 'vgg16_ssd_300'
    # (os.environ["ROS_PACKAGE_PATH"])
    # sys.path.insert(0, os.path.join("/home/wolfram/mxnet/python"))
    # print("The path1: {} ".format(os.environ["ROS_PACKAGE_PATH"]))
    prefix = os.path.join(sys.path[1], 'model', 'ssd')
    # print("The path2: {} ".format(sys.path[0]))
    epoch = 0
    data_shape = 300
    mean_r = 125
    mean_g = 117
    mean_b = 104
    nms_thresh = 0.5
    force_nms = True
    _dir = ''
    _extension = ''
    thresh = 0.71
    show_timer = True

    rospy.init_node('ros_mxnet_ssd', anonymous=True)

    # parse image list
    # image_list = ["data/demo/dog.jpg"]
    # assert len(image_list) > 0, "No valid image specified to detect"

    detector = get_detector(network, prefix, epoch,
                            data_shape,
                            (mean_r, mean_g, mean_b),
                            ctx, nms_thresh, force_nms)
    # run detection
    detector.detect_and_visualize(_dir, _extension,
                                  CLASSES, thresh, show_timer)
