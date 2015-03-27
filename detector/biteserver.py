#!/usr/bin/env python

# Python libs
import sys, time, math

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String

# json and base64
import base64
import json

# bite detector
import bitefinder

class AdaBiteServer:

    def __init__(self):
        self.VERBOSE = True
        self.finder = bitefinder.BiteFinder(debug=self.VERBOSE)
        self.ransac_iters = 200
        self.ransac_thresh = 0.01 # 1cm thresh?
        self.max_bites = 10
        self.bite_height = 0.01
        self.pub = None
        self.maskfn = "mask.png"
        self.prepare_mask()
        self.decimate = 30
        self.fcount = 0

    def prepare_mask(self):
    	self.raw_mask = cv2.imread(self.maskfn)
    	layers = cv2.split(self.raw_mask)
    	self.mask = np.zeros(layers[0].shape, dtype=np.float32)
    	self.mask[layers[0] > 128] = 1.0
    	print("Loaded mask")

    def decode_uncompressed_f32(self, data):
        if self.VERBOSE:
            print("Data has encoding: %s" % data.encoding)
        rows = data.height
        cols = data.step / 4 # assume 32 bit depth
        temp = np.fromstring(data.data, dtype=np.float32)
        temp = np.nan_to_num(temp)
        temp = temp.reshape(rows, cols)
        temp = temp * self.mask
        if self.VERBOSE:
            print("rows: %d" % rows)
            print("cols: %d" % cols)
            print("max: %g" % np.max(temp))
            print("min: %g" % np.min(temp))

        return temp

    def set_intrinsics_from_fov(self, hfov, vfov, width, height):
        # wlog say image plane at distance 1, then
        # image_half_width/1 = tan(hfov)
        # image_half_height/1 = tan(vfov)
        # image_half_height = image_half_width / aspect
        cx = width / 2.0
        cy = height / 2.0
        fx = (width / 2.0) / math.tan(hfov / 2.0) 
        fy = (height / 2.0) / math.tan(vfov / 2.0)
        self._projmat = np.array([[ fx, 0.0,  cx],
                                  [0.0,  fy,  cy],
                                  [0.0, 0.0, 1.0]])
        self._inv_projmat = np.linalg.inv(self._projmat)
        print(self._inv_projmat)

    def set_intrinsics(self, K, zero_centered = True):
        self._K = K.copy()
        if zero_centered and self._normMat is not None:
            self._projmat = self._normMat.dot(K)
        else:
            self._projmat = K.copy()
        self._inv_projmat = np.linalg.inv(self._projmat)
        print(self._inv_projmat)

    def _proj_pt(self, pt):
        img_pt = np.array([pt[1], pt[0], 1.0])
        tf_pt = self._inv_projmat.dot(img_pt)
        mult = pt[2] / tf_pt[2]
        tf_pt *= mult
        return list(tf_pt)

    def project_points(self, pts):
        return [self._proj_pt(pt) for pt in pts]

    def _assign_bite_depth(self, pt, coeffs):
        return (pt[0], pt[1], pt[0]*coeffs[0] + pt[1]*coeffs[1] + coeffs[2])

    def process_depth(self, img):
    	self.fcount += 1
    	if(self.fcount % self.decimate != 0):
    		return

        # first, fit a plane with ransac and get the residuals
        best_coeffs, num_inliers, residuals = bitefinder.ransac_plane(img,
                                                self.ransac_iters, 
                                                self.ransac_thresh)


        if(num_inliers == 0 or residuals is None):
        	if(self.VERBOSE):
        		print("No plane found.")
        	return

        # find bite-sized things in the residuals
        bites = self.finder.find_bites(residuals, 
                                       self.max_bites,
                                       -(self.bite_height))

        # estimate bite depths from plane fit
        bite_positions = [self._assign_bite_depth(b[0], best_coeffs) 
                            for b in bites]

        # compute projections
        pts_3d = self.project_points(bite_positions)

        self.publish_bites(best_coeffs, bites, pts_3d)

    def publish_bites(self, plane_coeffs, bites, bites3d):
        jdata = {}
        jdata["plane"] = list(plane_coeffs)
        jdata["bites"] = bites
        jdata["pts3d"] = bites3d

        strdata = json.dumps(jdata)

        if self.VERBOSE:
            print("data: " + strdata)

        if self.pub:
            self.pub.publish(strdata)

    def callback_depth(self, data):
        img_base = self.decode_uncompressed_f32(data)
        self.process_depth(img_base)
        
    def start_listening(self, depth_topic, pub_topic):
        rospy.init_node('adabitefinder')

        self.depth_sub = rospy.Subscriber(depth_topic, Image, 
                                          self.callback_depth, queue_size = 1)

        self.pub = rospy.Publisher(pub_topic, String)

def deg_to_rad(d):
    return math.pi * (d / 180.0)

if __name__ == '__main__':

    import sys

    frame_listener = AdaBiteServer()
    frame_listener.set_intrinsics_from_fov(deg_to_rad(70.95), deg_to_rad(55.00), 
                                            320.0, 240.0)
    frame_listener.start_listening( "/softkinetic_driver/depth/image",
                                    "/perception/morsel_detection")

    # keep ros going
    rospy.spin()
