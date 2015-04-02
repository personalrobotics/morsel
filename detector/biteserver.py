#!/usr/bin/env python

# Python libs
import math

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import rospy

# Ros Messages
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

# json
import json

# bite detector
import bitefinder


class AdaBiteServer(object):

    def __init__(self):
        self.VERBOSE = True
        self.DEBUG_IMAGES = True
        self.finder = bitefinder.BiteFinder(debug=self.DEBUG_IMAGES)
        self.ransac_iters = 200
        self.ransac_thresh = 0.01  # 1cm thresh?
        self.max_bites = 10
        self.bite_height = 0.01
        self.json_pub = None
        self.ros_pub = None
        self.maskfn = "mask.png"
        self.prepare_mask()
        self.decimate = 30
        self.fcount = 0
        self.downscale_factor = 0.5

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
        cols = data.step / 4  # assume 32 bit depth
        temp = np.fromstring(data.data, dtype=np.float32)
        temp = np.nan_to_num(temp)
        temp = temp.reshape(rows, cols)
        temp = temp * self.mask

        if self.downscale_factor < 1.0:
            cols = int(cols * self.downscale_factor)
            rows = int(rows * self.downscale_factor)
            temp = cv2.resize(temp, (cols, rows))

        if self.VERBOSE:
            print("rows: %d" % rows)
            print("cols: %d" % cols)
            print("max: %g" % np.max(temp))
            print("min: %g" % np.min(temp))

        return temp

    def set_intrinsics_from_fov(self, hfov, vfov, rwidth, rheight):
        # canonical image plane at distance 1, then
        # image_half_width/1 = tan(hfov)
        # image_half_height/1 = tan(vfov)
        # image_half_height = image_half_width / aspect
        width = rwidth * self.downscale_factor
        height = rheight * self.downscale_factor

        cx = width / 2.0
        cy = height / 2.0
        fx = (width / 2.0) / math.tan(hfov / 2.0)
        fy = (height / 2.0) / math.tan(vfov / 2.0)
        self._projmat = np.array([[fx, 0.0,  cx],
                                  [0.0,  fy,  cy],
                                  [0.0, 0.0, 1.0]])
        self._inv_projmat = np.linalg.inv(self._projmat)
        print(self._inv_projmat)

    def set_intrinsics(self, K, zero_centered=True):
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
        return (pt[0], pt[1], pt[0] * coeffs[0] + pt[1] * coeffs[1] + coeffs[2])

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

        if self.json_pub:
            self.json_pub.publish(strdata)

        if self.ros_pub:
            rosdata = PoseArray()
            rosdata.poses = [point_to_pose(p) for p in bites3d]

            self.ros_pub.publish(rosdata)

    def callback_depth(self, data):
        img_base = self.decode_uncompressed_f32(data)
        self.process_depth(img_base)

    def start_listening(self, depth_topic, json_pub_topic, ros_pub_topic):
        rospy.init_node('adabitefinder')

        self.depth_sub = rospy.Subscriber(depth_topic, Image,
                                          self.callback_depth, queue_size=1)

        self.json_pub = rospy.Publisher(json_pub_topic, String)
        self.ros_pub = rospy.Publisher(ros_pub_topic, PoseArray)


def point_to_pose(p):
    ret = Pose()
    ret.position.x = p[0]
    ret.position.y = p[1]
    ret.position.z = p[2]
    ret.orientation.w = 1.0  # make sure quat is normalized
    return ret


def deg_to_rad(d):
    return math.pi * (d / 180.0)

if __name__ == '__main__':

    frame_listener = AdaBiteServer()
    frame_listener.set_intrinsics_from_fov(deg_to_rad(58), deg_to_rad(45),
                                           640.0, 480.0)
    frame_listener.start_listening("/camera/depth/image",
                                   "/perception/morsel_detection",
                                   "/perception/morsel_pts")

    # keep ros going
    rospy.spin()
