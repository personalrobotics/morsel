#!/usr/bin/env python

# Python libs
import math, sys, json
import numpy as np
import cv2
import rospy
import argparse

# Ros Messages
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

# bite detector
import bitefinder

class AdaBiteServer(object):
    def __init__(self, options = {}):
        """ Create a bite server.

        Options:
        verbose: whether to print verbose debug information
        ransac_iters: iterations of ransac for plane finding
        ransac_thresh: inlier threshold (depth units) for plane finding
        max_bites: maximum number of bites to find
        bite_height: how tall (depth units) a bite must be
        mask_filename: filename of plate mask to use
        decimate: int N such that only process 1 out of every N frames
        downscale_factor: downscale depth image (e.g., 0.5 = half size)
        save_depth_images: true if you want to save depth images to disk
        depth_image_path: path to save depth images to
        depth_image_format: either png or jpg
        node_name: what the ros node should be named
        hfov: horizontal fov of depth camera (degrees)
        vfov: vertical fov of depth camera (degrees)
        hsize: horizontal size of depth image (pixels) before downscale
        vsize: vertical size of depth image (pixels) before downscale
        (for bite finder):
        kernel_size: size in pixels of the bite-finding kernel
        bite_radius: fraction of kernel occupied by the bite
        border_sigma: how 'fuzzy' the bite border is (larger = fuzzier)
        debug: write debug images to disk
        """
        self.VERBOSE = options.get("verbose", False)
        self.finder = bitefinder.BiteFinder(options)
        self.ransac_iters = options.get("ransac_iters", 10)
        self.ransac_thresh = options.get("ransac_thresh", 0.01)  # 1cm thresh?
        self.plane_coeffs = [0.0, 0.0, 0.0]
        self.max_bites = options.get("max_bites", 10)
        self.bite_height = options.get("bite_height", 0.01)
        self.json_pub = None
        self.ros_pub = None
        self.maskfn = options.get("mask_filename", "mask.png")
        self._prepare_mask()
        self.decimate = options.get("decimate", 5)
        self.fcount = 0
        self.downscale_factor = options.get("downscale_factor", 0.5)
        self.save_depth_images = options.get("save_depth_images", False)
        self.depth_image_path = options.get("depth_image_path", "depth_images/")
        self.depth_image_format = options.get("depth_image_format", "png")
        self.nodename = options.get("node_name", "adabitefinder")

        self.set_intrinsics_from_fov(deg_to_rad(options.get("hfov", 58)),
                                     deg_to_rad(options.get("vfov", 45)),
                                     options.get("hsize", 640.0),
                                     options.get("vsize", 480.0))

    def _prepare_mask(self):
        self.raw_mask = cv2.imread("config/{}".format(self.maskfn))
        layers = cv2.split(self.raw_mask)
        if len(layers) != 0:
            self.mask = np.zeros(layers[0].shape, dtype=np.float32)
            self.mask[layers[0] > 128] = 1.0
            print("Loaded mask")
        else:
            self.mask = None
            print("Error loading mask file {}".format(self.maskfn))

    def _decode_uncompressed_f32(self, data):
        if self.VERBOSE:
            print("Data has encoding: %s" % data.encoding)
        rows = data.height
        cols = data.step / 4  # assume 32 bit depth
        temp = np.fromstring(data.data, dtype=np.float32)
        temp = np.nan_to_num(temp)
        temp = temp.reshape(rows, cols)
        if self.mask is not None:
            if temp.shape == self.mask.shape:
                temp = temp * self.mask
            else:
                print("Mask does not match shape of image!")
                print("Ignoring mask")
                self.mask = None

        if self.downscale_factor < 1.0:
            cols = int(cols * self.downscale_factor)
            rows = int(rows * self.downscale_factor)
            temp = cv2.resize(temp, (cols, rows))

        if self.VERBOSE:
            print("rows: %d" % rows)
            print("cols: %d" % cols)
            print("max: %g" % np.max(temp))
            print("min: %g" % np.min(temp))

        if self.save_depth_images:
            fn = "{}frame_{}.{}".format(self.depth_image_path,
                                       self.fcount,
                                       self.depth_image_format)
            if self.depth_image_format == "png" or self.depth_image_format == "jpg":
                img = bitefinder.squash_depth(temp)
                color_img = cv2.merge([img, img, img])
                cv2.imwrite(fn, color_img)
            elif self.depth_image_format == "npy":
                np.save(fn, temp)
            else:
                print("Unknown depth image format: {}".format(self.depth_image_format))
                self.save_depth_images = False

        return temp

    def set_intrinsics_from_fov(self, hfov, vfov, rwidth, rheight):
        """ Set camera intrinsics from horizontal and vertical field of views.

        @param hfov: horizontal field of view (radians)
        @param vfov: vertical field of view (radians)
        @param rwidth: expected image width (pixels)
        @param rheight: expected image height (pixels)
        """

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
        """ Set camera intrinsics directly from a matrix.

        @param K: 3x3 intrinsics matrix
        @param zero_centered: whether the intrinsics matrix has (0,0) as center
        """
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
        """ Process a depth image to find and publish bites. """
        # first, fit a plane with ransac and get the residuals
        best_coeffs, num_inliers, residuals = bitefinder.ransac_plane(img,
                                                          self.ransac_iters,
                                                          self.ransac_thresh,
                                                          self.plane_coeffs)
        if self.VERBOSE:
            print("Number of inliers: {}".format(num_inliers))
        self.plane_coeffs = best_coeffs

        if num_inliers == 0 or residuals is None:
            if self.VERBOSE:
                print("No plane found.")
            return

        # find bite-sized things in the residuals
        bites = self.finder.find_bites(residuals,
                                       self.max_bites,
                                       -(self.bite_height))

        # estimate bite depths from plane fit
        bite_positions = [self._assign_bite_depth(b[0], best_coeffs)
                          for b in bites]

        # compute projections and publish results
        pts_3d = self.project_points(bite_positions)
        self._publish_bites(best_coeffs, bites, pts_3d)

    def _publish_bites(self, plane_coeffs, bites, bites3d):
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

    def _callback_depth(self, data):
        self.fcount += 1
        if(self.fcount % self.decimate != 0):
            return

        img_base = self._decode_uncompressed_f32(data)
        self.process_depth(img_base)

    def start_listening(self, depth_topic, json_pub_topic, ros_pub_topic):
        """ Start serving bites.

        @param depth_topic: the topic name to get depth images from
        @param json_pub_topic: the topic name to publish JSON string bites to
        @param ros_pub_topic: the topic name to publish a pose array to
        """
        rospy.init_node(self.nodename)

        self.depth_sub = rospy.Subscriber(depth_topic, Image,
                                          self._callback_depth, queue_size=1)

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

def load_options(optlist):
    opts = {}
    for fn in optlist:
        with open("config/{}".format(fn), "rt") as src:
            temp_opts = json.load(src)
            opts.update(temp_opts)
    if opts.get("verbose", False):
        print("Options: {}".format(opts))
    return opts

def do_main():
    # need to shove this into a function so we can use 'return'
    parser = argparse.ArgumentParser()
    parser.add_argument("--configfile", help="json config file(s) to use",
                        nargs="+")
    parser.add_argument("--configpath", help="ros param server path to use")
    args = parser.parse_args()

    if args.configfile:
        opts = load_options(args.configfile)
    elif args.configpath:
        opts = rospy.get_param(args.configpath)
    else:
        print("Need to provide either --configfile or --configpath!")
        return

    frame_listener = AdaBiteServer(opts)

    test_depth_image = opts.get("test_image", "")
    if test_depth_image != "":
        print("Running a test image...")
        image_base = np.load(test_depth_image)
        frame_listener.process_depth(image_base)
        print("Finished running test.")
    else:
        depth_topic = opts.get("depth_topic", "/camera/depth/image")
        morsel_topic = opts.get("morsel_topic", "/perception/morsel_detection")
        pose_topic = opts.get("pose_topic", "/perception/morsel_pose")

        # enter main listening loop
        frame_listener.start_listening(depth_topic, morsel_topic, pose_topic)
        rospy.spin()


if __name__ == '__main__':
    do_main()