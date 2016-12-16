import numpy as np
import random
import cv2


def swap_xy(t):
    """ Return a tuple with the first two elements swapped. """
    return (t[1], t[0])

def sanitize_numpy_int(t):
    """ Turn the input into a normal list of ints.

    Numpy has a habit of returning strange types.
    """
    return [int(v) for v in t]

def count_inliers(row_idx, col_idx, image, coeffs, inlier_thresh, valid_mask):
    """ Count how many pixels in a depth image are 'close' to a plane.

    @param row_idx: Numpy array of row coordinates (e.g., from meshgrid)
    @param col_idx: Numpy array of column coordinates (e.g., from meshgrid)
    @param image: Depth image (should be same shape as row_idx and col_idx)
    @param coeffs: list [A,B,C] of plane coefficients, z = Ax + By + C
    @param inlier_thresh: distance a depth pixel can be from plane to be inlier
    @param valid_mask: binary image (1=valid) of area to count
    """
    pmat = (coeffs[0] * row_idx) + (coeffs[1] * col_idx) + coeffs[2]
    residuals = image - pmat
    num_inliers = np.sum((np.abs(residuals) < inlier_thresh) * valid_mask)
    return num_inliers, residuals

def ransac_plane(img, niter, inlier_thresh, initial_coeffs = None):
    """ Fit a plane to a depth image using ransac.

    @param img: depth image
    @param niter: how many iterations of ransac to run
    @param inlier_thresh: how far from the plane a pixel can be to be an inlier
    @param initial_coeffs: optional initial plane coefficients Ax+By+C
    """

    if initial_coeffs is not None:
        best_coeffs = np.array(initial_coeffs)
    else:
        best_coeffs = np.array([0.0, 0.0, 0.0])
    most_inliers = 0
    best_residuals = None
    valid_mask = (img > 0.0)

    nrows = img.shape[0]
    ncols = img.shape[1]

    # warning: this might use a lot of memory depending on your image
    sample_locations = [np.unravel_index(v, valid_mask.shape)
                        for v in np.nditer(np.where(valid_mask.ravel()))]

    def rand_samp():
        return random.choice(sample_locations)

    if(len(sample_locations) == 0):
        return (best_coeffs, 0, None)

    samps = [(rand_samp(), rand_samp(), rand_samp())
             for i in range(niter)]

    col_idx, row_idx = np.meshgrid(range(ncols), range(nrows))

    # warm start with given coefficients
    if initial_coeffs is not None:
        most_inliers, best_residuals = count_inliers(row_idx, col_idx, img,
                                                     best_coeffs, inlier_thresh,
                                                     valid_mask)
    for (s1, s2, s3) in samps:
        A = np.array([[s1[0], s1[1], 1.0],
                      [s2[0], s2[1], 1.0],
                      [s3[0], s3[1], 1.0]])
        b = np.array([img[s1], img[s2], img[s3]])
        try:
            x = np.linalg.solve(A, b)
        except np.linalg.LinAlgError as e:
            # singular matrix
            continue

        num_inliers, residuals = count_inliers(row_idx, col_idx, img, x,
                                               inlier_thresh, valid_mask)

        if num_inliers > most_inliers or best_residuals is None:
            most_inliers = num_inliers
            best_residuals = residuals
            best_coeffs = x

    best_residuals[np.logical_not(valid_mask)] = 0.0
    return (best_coeffs, most_inliers, best_residuals)

class PlateFinder(object):
    def __init__(self, options={}):
        """ Create a PlateFinder

        Options can have:
        rim_width: expected plate rim thickness in pixels
        rim_margin: what margin around the rim to ignore
        min_radius: the smallest plate radius (pixels) to search for
        max_radius: the largest plate radius to search for
        radius_steps: how many intermediate radiuses to search
        filled: whether to search for the flat interior of the plate as well
        debug: enable debug output
        plate_margin: how much to expand the generated mask, in pixels
        """
        self._rim_width = options.get("rim_width", 4)
        self._rim_margin = options.get("rim_margin", 4)
        self._min_rad = options.get("min_radius", 25)
        self._max_rad = options.get("max_radius", 100)
        self._rad_steps = options.get("radius_steps", 30)
        self._filled = options.get("filled", True)
        self._debug = options.get("debug", True)
        self._plate_margin = options.get("plate_margin", 4)
        self._build_kernels()

    def _build_kernel(self, radius):
        b = radius + self._rim_width
        x, y = np.meshgrid(np.linspace(-b, b, int(b*2)),
                           np.linspace(-b, b, int(b*2)))
        rad = (x ** 2 + y ** 2) ** 0.5

        kern = np.ones(rad.shape, dtype=np.float32) * -1
        if not self._filled:
            kern[rad < radius] = 0.0
        else:
            kern[rad < radius] = -500.0 / (radius ** 2.0)
        kern[(rad >= (radius - self._rim_width / 2.0)) * (rad <= (radius + self._rim_width / 2.0))] = 1.0
        kern[rad >= (radius + self._rim_margin)] = 0.0

        return kern

    def _build_kernels(self):
        self._kernels = []
        for (idx, radius) in enumerate(np.linspace(self._min_rad, self._max_rad, self._rad_steps)):
            k = self._build_kernel(radius)
            self._kernels.append((radius, k))
            if self._debug:
                colkern = colorize_kernel(k, 255.0)
                cv2.imwrite("kernels/kernel_{}.png".format(idx), colkern)

    def _find_plate(self, image, k_idx):
        rad, k = self._kernels[k_idx]
        plateness = cv2.filter2D(image, -1, k) / np.sum(np.abs(k))

        bpos = np.unravel_index(np.argmax(plateness), plateness.shape)
        bval = plateness[bpos]

        if self._debug:
            pimg = colorize_kernel(plateness, 255.0)
            cv2.imwrite("plateness/plateness_{}.png".format(k_idx), pimg)
            print("k: {}, v: {}".format(k_idx, bval))

        return (bpos, rad, bval)

    def build_plate_mask(self, image, thresh):
        """ Create a plate mask (binary image) from a depth image.

        @param image: input depth image.
        @param thresh: Not really sure what this does TBH.
        """
        thresh_image = create_signed_thresh(image, thresh)
        if self._debug:
            dimg = colorize_kernel(thresh_image, 255.0)
            cv2.imwrite("plate_thresh.png", dimg)
        best_val = 0.0
        best_pos = (0,0)
        best_rad = 0.0
        for idx in range(len(self._kernels)):
            pos, rad, val = self._find_plate(thresh_image, idx)
            if val > best_val:
                best_val = val
                best_pos = pos
                best_rad = rad
        print("Res: {}".format((best_pos, best_rad, best_val)))
        mask = np.zeros(thresh_image.shape, dtype=np.uint8)
        cv2.circle(mask, swap_xy(best_pos), int(best_rad - self._plate_margin), 255, -1)
        return mask


class BiteFinder(object):
    def __init__(self, options = {}):
        """ Create a BiteFinder

        Options can include:
        kernel_size: size in pixels of the bite-finding kernel
        bite_radius: fraction of kernel occupied by the bite
        border_sigma: how 'fuzzy' the bite border is (larger = fuzzier)
        """
        self._kernel_size = options.get("kernel_size", 33)
        self._bite_radius = options.get("bite_radius", 0.6)
        self._border_sigma = options.get("border_sigma", 0.2)
        self._quality_thresh = 0.2 * (self._kernel_size ** 2.0)
        self._build_kernel()
        self._debug = options.get("debug", False)

    def _build_kernel(self):
        x, y = np.meshgrid(np.linspace(-1.0, 1.0, self._kernel_size),
                           np.linspace(-1.0, 1.0, self._kernel_size))
        rad = (x ** 2 + y ** 2) ** 0.5

        # create the smoothly varying border
        dist_from_border = np.maximum(rad - self._bite_radius, 0.0)
        gauss = np.exp(-0.5 * (dist_from_border ** 2.0) /
                       (self._border_sigma ** 2.0))
        gauss *= -1.0  # area outside of border should be negative

        # fill in center area with +1.0
        gauss[rad <= self._bite_radius] = 1.0
        self._kernel = gauss

    def _raw_find_bites(self, image, n):
        valid_map = np.ones(image.shape)
        bite_quality = cv2.filter2D(image, -1, self._kernel)
        self._last_bite_quality = bite_quality
        ret = []
        px_bite_rad = int(self._kernel_size * self._bite_radius * 0.5)

        for i in range(n):
            bpos = np.unravel_index(np.argmax(bite_quality * valid_map),
                                    bite_quality.shape)
            bval = bite_quality[bpos]
            if bval <= self._quality_thresh: # no more bites to find
                break
            ret.append((sanitize_numpy_int(bpos), px_bite_rad, bval))
            cv2.circle(valid_map, swap_xy(bpos), px_bite_rad*2, 0.0, -1)

        return ret

    def find_bites(self, image, n, thresh=0.0):
        """ Find 'bites' in a depth image.

        @param image: the depth image
        @param n: maximum number of bites to find
        @param thresh: how 'bitelike' a position has to be to count as a bite
        """
        if len(image.shape) > 2:
            b, g, r = cv2.split(image)
            image = r
        thresh_image = np.array(image, dtype=np.float64)
        # print("min: %g" % np.min(thresh_image))
        # print("max: %g" % np.max(thresh_image))
        old_image = thresh_image.copy()
        #old_image[old_image < 1.0] = 255.0
        thresh_image[old_image >= thresh] = -1.0
        thresh_image[old_image < thresh] = 1.0
        bites = self._raw_find_bites(thresh_image, n)
        if self._debug:
            cv2.imwrite("thresh.png", colorize_kernel(thresh_image, 255.0))
            self._debug_image = debug_draw_bites(
                colorize_kernel(image, 1000.0), bites)
            cv2.imwrite("bites.png", self._debug_image)
            cv2.imwrite("quality.png",
                        colorize_kernel(self._last_bite_quality, 0.5))
        return bites

def create_signed_thresh(image, thresh):
    """ Create a binary -1/+1 image from a source 1-channel image. """
    thresh_image = np.array(image, dtype=np.float64)
    old_image = thresh_image.copy()
    thresh_image[old_image >= thresh] = -1.0
    thresh_image[old_image < thresh] = 1.0
    return thresh_image

def colorize_kernel(k, mult=2550.0):
    """ Create a colored (r=neg, g=pos) image of a kernel for display. """
    r = np.array(np.maximum(-k, 0.0) * mult, dtype=np.uint8)
    g = np.array(np.maximum(k, 0.0) * mult, dtype=np.uint8)
    b = np.array(np.zeros(k.shape), dtype=np.uint8)
    return cv2.merge((b, g, r))

def squash_depth(image):
    """ Create a grayscale image of a depth image for display. """
    minval = np.min(image)
    maxval = np.max(image)
    valrange = maxval - minval
    newimg = (image - minval) * (1.0 / valrange)
    return np.array(newimg * 255.0, dtype = np.uint8)

def debug_draw_bites(image, bites):
    """ Draw bites as circles into an image for debug display purposes. """
    ret = image
    for bite in bites:
        bpos = bite[0]
        brad = bite[1]
        cv2.circle(ret, swap_xy(bpos), brad, (255, 0, 255), 3)
    return ret
