import numpy as np
import random
import cv2


def swap_xy(t):
    return (t[1], t[0])


def sanitize_numpy_int(t):
    return [int(v) for v in t]

def count_inliers(row_idx, col_idx, image, coeffs, inlier_thresh, valid_mask):
    pmat = (coeffs[0] * row_idx) + (coeffs[1] * col_idx) + coeffs[2]
    residuals = image - pmat
    num_inliers = np.sum((np.abs(residuals) < inlier_thresh) * valid_mask)
    return num_inliers, residuals

def ransac_plane(img, niter, inlier_thresh, initial_coeffs = None):
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
        most_inliers, best_residuals = count_inliers(row_idx, 
                                                     col_idx, 
                                                     img, 
                                                     best_coeffs, 
                                                     inlier_thresh, 
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

        #pmat = (x[0] * row_idx) + (x[1] * col_idx) + x[2]
        #residuals = img - pmat
        #num_inliers = np.sum((np.abs(residuals) < inlier_thresh) * valid_mask)
        num_inliers, residuals = count_inliers(row_idx, 
                                               col_idx, 
                                               img, 
                                               x, 
                                               inlier_thresh, 
                                               valid_mask)

        if num_inliers > most_inliers or best_residuals is None:
            most_inliers = num_inliers
            best_residuals = residuals
            best_coeffs = x

    best_residuals[np.logical_not(valid_mask)] = 0.0
    return (best_coeffs, most_inliers, best_residuals)


class BiteFinder(object):

    """finds bite size things to stab in a depth image"""

    def __init__(self, options = {}):
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
        pixel_bite_radius = int(self._kernel_size * self._bite_radius * 0.5)

        for i in range(n):
            bpos = np.unravel_index(np.argmax(bite_quality * valid_map),
                                    bite_quality.shape)
            bval = bite_quality[bpos]
            # print(bval)
            if bval > self._quality_thresh:
                ret.append((sanitize_numpy_int(bpos), pixel_bite_radius, bval))
                cv2.circle(valid_map, 
                           swap_xy(bpos), 
                           pixel_bite_radius * 2, 0.0, -1)
            else:
                break

        return ret

    def find_bites(self, image, n, thresh=0.0):
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


def colorize_kernel(k, mult=2550.0):
    r = np.array(np.maximum(-k, 0.0) * mult, dtype=np.uint8)
    g = np.array(np.maximum(k, 0.0) * mult, dtype=np.uint8)
    b = np.array(np.zeros(k.shape), dtype=np.uint8)
    return cv2.merge((b, g, r))

def squash_depth(image):
    minval = np.min(image)
    maxval = np.max(image)
    valrange = maxval - minval
    newimg = (image - minval) * (1.0 / valrange)
    return np.array(newimg * 255.0, dtype = np.uint8)

def debug_draw_bites(image, bites):
    ret = image
    for bite in bites:
        bpos = bite[0]
        brad = bite[1]
        cv2.circle(ret, swap_xy(bpos), brad, (255, 0, 255), 3)
    return ret
