from bitefinder import *
import numpy as np
import cv2
import json

if __name__ == '__main__':
    testo = BiteFinder()
    cv2.imwrite("test.png", colorize_kernel(testo._kernel))
    test_image = cv2.imread("actual_depth.png")

    r = np.array(cv2.split(test_image)[0], dtype = np.float64)
    best_coeffs, num_inliers, residuals = ransac_plane(r, 200, 0.5)
    print("Best coeffs: %s" % str(best_coeffs))
    print("Num inliers: %d" % num_inliers)
    cv2.imwrite("plane_residuals.png", colorize_kernel(residuals * 0.1))

    jdata = {}
    jdata["plane"] = list(best_coeffs)

    bites = testo.find_bites(residuals, 10, -3.0)
    print(bites)

    jdata["bites"] = bites

    with open("data.json", "wt") as dest:
    	dest.write(json.dumps(jdata))

    cv2.imwrite("test_bites.png", testo._debug_image)
    cv2.imwrite("test_qual.png", colorize_kernel(testo._last_bite_quality * (1.0 / testo._kernel_size ** 2.0)))
