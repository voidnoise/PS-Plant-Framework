#!/bin/python
# Copyright (C) 2014 Daniel Lee <lee.daniel.1986@gmail.com>
#
# This file is part of StereoVision.
#
# StereoVision is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# StereoVision is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with StereoVision.  If not, see <http://www.gnu.org/licenses/>.

"""
Tool for creating and exporting colored point clouds from stereo image pairs.
"""

import argparse

import cv2
from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.calibration import StereoCalibration
from stereovision.stereo_cameras import CalibratedPair
from stereovision.ui_utils import STEREO_BM_FLAG
import numpy as np

def main():
    """Produce PLY point clouds from stereo image pair."""
    parser = argparse.ArgumentParser(description="Read images taken with "
                                     "stereo pair and use them to produce 3D "
                                     "point clouds that can be viewed with "
                                     "MeshLab.", parents=[STEREO_BM_FLAG])
    parser.add_argument("calibration", help="Path to calibration folder.")
    parser.add_argument("left", help="Path to left image")
    parser.add_argument("right", help="Path to right image")
    parser.add_argument("output", help="Path to output file.")
    parser.add_argument("--bm_settings",
                        help="Path to block matcher's settings.")
    args = parser.parse_args()

    image_pair = [cv2.imread(image) for image in [args.left, args.right]]
    calib_folder = args.calibration
    if args.use_stereobm:
        # if True:
        print('using stereoBM')
        block_matcher = StereoBM()
    else:
        print('using StereoSGBM')
        block_matcher = StereoSGBM()

        min_disparity=16, 
        num_disp=96, 
        sad_window_size=3,
        uniqueness=10, 
        speckle_window_size=100, 
        speckle_range=32,
        p1=216, 
        p2=864, 
        max_disparity=1, 
        full_dp=False,
        

    if args.bm_settings:
        print('using bm_settings')
        block_matcher.load_settings(args.bm_settings)

    camera_pair = CalibratedPair(None,
                                StereoCalibration(input_folder=calib_folder),
                                block_matcher)
    print('camera_pair created')
    # cv2.imshow('l',image_pair[0])
    # cv2.imshow('r',image_pair[1])
    # cv2.waitKey(0)

    rectified_pair = camera_pair.calibration.rectify(image_pair)
    print('rectified pair', type(rectified_pair), type(rectified_pair[0]))
    # cv2.imshow('l',rectified_pair[0])
    # cv2.imshow('r',rectified_pair[1])
    # cv2.waitKey(0)

    # stereo = cv2.StereoBM_create()
    # # stereo = cv2.StereoSGBM_create(minDisparity=0,
    # #                                numDisparities=64,
    # #                                blockSize=11)
    # # Compute disparity at full resolution and downsample
    # disp = stereo.compute(rectified_pair[0], rectified_pair[1]).astype(float)
    # k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    # disparity_map = cv2.morphologyEx(disparity_map, cv2.MORPH_CLOSE, k)

    points = camera_pair.get_point_cloud(rectified_pair)
    points = points.filter_infinity()
    points.write_ply(args.output)
    print('now in 3d')

if __name__ == "__main__":
    main()
