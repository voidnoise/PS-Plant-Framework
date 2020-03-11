#!/usr/bin/env python

'''
Simple example of stereo image matching and point cloud generation.

Resulting .ply file cam be easily viewed using MeshLab ( http://meshlab.sourceforge.net/ )
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def main():
    print('loading images...')

    imgL = cv.pyrDown(cv.imread('aloeL.jpg'))  # downscale images for faster processing
    imgR = cv.pyrDown(cv.imread('aloeR.jpg'))

    # disparity range is tuned for 'aloe' image pair
    window_size = 3
    min_disp = 16
    num_disp = 112-min_disp

    '''
    StereoMatching parameters:

        'minDisparity' is the smallest disparity value to search for. Use smaller values to 
        look for scenes which include objects at infinity, and larger values for scenes
        near the cameras. Negative minDisparity can be useful if the cameras are 
        intentionally cross-eyed, but you wish to calculate long-range distances. Setting 
        this to a sensible value reduces interference from out of scene areas and reduces
        unneeded computation.

        'numDisparities' is the range of disparities to search over. It is effectively your
        scene's depth of field setting. Use smaller values for relatively shallow depth of 
        field scenes, and large values for deep depth-of-field scenes. Setting this to a 
        sensible value reduces interference from out of scene areas and reduces unneeded 
        computation.

        'blockSize' is the dimension (in pixels on a side) of the block which is compared 
        between the left and right images. Setting this to a sensible value reduces 
        interference from out of scene areas and reduces unneeded computation.

        'disp12MaxDiff', 'speckleRange', 'speckleWindowSize' - used in filtering the 
        disparity map before returning, looking for areas of similar disparity (small 
        areas will be assumed to be noise and marked as having invalid depth information). 
        These reduces noise in disparity map output.

    StereoBM:

        'preFilterCap', 'preFilterSize, preFilterType - used in filtering the input images
        before disparity computation. These may improve noise rejection in input images.

        'ROI1', 'ROI2' - region of interest, used to constrain the computation to a 
        smaller rectangle within the input images. These may help avoid unneeded computation.

        'textureThreshold', 'uniquenessRation', 'smallerBlockSize' - used in filtering the 
        disparity map before returning. May reduce noise.

    StereoSGBM:

        'mode' - selects the block compare mechanism to use. Some modes may perform much 
        better for certain input scenes, comparing a few lines out of the entire block 
        rather than every pixel in the block.

        'P1', 'P2', 'uniquenessRation' - used in filtering the disparity map before 
        returning to reject small blocks. May reduce noise.

        'preFilterCap' - used in filtering the input images before disparity computation. 
        These may improve noise rejection in input images.
    '''
    stereo = cv.StereoSGBM_create(
        minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    print('generating 3d point cloud...',)
    h, w = imgL.shape[:2]
    f = 0.8*w                          # guess for focal length
    Q = np.float32([[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1,      0]])
    points = cv.reprojectImageTo3D(disp, Q)
    colors = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]
    out_fn = 'out.ply'
    write_ply(out_fn, out_points, out_colors)
    print('%s saved' % out_fn)

    cv.imshow('left', imgL)
    cv.imshow('right', imgR)
    cv.imshow('disparity', (disp-min_disp)/num_disp)
    cv.waitKey()

    print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()