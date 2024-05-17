import numpy as np

'''
Code from below resource:
https://github.com/niconielsen32/ComputerVision/tree/master/StereoVisionDepthEstimation
'''

def find_depth(left_point, right_point, frame_left, frame_right, baseline, f, fov):
    # CONVERT FOCAL LENGTH f FROM [mm] TO [pixel]:
    height_right, width_right = frame_right.shape
    height_left, width_left = frame_left.shape
    

    if width_right == width_left:
        f_pixel = (width_right * 0.5) / np.tan(fov * 0.5 * np.pi/180)

    else:
        print('Left and right camera frames do not have the same pixel width')

    x_right = right_point[0]
    x_left = left_point[0]

    # CALCULATE THE DISPARITY:
    disparity = x_left-x_right      #Displacement between left and right frames [pixels]

    # CALCULATE DEPTH z:
    zDepth = (baseline*f_pixel)/disparity             #Depth in [cm]

    return zDepth