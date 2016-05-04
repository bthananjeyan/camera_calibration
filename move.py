import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx
import fitplane
from solve_system import *


def move_to(pt):
    psm1_initial = [pt[0], pt[1], pt[2], 0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]
    start_frame1 = get_frame(psm1_initial)
    psm1.move_cartesian_frame(start_frame1)


if __name__ == '__main__':
    mat = solve_for_matrix()
    camera_pts = load_camera_points()

    psm1 = robot("PSM1")

    pt = np.ones((4, 1))
    for i in range(3):
        pt[i, 0] = camera_points[19, i]

    pt = mat * pt

    move_to(pt)

