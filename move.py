import rospy, pickle, time
import robot
from geometry_msgs.msg import Pose
import numpy as np
from solve_system import *
import PyKDL


def move_to(pt):
    pos = [pt[0], pt[1], pt[2], 0.617571885272, 0.59489495214, 0.472153066551, 0.204392867261]
    rotation = PyKDL.Rotation.Quaternion(pos[3], pos[4], pos[5], pos[6])
    position = PyKDL.Vector(pos[0], pos[1], pos[2])
    fr = PyKDL.Frame(rotation, position)
    psm1.move_cartesian_frame(fr)

if __name__ == '__main__':
    mat = solve_for_matrix()
    camera_points = load_camera_points()

    psm1 = robot.robot("PSM1")

    pt = np.ones((4, 1))
    for i in range(3):
        pt[i, 0] = camera_points[14, i]
    pt = mat.reshape(3, 4) * pt
    move_to(pt)

