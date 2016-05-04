import numpy as np
import scipy
import pickle

def load_robot_points():
    lst = []
    f3 = open("calibration_data/psm1_calibration.p", "rb")
    pos1 = pickle.load(f3)
    lst.append(pos1)
    while True:
        try:
            pos1 = pickle.load(f3)
            lst.append(pos1)
        except EOFError:
            f3.close()
            return np.matrix(lst)

def load_camera_points():
    lst = []
    f3 = open("calibration_data/endoscope_points.p", "rb")
    pos1 = pickle.load(f3)
    lst.append(pos1)
    while True:
        try:
            pos1 = pickle.load(f3)
            lst.append(pos1)
        except EOFError:
            f3.close()
            return np.matrix(lst[0])

def solve_for_matrix():
    robot_points = load_robot_points()
    camera_points = load_camera_points()
    mat = np.zeros((12, 12))
    pts = [0, 7, 40, 47]
    for i in range(4):
        for j in range(4):
            if j == 3:
                mat[i, j] = 1
                mat[i + 4, j + 4] = 1
                mat[i + 8, j + 8] = 1
            else:
                mat[i, j] = camera_points[pts[i], j]
                mat[i + 4, j + 4] = camera_points[pts[i], j]
                mat[i + 8, j + 8] = camera_points[pts[i], j]
    b = []
    for i in range(3):
        for j in range(4):
            b.append(robot_points[j, i])
    b = np.matrix(b).T
    x = np.linalg.solve(mat, b)
    return x

if __name__ == '__main__':
    robot_points = load_robot_points()
    camera_points = load_camera_points()

    mat = solve_for_matrix().reshape(3, 4)
    pt = np.ones((4, 1))
    for i in range(3):
        pt[i, 0] = camera_points[19, i]
    print (mat * pt).T
    print robot_points[4,:]

