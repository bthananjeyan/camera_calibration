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
            pos2 = pickle.load(f3)
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
    pts = [0, 5, 24, 29]
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
    robot_points = np.array([[0.12569576750145217, -0.03555745264763437, -0.15131487515430558], [0.0670304616177095, -0.04245991629247583, -0.14596557682787561], [0.11915661941735446, 0.011155598795363226, -0.14774158561475165], [0.05945375073541192, 0.002772632532740616, -0.1437100581074438]])
    print robot_points
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
    print mat
    pt = np.ones((4, 1))
    for i in range(3):
        pt[i, 0] = camera_points[0, i]
    # print pt
    print (mat * pt).T
