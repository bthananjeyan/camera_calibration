import dvrk
import Tkinter
from Tkinter import *
import pickle
import sys

arm1 = dvrk.psm("PSM1")
arm2 = dvrk.psm("PSM2")
get_arm_pos = lambda arm: arm.get_current_position().p


def exit_callback():
    sys.exit()

def record_PSM1_location():
    psm_pose = get_arm_pos(arm1)
    print psm_pose
    f = open("calibration_data/psm1_calibration.p", "a")
    pickle.dump(psm_pose, f)
    f.close()

def record_PSM2_location():
    psm_pose = get_arm_pos(arm2)
    print psm_pose
    f = open("calibration_data/psm2_calibration.p", "a")
    pickle.dump(psm_pose, f)
    f.close()

if __name__ == '__main__':
    open('calibration_data/psm1_calibration.p', 'w+').close()
    open('calibration_data/psm2_calibration.p', 'w+').close()

    top = Tkinter.Tk()
    top.title('Calibration')
    top.geometry('400x200')

    B1 = Tkinter.Button(top, text="Record Position (Right)", command = record_PSM1_location)
    B2 = Tkinter.Button(top, text="Record Position (Left)", command = record_PSM2_location)
    D = Tkinter.Button(top, text="Exit", command = exit_callback)

    B1.pack()
    B2.pack()
    D.pack()

    top.mainloop()
