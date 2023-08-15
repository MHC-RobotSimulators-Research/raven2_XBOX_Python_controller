from ambf_raven_def import *
import math as m
import numpy as np
import raven_ik as ik
import ambf_raven_def as ard
def joint_to_dhvalue(joint, arm):
    success = False
    dhvalue = np.zeros(7, dtype='float')
    if arm < 0 or arm >= RAVEN_ARMS or joint.size != RAVEN_JOINTS:
        return success, dhvalue
    for i in range(RAVEN_JOINTS):
        if i != 2:
            if i == 5:
                if arm == 0:
                    dhvalue[i] = (joint[i] - joint[i + 1])
                else:
                    dhvalue[i] = -(joint[i] - joint[i + 1])
            else:
                dhvalue[i] = joint[i]
                while dhvalue[i] > m.pi:
                    dhvalue[i] -= 2 * m.pi
                while dhvalue[i] < -m.pi:
                    dhvalue[i] += 2 * m.pi
        else:
            dhvalue[i] = joint[i]
    success = True
    return success, dhvalue

def fwd_trans(a, b, dh_alpha, dh_theta, dh_a, dh_d):
    if ((b <= a) or b == 0):
        print("Invalid start/end indices")

    xx = float(m.cos(dh_theta[a]))
    xy = float(-m.sin(dh_theta[a]))
    xz = float(0)

    yx = float(m.sin(dh_theta[a])) * float(m.cos(dh_alpha[a]))
    yy = float(m.cos(dh_theta[a])) * float(m.cos(dh_alpha[a]))
    yz = float(-m.sin(dh_alpha[a]))

    zx = float(m.sin(dh_theta[a])) * float(m.sin(dh_alpha[a]))
    zy = float(m.cos(dh_theta[a])) * float(m.sin(dh_alpha[a]))
    zz = float(m.cos(dh_alpha[a]))

    px = float(dh_a[a])
    py = float(-m.sin(dh_alpha[a])) * float(dh_d[a])
    pz = float(m.cos(dh_alpha[a])) * float(dh_d[a])

    xf = np.matrix([[xx, xy, xz, px],
                    [yx, yy, yz, py],
                    [zx, zy, zz, pz],
                    [0, 0, 0, 1]])

    if b > a + 1:
        xf = np.matmul(xf, fwd_trans(a + 1, b, dh_alpha, dh_theta, dh_a, dh_d))

    return xf

def fwd_kinematics(arm, input_joint_pos):
    success = False

    dh_alpha = np.zeros(7, dtype = 'float')
    dh_theta = np.zeros(7, dtype = 'float')
    dh_a = np.zeros(7, dtype = 'float')
    dh_d = np.zeros(7, dtype = 'float')

    j2d = joint_to_dhvalue(input_joint_pos, arm)
    worked, jp_dh = j2d

    if worked == False:
        print("Something went wrong with joint to dh conversion")
        return success
    for i in range(RAVEN_JOINTS):
        if i == 2:
            dh_d[i] = jp_dh[i]
            dh_theta[i] = RAVEN_DH_THETA[arm][i]
        else:
            dh_d[i] = RAVEN_DH_D[arm][i]
            dh_theta[i] = jp_dh[i]
        dh_alpha[i] = RAVEN_DH_ALPHA[arm][i]
        dh_a[i] = RAVEN_DH_A[arm][i]

    print(dh_alpha)

    output_transformation = np.matmul(np.matmul(RAVEN_T_CB, RAVEN_T_B0[arm]), fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))
    return output_transformation

def fwd_kinematics_p5(arm, input_joint_pos):
    """
    Gets the position of joint 5 origin
    """
    success = False

    dh_alpha = np.zeros(7, dtype = 'float')
    dh_theta = np.zeros(7, dtype = 'float')
    dh_a = np.zeros(7, dtype = 'float')
    dh_d = np.zeros(7, dtype = 'float')

    j2d = joint_to_dhvalue(input_joint_pos, arm)
    worked, jp_dh = j2d

    if worked == False:
        print("Something went wrong with joint to dh conversion")
        return success
    for i in range(RAVEN_JOINTS):
        if i == 2:
            dh_d[i] = jp_dh[i]
            dh_theta[i] = RAVEN_DH_THETA[arm][i]
        else:
            dh_d[i] = RAVEN_DH_D[arm][i]
            dh_theta[i] = jp_dh[i]
        dh_alpha[i] = RAVEN_DH_ALPHA[arm][i]
        dh_a[i] = RAVEN_DH_A[arm][i]

    output_transformation = np.matmul(np.matmul(RAVEN_T_CB, RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))
    return output_transformation

def main():
    joint1 = np.array ([0.5216294527053833, 1.5818907022476196, 0.4003618359565735, 0.06717494130134583, -0.0012699863873422146, 0.8005852699279785, 0.8528488874435425])
    joint2  = np.array([0.5229672789573669, 1.5859100818634033, 0.40006133913993835, -0.05457039922475815, 0.024380048736929893, 0.8693668246269226, 0.7849395871162415],  dtype = 'float')
    success, dhvalue = joint_to_dhvalue(joint1, 0)
    print("dh value 1: ", dhvalue)
    fk5_1 = fwd_kinematics(0, joint1)
    print("fk5 1: ", fk5_1)
    jpl_1 = ik.inv_kinematics_p5(0, fk5_1, ard.HOME_JOINTS[5]+ ard.HOME_JOINTS[6], ard.HOME_DH)
    print("new joint arm 1: ",jpl_1[0])

    success_2, dhvalue_2 = joint_to_dhvalue(joint2, 1)
    print("dh value 2: ", dhvalue_2)
    fk5_2 = fwd_kinematics(1, joint2)
    print("fk5 2: ")
    print(fk5_2)
    jpl_2 = ik.inv_kinematics_p5(1, fk5_2, ard.HOME_JOINTS[5] + ard.HOME_JOINTS[6], ard.HOME_DH)
    print("new joint arm 2: ",jpl_2[0])

if __name__ == "__main__":
    main()
