#!/user/bin/env python
from __future__ import print_function
from arm_gazebo.srv import fk, fkResponse
import rospy
import numpy as np

def Rx(rad):
    return np.array([ 
        [1, 0, 0, 0 ],
        [0, np.cos(rad), -np.sin(rad), 0],
        [0, np.sin(rad), np.cos(rad), 0],
        [0, 0, 0, 1] 
    ])

def Rz(rad):
    return np.array([ 
        [np.cos(rad), -np.sin(rad), 0, 0],
        [np.sin(rad), np.cos(rad), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1 ]
    ])

def Tz(d):
    m = np.eye(4)
    m[2, 3] = d
    return m


def handle_fk(req):
    M1 = Tz(req.link1_length)  #base
    M2 = Rz(req.joint1_angle).dot(Tz(req.link2_length))  #arm 1 tip
    M3 = Rx(req.joint2_angle).dot(Tz(req.link3_length))  #arm 2 tip
    M4 = Rx(req.joint3_angle).dot(Tz(req.link4_length))  #arm 3 tip
    M5 = Rx(req.joint4_angle).dot(Tz(req.link5_length))  #arm 4 tip
    M6 = Rz(req.joint5_angle).dot(Tz(req.link6_length))  #arm 5 tip
    M7 = Rx(req.joint6_angle).dot(Tz(req.link7_length))  #palm tip

    M = (((((M1.dot(M2)).dot(M3)).dot(M4)).dot(M5)).dot(M6)).dot(M7)

    res = fkResponse()
    res.x = M[0][3]
    res.y = M[1][3]
    res.z = M[2][3]
    print("Returning......", M)
    return res

def fk_server():
    rospy.init_node("forward_kinematics_server")
    s = rospy.Service('fk', fk, handle_fk)
    print("Ready to calculate forward kinematics")
    rospy.spin()

if __name__ == "__main__":
    fk_server()