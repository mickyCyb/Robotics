#!/user/bin/env python
from __future__ import print_function
from arm_gazebo.srv import ik, ikResponse
import rospy
import numpy as np
import tinyik as IK

def handle_ik(req):
    arm = IK.Actuator([
    "z", [0.0, 0.0, req.link1_length + req.link2_length],
    "x", [0.0, 0.0, req.link3_length],
    "x", [0.0, 0.0, req.link4_length],
    "x", [0.0, 0.0, req.link5_length],
    "z", [0.0, 0.0, req.link6_length],
    "x", [0.0, 0.0, req.link7_length],
    ])

    arm.ee = [req.x, req.y, req.z]

    angles = arm.angles

    res = ikResponse()
    
    res.joint1_angle = angles[0]
    res.joint2_angle = angles[1]
    res.joint3_angle = angles[2]
    res.joint4_angle = angles[3]
    res.joint5_angle = angles[4]
    res.joint6_angle = angles[5]

    print("Returning......", angles)
    return res

def ik_server():
    rospy.init_node("inverse_kinematics_server")
    s = rospy.Service('ik', ik, handle_ik)
    print("Ready to calculate inverse kinematics")
    rospy.spin()

if __name__ == "__main__":
    ik_server()