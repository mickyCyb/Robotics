import rospy
from pub_sub.msg import Inputmsg, Outputmsg 
import numpy as np
import math

pub = rospy.Publisher('manipulated_vector', Outputmsg, queue_size=10)

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo("received x: %s, y: %s, z: %s, alpha: %s, beta: %s, gamma: %s, d: %s " % (data.x, data.y, data.z, data.alpha, data.beta, data.gamma, data.d))
    
    given_vector = np.array([data.x, data.y, data.z, 1])

    rotation_along_x_matrix = np.array([ 
        [1, 0, 0, 0 ],
        [0, math.cos(data.alpha), -math.sin(data.alpha), 0],
        [0, math.sin(data.alpha), math.cos(data.alpha), 0],
        [0, 0, 0, 1] 
    ])

    rotation_along_y_matrix = np.array([ 
        [math.cos(data.beta), 0, math.sin(data.beta), 0],
        [0, 1, 0, 0 ],
        [-math.sin(data.beta), 0, math.cos(data.beta), 0],
        [0, 0, 0, 1] 
    ])

    rotation_along_z_matrix = np.array([ 
        [math.cos(data.gamma), -math.sin(data.gamma), 0, 0],
        [math.sin(data.gamma), math.cos(data.gamma), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1 ]
    ])

    translation_matrix = np.array([
        [1, 0, 0, data.d],
        [0, 1, 0, data.d],
        [0, 0, 1, data.d],
        [0, 0, 0, 1]
    ])

    # retVector = np.dot(translation_matrix, np.dot(rotation_along_z_matrix, np.dot(rotation_along_y_matrix, np.dot(rotation_along_x_matrix, given_vector))))
    retVector = (((given_vector.dot(rotation_along_x_matrix)).dot(rotation_along_y_matrix)).dot(rotation_along_z_matrix)).dot(translation_matrix)
    
    msg = Outputmsg()
    msg.x = retVector[0]
    msg.y = retVector[1]
    msg.z = retVector[2]

    pub.publish(msg)

def subscriber():
    rospy.init_node('main_sub', anonymous=True)

    rospy.Subscriber("chatter", Inputmsg, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()