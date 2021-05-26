import rospy
from std_msgs import msg
from pub_sub.msg import Inputmsg, Outputmsg

# def callback(data):
#     rospy.loginfo("x: %s, y: %s, z: %s" % (data.x, data.y, data.z))
    

def publisher():
    pub = rospy.Publisher('chatter', Inputmsg, queue_size=10)
    rospy.init_node('main_pub', anonymous=True)
    rate = rospy.Rate(5)
    msg = Inputmsg()
    msg.x = 10
    msg.y = 10
    msg.z = 10
    msg.alpha = 2
    msg.beta = 1
    msg.gamma = 1
    msg.d = 10

    while not rospy.is_shutdown():
        # hello_str = "Hello worls %s" % rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        # rospy.Subscriber("manipulated_vector", Outputmsg, callback)
        rate.sleep()

# def subscribe():
#     rospy.Subscriber("manipulated_vector", Outputmsg, callback)
#     rospy.spin()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass