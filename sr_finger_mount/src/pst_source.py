from sr_robot_msgs.msg import ShadowPST
import numpy as np
import rospy 

rospy.init_node("pst_source_node")
pub = rospy.Publisher("rh/tactile", ShadowPST, queue_size=1)

msg = ShadowPST()
t = 0

while not rospy.is_shutdown():
    msg.pressure = [int(np.sin(t)*100)+100]
    pub.publish(msg)
    rospy.Rate(50).sleep()
    t = t + 0.01
