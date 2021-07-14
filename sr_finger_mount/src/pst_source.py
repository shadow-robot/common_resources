from sr_robot_msgs.msg import ShadowPST
import numpy as np
import rospy 

rospy.init_node("pst_source_node")
pub = rospy.Publisher("rh/tactile", ShadowPST, queue_size=1)

msg = ShadowPST()
t = 0

i = 0.1

while not rospy.is_shutdown():
    msg.pressure = [int(np.sin(t)*100)+100, int(np.sin(t+50)*100)+100, 1 ,2, 3]

    #msg.pressure = [int(1000 * i)+100]

    pub.publish(msg)
    rospy.Rate(10).sleep()

    if t % 50.0 == 0:
        i = i * -1

    t = t + 0.1