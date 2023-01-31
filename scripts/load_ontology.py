import rospy
from numpy import *
from std_msgs.msg import Float64
import actionlib
from inspection_robot.msg import MoveArmAction, MoveArmFeedback, MoveArmResult

link1 = Float64()
link2 = Float64()
link3 = Float64()

pub1 = rospy.Publisher('/m2wr/joint1_position_controller/command', Float64, queue_size = 10)
pub2 = rospy.Publisher('/m2wr/joint2_position_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/m2wr/joint3_position_controller/command', Float64, queue_size = 10)

def pub_position():

    link1.data = 1.5
    link2.data = 1.5
    link3.data = 2.0
    print("CULO 8")
    pub1.publish(link1)
    pub2.publish(link2)
    pub3.publish(link3)