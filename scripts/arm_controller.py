#!/usr/bin/env python3

import rospy
import random
from numpy import *
from std_msgs.msg import Float64
import actionlib
from inspection_robot.msg import MoveArmAction, MoveArmFeedback, MoveArmResult, MarkerList

link1 = Float64()
link2 = Float64()
link3 = Float64()

pub1 = rospy.Publisher('/insprob/joint1_position_controller/command', Float64, queue_size = 10)
pub2 = rospy.Publisher('/insprob/joint2_position_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/insprob/joint3_position_controller/command', Float64, queue_size = 10)

global marker_id

class ArmControllerServer():
    def __init__(self):
        #self._action_name = name
        self.a_server = actionlib.SimpleActionServer("move_arm_as", MoveArmAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        self.marker_list = []
        self.marker_count = 0
        rospy.Subscriber("marker_detector/MarkerList", MarkerList, self.marker_cb)

    def marker_cb(self, msg):

        global marker_id 
        marker_id = msg.markers
        print(marker_id)
        self.marker_list.append(marker_id)
    """
        function to publish on the joint topics
    """
    def pub_position(self,joint1, joint2, joint3):

        link1.data = joint1
        link2.data = joint2
        link3.data = joint3
        pub1.publish(link1)
        pub2.publish(link2)
        pub3.publish(link3)

    def execute_cb(self, goal):
        print(goal)
        positions = array([['pose0',0.0,0.0,0.0],['pose1',0.2,0.0,-0.7],['pose2',1.5,0.0,0.2],['pose3',1.5,0.0,-0.5],['pose4',2.2,0.0,0.2],['pose5',2.8,0.0,0.4],['pose6',-1.2,0,0.3],['pose7',-1.2,0,-0.5]])
        success = True
        feedback = MoveArmFeedback()
        result = MoveArmResult()

        if goal is None:
            print('no goal has been received')
            return

        i = 0
        while size(self.marker_list) <= 6:
            if self.a_server.is_preempt_requested():
                success = False
                break

            epsilon = 0#random.uniform(0, 0.15)
            self.pub_position(float(positions[i][1]), float(positions[i][2]), float(positions[i][3]))
            #self.pub_position(float(positions[i][1]) + epsilon, float(positions[i][2]) + epsilon , float(positions[i][3]) + epsilon)
            #self.a_server.publish_feedback(feedback)
            rospy.sleep(0.5)

            if i == 7:
                i = 0
            else:
                i = i + 1

        if success:
            self.a_server.set_succeeded(result)

if __name__== '__main__':

    rospy.init_node('arm_controller')
    s = ArmControllerServer()
    rospy.spin()