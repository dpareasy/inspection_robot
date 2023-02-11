#!/usr/bin/env python3

import rospy
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from inspection_robot.msg import SurveyAction, SurveyResult
from geometry_msgs.msg import Twist
import inspection_robot  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

class SurveyRoom(object):
    def __init__(self):

        self._as = SimpleActionServer('surveyor',
                                        inspection_robot.msg.SurveyAction,
                                        execute_cb=self.execute_callback,
                                        auto_start=False)
        self._as.start()
    
    def execute_callback(self, goal):

        rate = rospy.Rate(10)

        goal = goal.survey

        if goal is None:
            print('No goal has been received')
            self._as.set_aborted()
            return
        
        # Publish a twist message to rotate the base link
        twist = Twist()
        twist.angular.z = 1.0
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        print('Culo')

        for i in range(200):
            pub.publish(twist)
            rate.sleep()

            # Check if the action has been cancelled by the client
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                return

        result = SurveyResult()
        result.result = 'Scanned area'
        print('goal succeded')
        self._as.set_succeeded(result)
        return

if __name__ == '__main__':
    rospy.init_node("survey_controller")
    server = SurveyRoom()
    rospy.spin()