#! /usr/bin/env python3

import random
import rospy
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from inspection_robot.msg import ControlFeedback, ControlResult
from inspection_robot.srv import SetPose
import inspection_robot # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

# A tag for identifying logs producer.

# An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `robot-state` node.
class ControllingAction(object):

    def __init__(self):
        self._as = SimpleActionServer('motion/controller',
                                      inspection_robot.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        # Log information.
        #log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
        #           f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
        #rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in 
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None:
            print('no goal has been received')
            self._as.set_aborted()
            return
        target = goal.target_point
        print(target)
        # Construct the feedback and loop for each via point.
            #feedback = ControlFeedback()
            #print('Server is controlling')
        #rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))

            # Check that the client did not cancel this service.
            #if self._as.is_preempt_requested():
            #    print('Service has been cancelled by the client!')
                # Here implement the cance Goal
                # Actually cancel this service.
            #    self._as.set_preempted()
            #    return
        
        #
        
            # Publish a feedback to the client to simulate that a via point has been reached. 
            #feedback.reached_point = point
            #self._as.publish_feedback(feedback)


            # Publish the results to the client.
            #result = ControlResult()
            #result.reached_point = feedback.reached_point

            #return  # Succeeded.


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node('controller', log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
