#! /usr/bin/env python

import rospy


import actionlib
import control_msgs.msg

class UR5e_action_server():
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self):
        self._action_name = 'UR5e_trajectory'
        self._action_server = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_callback, auto_start = False)
        self._action_server.start()
      
    def execute_callback(self, goal):
        print("Executing action server callback function")
        # helper variables
        r = rospy.Rate(1)
        success = True

        print("new msg :D")
        print("goal: ", goal)
        
        # append the seeds for the fibonacci sequence
        # self._feedback.sequence = []
        # self._feedback.sequence.append(0)
        # self._feedback.sequence.append(1)
        self._feedback = goal

        print("wah")
        
        # publish info to the console for the user
        #rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal))
        
        # start executing the action
        # for i in range(1, goal.):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
        
        #self._as.publish_feedback(self._feedback)
        
          
        if success:
            self._result = self._feedback
            rospy.loginfo('%s: Succeeded' % self._action_name)
            #self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('UR5_action_server')
    server = UR5e_action_server()
    rospy.spin()

