#!/usr/bin/env python3

#
# This is the for now ros hardware interface
#


import os, sys
import math, numpy as np

import rospy, cv2

#import roslib, rospy, rospkg, rostopic, dynamic_reconfigure.server
import geometry_msgs.msg

import common_vision.rospy_utils as cv_rpu

import site
site.addsitedir('/home/ubuntu/work/trilosaurus/trilosaurus_noros/scripts')
import robot_manager # from trilopi_no_ros

class RobotManager(cv_rpu.PeriodicNode):

    def __init__(self, cmd_topic='trilopi/cmd'):
        cv_rpu.PeriodicNode.__init__(self, 'trilopi_robot_manager_node')
        rospy.loginfo("trilopi_robot_manager_node Starting")
        self.rm = robot_manager.RobotManager()
        self.loop_cnt = 0
        self.lin_sp, self.ang_sp = 0., 0.
        #self.sub_cmd = rospy.Subscriber(cmd_topic, geometry_msgs.msg.Twist, self.cmd_cbk, queue_size=1)
        self.sub_cmd = cv_rpu.TwistSubscriber(topic='cmd_vel', what='robot_manager_ros', timeout=0.1)
        
    def start(self):
        self.rm.start()

    #def cmd_cbk(self, msg):
    #    self.lin_sp, self.ang_sp = msg.linear.x, msg.angular.z 
    #    print(f'on cmd {self.lin_sp} {self.ang_sp}')
        
    def periodic(self):
        try:
            self.lin_sp, self.ang_sp = self.sub_cmd.get()
            self.rm.loop(self.loop_cnt, (self.lin_sp, self.ang_sp))
        except cv_rpu.NoRXMsgException:
            #print('no msg')
            self.rm.loop(self.loop_cnt, None)
        except cv_rpu.RXMsgTimeoutException:
            #print('msg timeout')
            self.rm.loop(self.loop_cnt, None)
        self.loop_cnt += 1

          
def main(args):
  n = RobotManager()
  n.start()
  n.run(30)

if __name__ == '__main__':
    main(sys.argv)

