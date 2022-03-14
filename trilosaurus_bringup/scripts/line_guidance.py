#!/usr/bin/env python3

# ./work/trilosaurus/trilosaurus_bringup/scripts/line_guidance.py
# 
import os, sys
import math, numpy as np
import roslib, rospy, rospkg, rostopic, dynamic_reconfigure.server
import geometry_msgs.msg

import cv2
import common_vision.rospy_utils as cv_rpu

import two_d_guidance.trr.guidance as tdg_guid
#import two_d_guidance.trr.rospy_utils as trr_rpu # go away!!!
import two_d_guidance.cfg.trr_guidanceConfig

class Publisher:
    def __init__(self, topic='guidance/status', cmd_topic='guidance/cmd'):
        self.sta_pub = cv_rpu.GuidanceStatusPublisher(topic, 'guidance')
        self.pub_cmd = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=1)

    def publish_cmd(self, lin, ang):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x, msg.angular.z = lin, ang
        self.pub_cmd.publish(msg)
        
    def publish_status(self, guidance): self.sta_pub.publish(guidance)




class Node(cv_rpu.PeriodicNode):

    def __init__(self):
        cv_rpu.PeriodicNode.__init__(self, 'guidance_node')
        rospy.loginfo("guidance_node Starting")
        ref_frame = rospy.get_param('~ref_frame', 'trilopi/base_link_footprint')
        rospy.loginfo(' using ref_frame: {}'.format(ref_frame))

        lookahead = 0.2#rospy.get_param('~lookahead_dist', 0.2)
        tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
        path_name = rospy.get_param('~path_name', 'demo_z/track_trr_real_1.npz')
        fname = os.path.join(tdg_dir, 'paths/{}'.format(path_name))
        self.guidance = tdg_guid.Guidance(lookahead=lookahead, path_fname=fname, vel_sp=0.025)
        # dynamic reconfigurable parameters
        self.cfg_srv = dynamic_reconfigure.server.Server(two_d_guidance.cfg.trr_guidanceConfig, self.dyn_cfg_callback)
        
        cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        rospy.loginfo(' publishing commands on: {}'.format(cmd_topic))
        self.publisher = Publisher(cmd_topic=cmd_topic)

        
        self.lane_model_sub = cv_rpu.LaneModelSubscriber('/vision/lane/model', timeout=0.15)

        #self.guidance.set_mode(2)
        self.guidance.lookaheads[0].d = 0.13
        
    def dyn_cfg_callback(self, config, level):
        rospy.loginfo(f" Reconfigure Request: level: {level}")
        rospy.loginfo(" Reconfigure Request: mode: {guidance_mode}, lookahead: {lookahead_dist}, vel_setpoint: {vel_sp}".format(**config))
        if level == -1:
            config['guidance_mode'] = self.guidance.mode
            config['lookahead_dist'] = self.guidance.lookaheads[0].d
            config['vel_setpoint'] = self.guidance.vel_ctl.sp
            print(f"defaults: mode {config['guidance_mode']} lookahead {config['lookahead_dist']} vel {config['vel_setpoint']}")
        else:
            self.guidance.set_mode(config['guidance_mode'])
            #self.guidance.lookaheads[0].set_dist(config['lookahead_dist'])
            #self.guidance.lookahead_mode = config['lookahead_mode']
            #self.guidance.vel_ctl.sp = config['vel_sp']
        return config

        
    def periodic(self):
        # if we don't have a lane, do nothing - and it's bad! report it
        try:
            self.lane_model_sub.get(self.guidance.lane)
        except cv_rpu.NoRXMsgException:
            rospy.loginfo_throttle(1., 'guidance (lane): NoRXMsgException')
            return
        except cv_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., 'guidance: (lane) RXMsgTimeoutException')
            return
        _s, _is, _v, _ds, _df = 0, 0, 0.5, 0., 0.
        _v = 0. # est_vel
        self.guidance.compute(_s, _is, _v, expl_noise=0.)
        if self.guidance.mode != tdg_guid.Guidance.mode_idle:
            #self.guidance.lin_sp = 0.#5
            #self.guidance.ang_sp = 0.#5
            #print(self.guidance.lin_sp, self.guidance.ang_sp)
            self.publisher.publish_cmd(self.guidance.lin_sp, self.guidance.ang_sp)
        self.publisher.publish_status(self.guidance)
        
    
def main(args):
  Node().run(30)

if __name__ == '__main__':
    main(sys.argv)
