#!/usr/bin/env python3

# ./work/trilosaurus/trilosaurus_bringup/scripts/line_guidance_display.py _camera:=camera1 camera1/image_raw/compressed:=camera1/image/compressed

import os, sys
import math, numpy as np
import roslib, rospy, rospkg, rostopic, dynamic_reconfigure.server
#import nav_msgs.msg , geometry_msgs.msg, visualization_msgs.msg, sensor_msgs.msg

import cv2

import common_vision.utils as cv_u
import common_vision.rospy_utils as cv_rpu

import two_d_guidance.trr.rospy_utils as trr_rpu

class ImgPublisher(cv_rpu.DebugImgPublisher):
    def __init__(self, img_topic, cam_name):
        cv_rpu.DebugImgPublisher.__init__(self, cam_name, img_topic)
 
    def _draw(self, img_bgr, model, data):
        y0=20; font_color=(128,0,255)
        f, h1, h2, c, w = cv2.FONT_HERSHEY_SIMPLEX, 1.25, 0.9, font_color, 2
        cv2.putText(img_bgr, 'Guidance:', (y0, 40), f, h1, c, w)

        msg = model.guid_stat_sub.get()
        str_of_guid_mode = ['idle', 'stopped', 'driving']
        mode_name, curv = str_of_guid_mode[msg.guidance_mode], msg.poly[1]
        cv2.putText(img_bgr, 'mode: {:s} curv: {:-6.2f}'.format(mode_name, curv), (20, 90), f, h2, c, w)
        
        lin_odom, ang_odom = 0, 0#model.odom_sub.get_vel()
        lin_sp, ang_sp = msg.lin_sp, msg.ang_sp
        lookahead = msg.lookahead_dist
        lookahead_time = np.inf if lin_sp == 0 else  lookahead/lin_sp
        cv2.putText(img_bgr, 'lookahead: {:.2f}m {:.2f}s'.format(lookahead, lookahead_time), (20, 140), f, h2, c, w)
        cv2.putText(img_bgr, 'lin:  sp/odom {:.2f}/{:.2f} m/s'.format(lin_sp, lin_odom), (20, 190), f, h2, c, w)
        cv2.putText(img_bgr, 'ang: sp/odom {: 6.2f}/{: 6.2f} deg/s'.format(np.rad2deg(ang_sp), np.rad2deg(ang_odom)), (20, 240), f, h2, c, w)

        
        R, carrot = msg.R, [msg.carrot_x, msg.carrot_y]
        #self.lane_model.
        model.lane_model.coefs = msg.poly
        #if self.lane_model.is_valid() and self.display_mode not in [Contour1Pipeline.show_be]:
        x_min, x_max = 0.11, 0.3#model.lane_model.x_min, model.lane_model.x_max
        model.lane_model.draw_on_cam_img(img_bgr, model.cam, l0=x_min, l1=x_max, color=(0,128,0))
        carrot_lfp = np.array([[msg.lookahead_dist, model.lane_model.get_y(msg.lookahead_dist), 0]])
        carrot_img = model.cam.project(carrot_lfp).astype(int).squeeze()
        radius, color, thickness = 3, (255,0,0), 2
        try:
            image = cv2.circle(img_bgr, tuple(carrot_img), radius, color, thickness)
        except OverflowError:
            pass

class Node(cv_rpu.PeriodicNode):

    def __init__(self):
        cv_rpu.PeriodicNode.__init__(self, 'guidance_display_node')
        robot_name = rospy.get_param('~robot_name', '')
        def prefix(robot_name, what): return what if robot_name == '' else '{}/{}'.format(robot_name, what)
        cam_name = rospy.get_param('~camera', prefix(robot_name, 'camera_road_front'))
        ref_frame = rospy.get_param('~ref_frame', prefix(robot_name, 'base_link_footprint'))
        rospy.loginfo(' using ref_frame: {}'.format(ref_frame))
        self.cam = cv_rpu.retrieve_cam(cam_name, fetch_extrinsics=True, world=ref_frame)
        #FIXME
        pkg_dir = rospkg.RosPack().get_path('trilosaurus_bringup')
        extr_cam_calib_path = os.path.join(pkg_dir, 'cfg/camera1_extrinsics.yaml')
        self.cam.load_extrinsics(extr_cam_calib_path)
        
        self.lane_model = cv_u.LaneModel()
        self.img_pub = ImgPublisher("/guidance/image_debug", cam_name)
        self.guid_stat_sub = trr_rpu.GuidanceStatusSubscriber(topic='/guidance/status', timeout=0.5)
        self.lane_model = cv_u.LaneModel()
        #self.lane_model_sub = cv_rpu.LaneModelSubscriber('/vision/lane/model', timeout=0.15)

        

    def periodic(self):
        try:
            self.img_pub.publish(self, None)
        except trr_rpu.NoRXMsgException :
            print('guidance display im: no Status received from guidance')
        except trr_rpu.RXMsgTimeoutException :
            print('guidance display im: timeout receiving Status from guidance')
     
def main(args):
  rospy.init_node('guidance_display_node')
  Node().run(10)


if __name__ == '__main__':
    main(sys.argv)
