#!/usr/bin/env python3

import sys, numpy as np, rospy
import cv2


import common_vision.rospy_utils as cv_rpu
import common_vision.utils as cv_u

class NonePipeline(cv_u.Pipeline):
    show_none = 0
    def __init__(self, cam, robot_name):
        self.cam = cam
        self.img = None
        cv_u.Pipeline.__init__(self)
        self.display_mode = 1

        #be_param = trr_vu.NamedBirdEyeParam(robot_name)
        #self.bird_eye = trr_vu.BirdEyeTransformer(cam, be_param)
        
    def _process_image(self, img, cam, stamp):
        self.img = img
        #self.img = self.bird_eye.process(img)

    def draw_debug_bgr(self, cam, img_cam=None):
        if self.img is None:
            return np.zeros((cam.h, cam.w, 3), dtype=np.uint8)
        else:
            debug_img = self.img.copy()#cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
            #cv2.rectangle(debug_img, tuple(self.tl), tuple(self.br), color=(0, 0, 255), thickness=3)
            #if len(self.ids)>0: cv2.aruco.drawDetectedMarkers(debug_img, self.corners, self.ids)
            self.draw_timing(debug_img)
            return debug_img

        
class Node(cv_rpu.SimpleVisionPipeNode):

    def __init__(self):
        # will load a camera (~camera and ~ref_frame)
        cv_rpu.SimpleVisionPipeNode.__init__(self, NonePipeline, self.pipe_cbk, fetch_extrinsics=False)
        self.img_pub = cv_rpu.CompressedImgPublisher(self.cam, '/vision/test_node')
        #self.img_pub2 = ImgPublisher("/vision/test_node", self.cam_name)
        self.start()

    def pipe_cbk(self):
        #self.img_pub.publish(self, self.cam, "bgr8")
        pass

    def periodic(self):
        #print('proc: {:.1f}ms'.format(self.pipeline.lp_proc*1e3))
        if self.pipeline.display_mode != self.pipeline.show_none:
            self.img_pub.publish(self.pipeline, self.cam)
            #self.img_pub2.publish(self, None)

    def draw_debug(self, cam, img_cam=None):
        return self.pipeline.img


def main(args):
    name = 'trilosaurus_test_node'
    rospy.init_node(name)
    rospy.loginfo('{} starting'.format(name))
    rospy.loginfo('  using opencv version {}'.format(cv2.__version__))
    Node().run(low_freq=3)


if __name__ == '__main__':
    main(sys.argv)
