#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from yolov3_pytorch_ros.srv import GetMoveGoal
camera_factor=1000
cx=325.5
cy=253.5
fx=518.0
fy=519.0

class get_move_goal:
    def __init__(self):
        # subscribe topics
        #self.depth_image_topic = '/kinect2/hd/image_depth_rect'
        #self.detected_objects_topic = rospy.get_param('~detected_objects_topic', 'detected_objects_in_image')
        #self.
        # publish topics

        # cv bridge
        self.bridge = CvBridge()

        # set server
        self.server = rospy.Service('get_move_goal', GetMoveGoal, self.get_avg_depth)

        
    
    def get_avg_depth(self, req):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(req.depth_image, "16UC1")
        except CvBridgeError as e:
            print(e)
        box = req.box
        # print(depth_image.shape) (h, w)-->(y,x)
        roi = depth_image[box.ymin:box.ymax, box.xmin:box.xmax]
        avg_depth = roi.mean()
        return avg_depth

if __name__ == "__main__":
    node = get_move_goal()
    rospy.init_node('get_move_goal', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")