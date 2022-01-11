#!/usr/bin/env python
import sys
import rospy
import message_filters
from sensor_msgs.msg import Image
from yolov3_pytorch_ros.msg import BoundingBoxes
from yolov3_pytorch_ros.srv import GetMoveGoal

class person_navigation():
    def __init__(self):
        rospy.wait_for_service('get_move_goal')
        # set sub
        depth_image_sub = message_filters.Subscriber('/kinect2/hd/image_depth_rect', Image)
        detected_objects_topic = rospy.get_param('~detected_objects_topic', 'detected_objects_in_image')
        detected_objects_sub =  message_filters.Subscriber(detected_objects_topic, BoundingBoxes)
        ts = message_filters.TimeSynchronizer([depth_image_sub, detected_objects_sub], 10)
        ts.registerCallback(self.callback)


    def callback(self, depth_image, boxes):
        for box in boxes.bounding_boxes:
            det_class = box.Class
            if det_class >= 0:
                # == 0 means being person
                try:
                    get_move_goal = rospy.ServiceProxy('get_move_goal', GetMoveGoal)
                    ret = get_move_goal(depth_image, box)
                    return ret
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

if __name__ == "__main__":
    node = person_navigation()
    rospy.init_node('person_navigation', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")