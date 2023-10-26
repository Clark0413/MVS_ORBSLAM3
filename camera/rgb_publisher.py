#!python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np

def publish_video():
    rospy.init_node('video_publisher', anonymous=True)

    image_pub = rospy.Publisher('/image_rgb', Image, queue_size=10)
    

    cap = cv2.VideoCapture(0)  
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    bridge = CvBridge()
    
    rate = rospy.Rate(20)  # fps

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:

            header = Header()
            header.stamp = rospy.Time.now()
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # image_msg = bridge.cv2_to_imgmsg(frame, encoding="mono8")
            image_msg.header = header
            
            image_pub.publish(image_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass
