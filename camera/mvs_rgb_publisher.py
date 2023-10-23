import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np

def publish_video():
    rospy.init_node('video_publisher', anonymous=True)

    mvs_pub = rospy.Publisher('/image_edge', Image, queue_size=10)
    rgb_pub = rospy.Publisher('/image_rgb', Image, queue_size=10)
    
    # 打开摄像头
    cap_rgb = cv2.VideoCapture(0)  # 0表示默认的摄像头
    cap_mvs = cv2.VideoCapture(2)  # 0表示默认的摄像头
    cap_rgb.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_rgb.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap_mvs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_mvs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    bridge = CvBridge()
    
    rate = rospy.Rate(20)  # fps
    # kernel = np.ones((3,3), np.uint8)

    while not rospy.is_shutdown():
        ret_rgb, frame_rgb = cap_rgb.read()
        ret_mvs, frame_mvs = cap_mvs.read()
        if ret_mvs:

            frame = cv2.cvtColor(frame_mvs, cv2.COLOR_BGR2YUV)[:, :, 0]
            # 将OpenCV图像转换为ROS图像消息
            header = Header()
            header.stamp = rospy.Time.now()
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="mono8")
            image_msg.header = header
            
            # 发布图像消息到指定话题
            mvs_pub.publish(image_msg)
        else:
            print("mvs err")

        if ret_rgb:

            # 将OpenCV图像转换为ROS图像消息
            header = Header()
            header.stamp = rospy.Time.now()
            image_msg = bridge.cv2_to_imgmsg(frame_rgb, encoding="bgr8")
            image_msg.header = header
            
            # 发布图像消息到指定话题
            rgb_pub.publish(image_msg)
        else:
            print("rgb err")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass
