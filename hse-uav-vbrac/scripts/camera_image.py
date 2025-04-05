import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Callback функция для обработки ROS Image сообщения
def image_callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")  # Преобразование ROS Image сообщения в изображение OpenCV
    cv2.imshow("Camera Image", cv_image)
    cv2.waitKey(1)

rospy.init_node('image_listener')
image_sub = rospy.Subscriber('/s_camera_2/s_image_raw', Image, image_callback)  # Подписка на топик с изображением с камеры

rospy.spin()
