import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray,Int32MultiArray,Bool
# Define ROS node and initialize cv_bridge
rospy.init_node('circle_detector')
bridge = CvBridge()

# Define default values for circle center and radius
x = 0
y = 0
radius = 0

# Define ROS subscriber to receive images from topic
def image_callback(msg):
    global x,y,radius
    # Convert ROS image message to OpenCV image
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Step 2: Filter the received image to remove noise
    filtered = cv2.GaussianBlur(img, (1,1), 0)

    # Step 3: Enhance the filtered image to be robust to lighting changes
    gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
    # step 3.1: equalize the histogram to improve the contrast
    gray = cv2.equalizeHist(gray)
    #use adaptive thresholding to get a binary image
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 9, 12)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,1))
    opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    circles=cv2.HoughCircles(closed,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=20,minRadius=0,maxRadius=0)
    if circles is not None:
        #publish that ball is detected
        BallFlagMsg=Bool()
        BallFlagMsg.data=True
        #end of publish
        #largest[0]-->x    largest[1]--->y
        circles = np.round(circles[0, :]).astype("int")
        largest_circle = max(circles, key=lambda circle: circle[2])
        center = (largest_circle[0], largest_circle[1])
        radius = largest_circle[2]
        # draw the circle around the ball on the frame
        cv2.circle(img, center, radius, (0,255,0), 2)
        cv2.circle(img, center, 2, (0,0,255), 3)
        error = (center[0] - (img.shape[1] / 2))
        last_error=[radius,error]
    else:
        #publish that ball is not detected
        BallFlagMsg=Bool()
        BallFlagMsg.data=False
    # Display the image
    cv2.imshow("CameraStream", img)
    cv2.waitKey(3)

# Define ROS subscriber to receive images from topic
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Run ROS node
rospy.spin()
