#! /usr/bin/python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import apriltag

print("I am running")
# Image call back function. It takes the CompressedImage message and returns a CV2 image
def image_callback(msg):
    # CVbridge does not support CompressedImage type data therefore, numpy is used.
    # Alternatively, frombuffer can also be used.
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    gray = color_to_gray(image_np)

def color_to_gray(color_image):
    print("3")
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    tags = detector(gray, color_image)

def detector(gray_image, image):
    print("[INFO] detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray_image)
    print("[INFO] {} total AprilTags detected".format(len(results)))

    for r in results:
        print("tag ID: ", r.tag_id)
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        # tagFamily = r.tag_family.decode("utf-8")
        tagID = r.tag_id
        cv2.putText(image, str(tagID), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)       
        # print("[INFO] tag family: {}".format(tagFamily))
        # show the output image after AprilTag detection
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    

# main function to initilize image_listener node that subscribes to the camera topic
def main():
    rospy.init_node('image_listener')
    # subscriber function, first argument is the topic, second argument is the topic message type
    # and the last argument is the callback function
    rospy.Subscriber("/csc22906/camera_node/image/compressed", CompressedImage, image_callback)
    # rospy.spin() makes sure that the node keeps running until the program is manually killed.
    rospy.spin()


if __name__ == '__main__':
    main()
