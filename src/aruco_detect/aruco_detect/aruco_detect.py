import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import math

def quaternion_from_rotation_vector(rvec):
    """
    Convert rotation vector to quaternion
    :param rvec: rotation vector, e.g. [1,2,3]
    :return: quaternion, [x,y,z,w]
    """
    theta = np.linalg.norm(rvec)
    if theta < 1e-6:
        return np.array([0, 0, 0, 1], dtype=np.float)
    else:
        axis = rvec / theta
        return np.array([np.sin(theta / 2) * axis[0], np.sin(theta / 2) * axis[1],
                         np.sin(theta / 2) * axis[2], np.cos(theta / 2)], dtype=np.float)


class DetectAruco(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        self.bridge = CvBridge()

        self.get_logger().info('aruco_detect node started')

        self.declare_parameter("image_topic", "/camera/color/image_raw", ParameterDescriptor(
            name="image_topic", description="image topic"))
        
        self.declare_parameter("image_info_topic", "/camera/color/camera_info", ParameterDescriptor(
            name="camera_info_topic", description="camera info topic"))
        
        self.declare_parameter("view_image", True, ParameterDescriptor(
            name="view_image", description="whether to show the detect result in a window"))
        
        self.declare_parameter("aruco_id", 2, ParameterDescriptor(
            name="aruco_id", description="aruco marker id to detect"))
        
        self.declare_parameter("aruco_length", 0.03, ParameterDescriptor(
            name="aruco_length", description="marker size, unit: [m]"))
 
        self.declare_parameter("camera_color_optical_frame", "camera_color_optical_frame", ParameterDescriptor(
            name="camera_color_optical_frame", description="frame id of the camera color optical frame"))
        
        self.declare_parameter("aruco_marker_frame", "aruco_marker_frame", ParameterDescriptor(
            name="aruco_marker_frame", description="frame id of the aruco marker"))

        # subscribe to camera info
        image_info_topic = self.get_parameter("image_info_topic").value
        self.image_info = None
        self.image_info_sub = self.create_subscription(CameraInfo, image_info_topic, self.image_info_callback, 10)

        # subscribe to camera image
        image_topic = self.get_parameter("image_topic").value
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.image = Image()
        
        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcast = StaticTransformBroadcaster(self)
        
        self.view_img = self.get_parameter("view_image").value  # whether to show the detect result in a window
        
        self.aruco_id = self.get_parameter("aruco_id").value    # aruco marker id to detect
        self.aruco_length = self.get_parameter("aruco_length").value  # marker size, unit: [m]
        
        self.camera_color_optical_frame = self.get_parameter("camera_color_optical_frame").value
        self.aruco_marker_frame = self.get_parameter("aruco_marker_frame").value
        
        
    def image_info_callback(self, msg:CameraInfo):
        self.image_info = msg

    def image_callback(self, msg:Image):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_draw = self.image.copy() # for drawing

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100) # 5x5 aruco marker, 100 ids
        aruco_params = cv2.aruco.DetectorParameters_create()    # default parameters
        # detect aruco marker
        # corners: list of 4 corners of each marker
        # ids: list of ids of each marker
        corners, ids, rejected = cv2.aruco.detectMarkers(self.image, aruco_dict, parameters=aruco_params) 

        if ids is not None and self.image_info is not None:
            marker_border_color = (0, 255, 0)
            cv2.aruco.drawDetectedMarkers(img_draw, corners, ids, marker_border_color)
            camera_intrinsic = np.array(self.image_info.k, dtype=np.float32).reshape((3, 3))
            distortion_parameter = np.array(self.image_info.d, dtype=np.float32).reshape((1, 5))
            
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_length, camera_intrinsic, distortion_parameter)
            cv2.aruco.drawAxis(img_draw, camera_intrinsic, distortion_parameter, rvec, tvec, 0.03)
            
            # only publish tf of the specified aruco marker
            if self.aruco_id in ids:
                idx = np.where(ids == self.aruco_id)[0][0]
                rvec = rvec[idx] # rotation vector, shape: (1, 3), e.g. [[-2.76186719  0.23810233 -0.76484692]]
                tvec = tvec[idx] # translation vector, shape: (1, 3), e.g. [[-0.0084606  -0.01231327  0.26638986]]
                rvec = np.squeeze(rvec) # shape: (3, ), e.g. [-2.76186719  0.23810233 -0.76484692]
                tvec = np.squeeze(tvec) # shape: (3, ), e.g. [-0.0084606  -0.01231327  0.26638986]
                # convert rotation vector to quaternion
                quat = quaternion_from_rotation_vector(rvec)
                # prepare tf message
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.camera_color_optical_frame
                t.child_frame_id = self.aruco_marker_frame
                
                t.transform.translation.x = tvec[0]
                t.transform.translation.y = tvec[1]
                t.transform.translation.z = tvec[2]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                
                # send tf
                self.tf_broadcaster.sendTransform(t)
                
        if self.view_img:
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('image', img_draw.shape[1], img_draw.shape[0])
            cv2.imshow('image', img_draw)
            cv2.waitKey(1)



def main():
    rclpy.init()
    node = DetectAruco()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


# baselink to camera_color_optical_frame
# x = 6.2 cm
# y = 3.2 cm
# z = 15-1.25 = 13.75 cm
