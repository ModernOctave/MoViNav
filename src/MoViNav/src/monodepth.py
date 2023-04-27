#!/usr/bin/env python3.8

import time
import numpy as np
import cv2
import rospkg
import rospy
import torch


from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf2_msgs.msg import TFMessage

from predict import predict


class MonoDepth():
    def __init__(self):
        # Setup pytorch
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Get parameters
        self.debug = rospy.get_param("~debug", False)
        self.frame_id = rospy.get_param("~frame_id", "camera_depth")

        self.topic_color = rospy.get_param("~topic_color", "/iris_0/stereo_camera/left/image_raw")
        self.topic_color_image = rospy.get_param("~topic_color_image", "/camera/image_raw")
        self.camera_info_color = rospy.get_param("~camera_info_color", "/camera/camera_info")
        self.camera_info = rospy.get_param("~camera_info", "/camera_info")
        self.topic_depth = rospy.get_param("~topic_depth", "/iris_0/depth")
        self.topic_pointcloud = rospy.get_param("~topic_pointcloud", "/iris_0/pointcloud")

        self.min_depth = rospy.get_param("~min_depth", 10)
        self.max_depth = rospy.get_param("~max_depth", 1000)
        self.batch_size = rospy.get_param("~batch_size", 1)
        self.model_file = rospy.get_param("~model_file", "/models/densedepth.pth")

    # predictions = model.predict(image, batch_size=batch_size)
        # Read keras model
        self.rospack = rospkg.RosPack()
        self.model_path = self.rospack.get_path("movinav") + self.model_file
        
        # Load model into GPU / CPU
        self.model = torch.load(self.model_path)
        self.model.eval()
        self.model.to(self.device)

        # Publishers
        self.pub_image_depth = rospy.Publisher(self.topic_depth, Image, queue_size=1)
        self.pub_pointcloud = rospy.Publisher(self.topic_pointcloud, PointCloud2, queue_size=1)
        self.counter = 0

        # Subscribers
        # self.bridge = CvBridge()
        self.sub_color = rospy.Subscriber(self.topic_color, Image, self.image_callback,queue_size=1)
        self.sub_input_camera_info = rospy.Subscriber(self.camera_info, CameraInfo, self.cb_camera_info,queue_size=1)
        self.cam_info = CameraInfo()
        rospy.Subscriber('/tf', TFMessage, self.cb_tf)
    
    def cb_tf(self,data):
        self.tf = data

    def cb_camera_info(self, data):
        self.cam_info = data

    # Create a sensor_msgs.PointCloud2 from the depth and color images provided
    #
    # It ignores are camera parameters and assumes the images to be rectified
    def create_pointcloud_msg(self, depth, color):
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.header.seq = self.counter

        height, width, c = depth.shape

        # Resize color to match depth
        img = cv2.resize(color, (width, height))

        # Point cloud data numpy array
        i = 0
        # Message data size
        msg.height = 1
        msg.width = width * height

        data = np.zeros((height * width * 6), dtype=np.float32)
        depth = depth/255*self.max_depth/100

        img = img.astype('float')
        w_v = np.arange(width,dtype=np.float64)
        h_v = np.arange(height,dtype=np.float64)
        mat_A = np.broadcast_to(w_v,(height,len(w_v)))
        mat_B = np.broadcast_to(h_v,(width,len(h_v))).T
        mat_A = (-(mat_A) + width/2)/(width / 2)
        mat_B = (-(mat_B) + height/2)/(width / 2)
        mat_A = np.multiply(mat_A, depth[0].T)
        mat_B = np.multiply(mat_B, depth[0].T)
        data = np.dstack((mat_A,mat_B,depth,img/255))
        data = data.flatten()
        data = data.reshape(data.shape[0],1)
        data = data.astype('float32')

        # Fields of the point cloud
        msg.fields = [
            PointField("y", 0, PointField.FLOAT32, 1),
            PointField("z", 4, PointField.FLOAT32, 1),
            PointField("x", 8, PointField.FLOAT32, 1),
            PointField("b", 12, PointField.FLOAT32, 1),
            PointField("g", 16, PointField.FLOAT32, 1),
            PointField("r", 20, PointField.FLOAT32, 1)
        ]

        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * height * width
        msg.is_dense = True
        msg.data = data.tostring()

        return msg

    # Callback to receive and process image published.
    #
    # After processing it publishes back the estimated depth result
    def image_callback(self, msg):
        # Convert message to opencv image
        try:
            # image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1).astype(np.uint8)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            print(e)

        # Display image
        if self.debug:
            cv2.imshow("Image", image)
            cv2.waitKey(1)

        # Get image data as a numpy array to be passed for processing.
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (640, 480))
        arr = np.clip(np.asarray(image, dtype=np.float32) / 255, 0, 1)

        # Predict depth image
        result = predict(self.model, self.device, arr, self.min_depth, self.max_depth, self.batch_size)

        # Resize and reshape output
        depth = result.reshape(result.shape[1], result.shape[2], 1)

        # Display depth
        if self.debug:
            cv2.imshow("Result", depth)
            cv2.waitKey(1)

        # Publish depth image
        depth = 255 * depth
        depth_pc = depth
        depth = cv2.resize(depth,(640,480))
        depth_image = Image()
        # depth_image = self.bridge.cv2_to_imgmsg(depth.astype(np.uint16))
        depth_image.height = depth.shape[0]
        depth_image.width = depth.shape[1]
        depth_image.encoding = '32FC1'
        depth_image.is_bigendian = False
        depth_image.data = depth.astype(np.float32).tobytes()
        depth_image.step = depth_image.width * 4
        # depth_image.header.frame_id = 'iris_0/depth_camera_base'
        depth_image.header.frame_id = 'camera'
        depth_image.header.stamp = msg.header.stamp
        self.cam_info.header.stamp = msg.header.stamp

        self.pub_image_depth.publish(depth_image)

        # Generate Point cloud
        cloud_msg = self.create_pointcloud_msg(depth_pc, image)
        # cloud_msg.header.stamp = self.tf_msg.transforms[0].header.stamp
        # cloud_msg.header.stamp = msg.header.stamp
        self.pub_pointcloud.publish(cloud_msg)
        print(self.counter)

        # Increment counter
        self.counter += 1

def main():
    rospy.init_node("monodepth")

    depth = MonoDepth()

    rospy.spin()

if __name__ == "__main__":
    main()
