import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion
from cv_bridge import CvBridge
from dt_apriltags import Detector
from tf2_ros import TransformListener, Buffer, TransformStamped
from scipy.spatial.transform import Rotation
import cv2
import tf2_geometry_msgs
import numpy as np



class ImageProcessor(Node):
    """
    ROS2 node that reads raw image stream from the camera, processes it to detect an Apriltag,
    and publishes a goal location in space for the robot arm to get to.
    """

    def __init__(self):
        super().__init__('image_processor')
        self.logger = self.get_logger()
        self.bridge = CvBridge()
        self.color_info_subscription = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.color_info_callback, 10)
        self.color_info_subscription  # prevent unused variable warning
        self.color_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.color_subscription  # prevent unused variable warning
        self.camera_info = None
        self.camera_info_received = False
        self.object_pose_publisher = self.create_publisher(Pose, '/object_3d_pose', 10)
        self.timer = self.create_timer(3.0, self.publish_pose)
        self.object_pose = None
        self.tf_buffer = Buffer(rclpy.time.Duration(seconds=1))
        self.tf_listener = TransformListener(self.tf_buffer, self)
                
    def color_callback(self, msg):
        if not self.camera_info_received or not self.camera_info:
            self.logger.warn("Waiting for camera info.")
            return
        
        if self.object_pose != None:
            self.logger.warn("Object pose has been determined, ignoring color_callback")
            return

        cv_image = None
        #families= 'tagStandard41h12', tag36h11
        at_detector = Detector(families='tag36h11', 
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.logger.error(f"Error retrieving the color image. {e}")
            return
        
        gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        
        camera_matrix = np.array(self.camera_info.k).reshape((3,3))
        camera_params = ( camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2] )
        tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.0238)  # 0.0238 for our tag36h11 tags, 0.012 for our tagStandard41h12 tags.
        
        if not tags:  # no apriltags detected
            self.logger.warn("No apriltags detected in image stream")
            return

        for tag in tags:  # loop should run once if there's only one apriltag in the image.
            new_pose = Pose()
            pose_translation = tag.pose_t
            pose_rotation_matrix = tag.pose_R
            # print(f"Tag id is {tag.tag_id}") # can compare tag_id with an expected value to decide whether to move forward with the pick_and_place task or not.
            pose_rotation_quaternion = None
            try:
                pose_rotation_quaternion = Rotation.from_matrix(pose_rotation_matrix).as_quat()
            except Exception as e:
                self.logger.error(f"Error getting pose rotation from apriltag. {e}")
                return
            try:
                new_pose.position.x = pose_translation[0][0]
                new_pose.position.y = pose_translation[1][0]
                new_pose.position.z = pose_translation[2][0]
                new_pose.orientation.w = pose_rotation_quaternion[0]
                new_pose.orientation.x = pose_rotation_quaternion[1]
                new_pose.orientation.y = pose_rotation_quaternion[2]
                new_pose.orientation.z = pose_rotation_quaternion[3]
            except Exception as e:
                self.logger.error(f"Error extracting pose information from apriltag. {e}")
            
            try:
                self.object_pose = new_pose
            except Exception as e:
                self.logger.error(f"Error setting object pose in color_callback. {e}")
            break  # only handling one april tag at a time for now.
        
    def color_info_callback(self,  msg):
        try:
            if not self.camera_info and not self.camera_info_received:
                self.camera_info = msg
                self.camera_info_received = True
        except Exception as e:
            self.logger.error(f"Error getting camera info. {e}")

    def publish_pose(self):
        if self.object_pose != None:
            # transform the pose to robot base frame
            try:
                cam_to_base_transform = self.tf_buffer.lookup_transform("base_link", "camera_color_frame", rclpy.time.Time(), rclpy.time.Duration(seconds=1))
                transformed_pose = tf2_geometry_msgs.do_transform_pose(self.object_pose, cam_to_base_transform)
                transformed_pose.position.z = 0.0
                transformed_pose.position.x -= 0.02
                transformed_pose.orientation = Quaternion()
                # transformed_pose = Pose()
                self.object_pose_publisher.publish(transformed_pose)
                self.logger.info(f"Object detected with Pose {transformed_pose}")
            except Exception as e:
                self.logger.error(f"Error with pose transformation. {e}")
        else:
            self.logger.warn("Waiting for object pose.")


def main():
    rclpy.init()
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
