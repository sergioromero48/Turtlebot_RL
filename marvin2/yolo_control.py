import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # To receive annotated image data
from std_msgs.msg import String  # To receive detection data
from geometry_msgs.msg import Twist  # For velocity commands
import json  # For JSON parsing
import numpy as np  # For efficient numerical calculations
import cv2  # OpenCV for drawing and displaying
from cv_bridge import CvBridge  # For converting between ROS and OpenCV images


class TrackerNode(Node):
    """
    A ROS 2 Node that tracks objects detected by YOLO and uses pixel distance
    to compute velocity commands for a robot.
    """
    def __init__(self, screen_width=640, screen_height=480):
        """
        Class constructor to set up the node.
        :param screen_width: Width of the screen or frame
        :param screen_height: Height of the screen or frame
        """
        super().__init__('tracker_node')
        
        # Screen center as a numpy array
        self.screen_center = np.array([screen_width / 2, screen_height / 2])
        self.screen_width = screen_width
        self.screen_height = screen_height

        # Bridge to convert ROS images to OpenCV
        self.br = CvBridge()

        # Current detections and frame
        self.current_detections = []
        self.current_frame = None

        # Velocity publisher
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Image publisher
        self.image_pub = self.create_publisher(Image, 'image_tracker', 10)

        # Subscribers
        self.create_subscription(String, '/camera/full_detections', self.detection_callback, 10)
        self.create_subscription(Image, '/camera/image_annotated', self.image_callback, 10)

    def detection_callback(self, msg):
        """
        Callback function that processes detection data.
        """
        try:
            full_results = json.loads(msg.data)
            self.current_detections = full_results.get("detections", [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON data: {e}")

    def image_callback(self, msg):
        """
        Callback function that processes the annotated image and performs visualization.
        """
        self.current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.current_detections:
            self.process_detections()

    def process_detections(self):
        """
        Process detections to calculate distances and control the robot.
        """
        closest_distance = float('inf')
        closest_centroid = None

        # Iterate through detections
        for detection in self.current_detections:
            boxes = detection.get("boxes", [])
            for box in boxes:
                x1, y1, x2, y2, confidence, class_id = box  # [x1, y1, x2, y2, confidence, class_id]
                
                # Filter for person detections with confidence > 80%
                if class_id == 11 and confidence > 0.80:
                    centroid = np.array([(x1 + x2) / 2, (y1 + y2) / 2])  # Centroid of the box
                    distance = np.linalg.norm(centroid - self.screen_center)  # Pixel distance

                    # Track the closest detection
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_centroid = centroid

                    # Draw bounding box and centroid
                    cv2.rectangle(self.current_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    cv2.circle(self.current_frame, (int(centroid[0]), int(centroid[1])), 5, (0, 255, 0), -1)

        if closest_centroid is not None:
            # Draw line to screen center
            cv2.line(self.current_frame,
                     (int(closest_centroid[0]), int(closest_centroid[1])),
                     (int(self.screen_center[0]), int(self.screen_center[1])),
                     (255, 255, 255), 2)

            # Publish control command
            self.publish_control(closest_centroid, closest_distance)

        # Publish the annotated image
        self.image_pub.publish(self.br.cv2_to_imgmsg(self.current_frame, "bgr8"))

    def publish_control(self, centroid, pixel_distance):
        """
        Compute and publish a Twist message based on the centroid's pixel offset.
        """
        # Pixel offset from the center
        x_offset = centroid[0] - self.screen_center[0]  # Horizontal offset
        y_offset = self.screen_center[1] - centroid[1]  # Vertical offset

        # Control gains
        k_angular = 0.002  # Gain for angular velocity
        k_linear = 0.005  # Gain for linear velocity
        desired_pixel_distance = 150  # Desired distance in pixels

        # Compute velocities
        angular_z = -k_angular * x_offset  # Negative because left/right inversion
        linear_x = k_linear * (pixel_distance - desired_pixel_distance)

        # Create and publish Twist message
        cmd = Twist()
        cmd.angular.z = angular_z
        cmd.linear.x = linear_x
        self.twist_pub.publish(cmd)

        # Log control info
        self.get_logger().info(f"Centroid Offset: x={x_offset:.2f}, y={y_offset:.2f}, Distance: {pixel_distance:.2f}")
        self.get_logger().info(f"Command: Linear.x={linear_x:.2f}, Angular.z={angular_z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    tracker_node = TrackerNode(screen_width=640, screen_height=480)
    rclpy.spin(tracker_node)
    tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
