import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import math

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_control')
        
        self.image = None
        self.last_detection_list = []
        self.past_dist_to_tag = 0.0
        self.past_angle_to_tag = 0.0

        # Transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CV Bridge
        self.br = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'imagetimer', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.create_subscription(
            Image, '/camera/image_raw', self.callback_to_get_image_raw, 10)
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self.callback_to_get_tag_detections, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.process)

    def callback_to_get_image_raw(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def callback_to_get_tag_detections(self, msg):
        # Just store the detections to know that a tag is detected
        self.last_detection_list = msg.detections

    def process(self):
        # If we have no detections, just publish the image if available
        if not self.last_detection_list:
            if self.image is not None:
                self.image_pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            return

        # Get the transform from base_link to taggy
        try:
            t_base_link_to_tag = self.tf_buffer.lookup_transform(
                'camera_link', 'tag36h11:1', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Extract relative position
        x_dist = t_base_link_to_tag.transform.translation.x
        y_dist = t_base_link_to_tag.transform.translation.y

        # Compute angle and distance
        angle_to_tag = math.atan(y_dist / x_dist)
        dist_to_tag = math.sqrt(x_dist**2 + y_dist**2)

        # Compute rate of change if needed
        delta_angle_to_tag = (angle_to_tag - self.past_angle_to_tag)
        delta_dist_to_tag = (dist_to_tag - self.past_dist_to_tag) / 0.1

        # Update past values
        self.past_angle_to_tag = angle_to_tag
        self.past_dist_to_tag = dist_to_tag

        # Just log the distance and angle
        self.get_logger().info(f"Distance to Tag: {dist_to_tag:.2f}, Angle to Tag: {math.degrees(angle_to_tag):.2f} deg")

        cmd = Twist()
        # Simple controller
        desired_standoff = 1.0
        cmd.angular.z = (0.05 * angle_to_tag + 0.2 * delta_angle_to_tag)
        cmd.linear.x = 0.5 * (dist_to_tag - desired_standoff) + 0.05 * delta_dist_to_tag
        self.twist_pub.publish(cmd)

        # Publish image if available
        if self.image is not None:
            self.image_pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
