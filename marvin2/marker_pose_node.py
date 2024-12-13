import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose


class EntityStateClient(Node):
    def __init__(self):
        super().__init__('entity_state_client')
        self.callback_group = ReentrantCallbackGroup()
        
        self.client = self.create_client(GetEntityState, '/get_entity_state', callback_group=self.callback_group)
        self.timer = self.create_timer(0.10, self.timer_callback, callback_group=self.callback_group)
        self.pub = self.create_publisher(Pose, '/marker_pose', 10)
        
        self.req = GetEntityState.Request()
        self.req.name = 'marker'
        self.req.reference_frame = 'world'

        # State to prevent overlapping requests
        self.is_fetching = False
        # Count consecutive failures to enable retries or recovery
        self.failure_count = 0
        self.max_retries = 3  # Adjust based on tolerance

    def timer_callback(self):
        # Skip new request if one is already in progress
        if self.is_fetching:
            self.get_logger().debug("Skipping request, still processing previous one.")
            return
        
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, retrying...")
            return

        self.is_fetching = True
        future = self.client.call_async(self.req)
        future.add_done_callback(self.callback_done)

    def callback_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.failure_count = 0  # Reset failure count on success
                pose = response.state.pose
                self.pub.publish(pose)
                self.get_logger().debug(f"Marker pose published: {pose.position.x}, {pose.position.y}, {pose.position.z}")
            else:
                self.failure_count += 1
                if self.failure_count > self.max_retries:
                    self.get_logger().error(f"Failed to get marker pose after {self.max_retries} retries.")
                else:
                    self.get_logger().warn(f"Failed to get marker state (attempt {self.failure_count}/{self.max_retries}).")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        finally:
            self.is_fetching = False

def main(args=None):
    rclpy.init(args=args)
    client = EntityStateClient()
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    try:
        executor.spin()
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
