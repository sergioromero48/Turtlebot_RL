import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from rclpy.node import Node
from geometry_msgs.msg import Pose
import random


class SpawnServiceClient(Node):
    def __init__(self):
        super().__init__('spawn_service_client')

        # Service clients for spawning and deleting markers
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Ensure services are available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity service...')

        # Marker parameters
        self.marker_name = 'marker'
        self.marker_path = "/home/sergio/ros2_ws/src/marvin2/models/marker.sdf"
        self.xrange = 2.0
        self.yrange = 2.0

        # State management
        self.is_operating = False
        self.failure_count = 0
        self.max_retries = 3

        # Timer to control the respawn cycle (every 30 seconds)
        self.timer = self.create_timer(30.0, self.respawn_marker)

        # Initial spawn
        self.spawn_marker()

    def spawn_marker(self):
        """Request to spawn the marker."""
        if self.is_operating:
            self.get_logger().warn("Already operating. Spawn request ignored.")
            return

        self.is_operating = True

        # Create a new SpawnEntity.Request
        spawn_req = SpawnEntity.Request()
        spawn_req.name = self.marker_name

        # Load marker SDF file
        try:
            with open(self.marker_path, 'r') as file:
                spawn_req.xml = file.read()
        except IOError as e:
            self.get_logger().error(f"Failed to read marker SDF file: {e}")
            self.is_operating = False
            return

        spawn_req.reference_frame = 'world'

        # Generate random spawn position
        new_pose = Pose()
        new_pose.position.x = 2 * (random.random() - 0.5) * self.xrange
        new_pose.position.y = 2 * (random.random() - 0.5) * self.yrange
        new_pose.position.z = 0.10
        new_pose.orientation.w = 1.0
        spawn_req.initial_pose = new_pose

        self.get_logger().info(f"Spawning marker at ({new_pose.position.x:.2f}, {new_pose.position.y:.2f})")

        # Call spawn service asynchronously
        future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(lambda fut: self.spawn_done_callback(fut, new_pose))

    def spawn_done_callback(self, future, pose):
        """Callback for spawn service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Marker successfully spawned at ({pose.position.x:.2f}, {pose.position.y:.2f})")
                self.failure_count = 0
            else:
                self.get_logger().error(f"Failed to spawn marker: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Spawn service call failed: {e}")
        finally:
            self.is_operating = False

    def delete_marker(self):
        """Request to delete the marker."""
        if self.is_operating:
            self.get_logger().warn("Already operating. Delete request ignored.")
            return

        self.is_operating = True

        # Create a new DeleteEntity.Request
        delete_req = DeleteEntity.Request()
        delete_req.name = self.marker_name

        self.get_logger().info("Requesting to delete marker.")

        # Call delete service asynchronously
        future = self.delete_client.call_async(delete_req)
        future.add_done_callback(self.delete_done_callback)

    def delete_done_callback(self, future):
        """Callback for delete service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Marker successfully deleted.")
                # Schedule spawn after deletion
                self.create_timer(1.0, self.spawn_marker)  # Delay spawn by 1 second
            else:
                self.get_logger().warn(f"Failed to delete marker: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Delete service call failed: {e}")
        finally:
            self.is_operating = False

    def respawn_marker(self):
        """Delete and respawn the marker."""
        if self.is_operating:
            self.get_logger().warn("Respawn already in progress. Ignoring request.")
            return

        self.get_logger().info("Respawning marker: Deleting existing marker.")
        self.delete_marker()


def main(args=None):
    rclpy.init(args=args)
    spawn_service_client = SpawnServiceClient()
    try:
        rclpy.spin(spawn_service_client)
    except KeyboardInterrupt:
        spawn_service_client.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        spawn_service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
