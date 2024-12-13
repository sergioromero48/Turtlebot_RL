#python libraries for ros and gazebo
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose


#define a class that inherits from the ros node class, node fetches pose from gazebo
class EntityStateClient(Node):
    def __init__(self):
        super().__init__('entity_state_client')
        self.callback_group = ReentrantCallbackGroup()
        
        #create a service client too use the get_entity_state gazebo service too fetch robot pose
        self.client = self.create_client(GetEntityState, '/get_entity_state', callback_group=self.callback_group)
        
        #create a timer too periodically us3e the get_entity_state service every 0.5s
        self.timer = self.create_timer(0.10, self.timer_callback, callback_group=self.callback_group)
        
        #create a ros publisher too publish robot pose from get entity response too small-boi topic
        self.pub = self.create_publisher(Pose,'/waffle_pose',10)
        
        #create a get entity request too send request message too the get entity service server
        self.req = GetEntityState.Request()
        self.req.name = 'waffle'
        self.req.reference_frame = 'world'


    #define a timer callback function to be called every 0.5s, callback sends get entity request and fetches get_entity response
    def timer_callback(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            return

        future = self.client.call_async(self.req)
        future.add_done_callback(self.callback_done)

    #callback_done function handles the get entity response if the request to the get entity service is sucessful
    #This extracts the pose of the robot from the response and publishes in a topic called small-boi-pose
    def callback_done(self, future):
        try:
            response = future.result()
            if response.success:
                pose = response.state.pose
                self.pub.publish(pose)
            else:
                self.get_logger().warn("Failed to get entity state")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

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