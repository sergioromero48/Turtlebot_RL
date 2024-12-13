import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
import random
import os

class QLearningNavigator(Node):
    def __init__(self):
        super().__init__('q_learning_navigator')

        # File path for Q-table persistence
        self.q_table_file = '/tmp/q_table.npy'

        # Actions: forward, back, left, right, stop
        self.actions = [
            (0.5, 0.0),   # forward
            (-0.5, 0.0),  # back
            (0.0, 0.2),   # left
            (0.0, -0.2),  # right
            (0.0, 0.0)    # stop
        ]

        # Distance thresholds
        self.distance_thresholds = [0.2]
        increment = (3.0 - 0.2) / 9.0
        for i in range(1,10):
            self.distance_thresholds.append(0.2 + i*increment)

        # Bearing thresholds
        self.bearing_thresholds = [0.1]
        bearing_increment = (math.pi - 0.1)/9.0
        for i in range(1,10):
            self.bearing_thresholds.append(0.1 + i*bearing_increment)

        # State space: 11 bins for distance (0-10) and 11 bins for bearing (0-10)
        self.num_distance_bins = 11
        self.num_bearing_bins = 11
        self.num_states = self.num_distance_bins * self.num_bearing_bins
        self.num_actions = len(self.actions)

        # Load Q-table if exists, otherwise initialize a new one
        self.Q = self.load_q_table()

        # Q-learning hyperparameters
        self.alpha = 0.1   # learning rate
        self.gamma = 0.99 # discount factor
        self.epsilon = 0.15 # exploration rate

        # Publisher for robot velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers to poses published by other nodes
        self.marker_pose = None
        self.robot_pose = None
        self.marker_sub = self.create_subscription(Pose, '/marker_pose', self.marker_pose_callback, 10)
        self.robot_sub = self.create_subscription(Pose, '/waffle_pose', self.robot_pose_callback, 10)

        self.current_state = None
        self.current_action = None

        # Control loop timer at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.goal_threshold = 0.1  # Distance at which we consider the goal reached

    def load_q_table(self):
        """Load Q-table from file if it exists, otherwise return a new Q-table."""
        if os.path.isfile(self.q_table_file):
            try:
                Q = np.load(self.q_table_file)
                self.get_logger().info(f"Q-table loaded from {self.q_table_file}")
                # Check if Q shape matches expected shape
                if Q.shape == (self.num_states, self.num_actions):
                    return Q
                else:
                    self.get_logger().warn("Loaded Q-table shape doesn't match current state/action dimensions. Initializing a new one.")
                    return np.zeros((self.num_states, self.num_actions))
            except Exception as e:
                self.get_logger().error(f"Error loading Q-table: {e}, initializing a new one.")
                return np.zeros((self.num_states, self.num_actions))
        else:
            self.get_logger().info("No Q-table found, initializing a new one.")
            return np.zeros((self.num_states, self.num_actions))

    def save_q_table(self):
        """Save the current Q-table to file."""
        try:
            np.save(self.q_table_file, self.Q)
            self.get_logger().info(f"Q-table saved to {self.q_table_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save Q-table: {e}")

    def destroy_node(self):
        """Override to save Q-table on shutdown."""
        self.save_q_table()
        super().destroy_node()

    def marker_pose_callback(self, msg):
        """Callback for marker pose updates."""
        self.marker_pose = msg

    def robot_pose_callback(self, msg):
        """Callback for robot pose updates."""
        self.robot_pose = msg

    def control_loop(self):
        """Main control loop: runs every 0.1s, updates Q-table and selects actions."""
        # Ensure we have both robot and marker poses
        if self.marker_pose is None or self.robot_pose is None:
            return

        s = self.get_current_state()
        if s is None:
            return

        # Update Q-table if we have a previous state-action pair
        if self.current_state is not None and self.current_action is not None:
            r = self.get_reward(s)
            s_next = s
            a_next = np.argmax(self.Q[s_next,:])
            td_target = r + self.gamma * self.Q[s_next, a_next]
            td_error = td_target - self.Q[self.current_state, self.current_action]
            self.Q[self.current_state, self.current_action] += self.alpha * td_error

        # Epsilon-greedy action selection
        if random.random() < self.epsilon:
            a = random.randint(0, self.num_actions-1)
        else:
            a = np.argmax(self.Q[s,:])

        self.execute_action(a)

        self.current_state = s
        self.current_action = a

    def get_current_state(self):
        """Compute the current discretized state from robot and marker poses."""
        # Relative position: marker - robot
        x_rel = self.marker_pose.position.x - self.robot_pose.position.x
        y_rel = self.marker_pose.position.y - self.robot_pose.position.y

        robot_yaw = self.pose_to_yaw(self.robot_pose)
        distance = math.sqrt(x_rel**2 + y_rel**2)
        bearing = math.atan2(y_rel, x_rel) - robot_yaw

        dist_idx = self.discretize_distance(distance)
        bearing_idx = self.discretize_bearing(bearing)

        s = self.state_to_index(dist_idx, bearing_idx)
        return s

    def pose_to_yaw(self, pose):
        """Convert a Pose's quaternion orientation to a yaw angle."""
        q = pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def discretize_distance(self, distance):
        """Map continuous distance to a discrete bin (0 to 10)."""
        if distance < self.distance_thresholds[0]:
            return 0
        for i, thr in enumerate(self.distance_thresholds):
            if distance < thr:
                return i
        # If distance not less than any threshold and <3.0m, it's bin 9
        if distance < 3.0:
            return 9
        # Else bin 10 means ≥3.0m
        return 10

    def discretize_bearing(self, bearing):
        """Map continuous bearing to a discrete bin (0 to 10)."""
        abs_b = abs(bearing)
        if abs_b < self.bearing_thresholds[0]:
            return 0
        for i, thr in enumerate(self.bearing_thresholds):
            if abs_b < thr:
                return i
        return 10

    def state_to_index(self, dist_idx, bearing_idx):
        """Flatten (dist_idx, bearing_idx) into a single integer index."""
        return dist_idx * self.num_bearing_bins + bearing_idx

    def index_to_state(self, s):
        """Inverse of state_to_index: given s, recover (dist_idx, bearing_idx)."""
        bearing_idx = s % self.num_bearing_bins
        dist_idx = s // self.num_bearing_bins
        return dist_idx, bearing_idx

    def get_reward(self, s):
        """Compute reward based on the discretized state."""
        dist_idx, bearing_idx = self.index_to_state(s)
        # If dist_idx=0 means very close to the marker, high positive reward
        if dist_idx == 0:
            reward = 100.0
        elif dist_idx == 1:
            reward = 50.0
        elif dist_idx == 2:
            reward = 25.0
        elif dist_idx == 3:
            reward = 15.0
        elif dist_idx == 4:
            reward = 10.0
        elif dist_idx == 5:
            reward = 7.5
        elif dist_idx == 6:
            reward = 5.0
        elif dist_idx == 7:
            reward = 2.5
        elif dist_idx == 8:
            reward = 1.0
        elif dist_idx == 9:
            reward = 0.5
        elif dist_idx == 10:
            reward = -100.0
        else:
            # Negative reward proportional to distance
            reward = -dist_idx
            
        return reward

    def execute_action(self, a):
        """Publish a Twist message corresponding to the chosen action."""
        cmd = Twist()
        lin_vel, ang_vel = self.actions[a]
        cmd.linear.x = lin_vel
        cmd.angular.z = ang_vel
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = QLearningNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        # Ensures Q-table is saved when shutting down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
