import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class StanceController(Node):
    def __init__(self):
        super().__init__('stance_controller')
        
        # Publisher for idle stance joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribe to joint states to monitor gait engine activity
        # We use a subscription group to not interfere with the converter
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Track last time we received joint states from gait engine
        self.last_gait_time = None
        self.gait_active = False
        self.ever_received_gait = False
        
        # Timeout before engaging idle stance (seconds)
        self.declare_parameter('idle_timeout', 0.5)
        self.idle_timeout = self.get_parameter('idle_timeout').value
        
        # Define idle stance - all joints at 0.0 (default SDF position)
        # This matches the position when robot spawns in Gazebo
        self.idle_joint_names = [
            'leg1_joint1', 'leg1_joint2', 'leg1_joint3', 'leg1_joint4',  # Rear Left
            'leg2_joint1', 'leg2_joint2', 'leg2_joint3', 'leg2_joint4',  # Rear Right
            'leg3_joint1', 'leg3_joint2', 'leg3_joint3', 'leg3_joint4',  # Front Left
            'leg4_joint1', 'leg4_joint2', 'leg4_joint3', 'leg4_joint4',  # Front Right
        ]
        self.idle_positions = [0.0] * len(self.idle_joint_names)
        
        # Timer to check if we should publish idle stance
        self.stance_timer = self.create_timer(0.01, self.check_and_publish_idle)
        
        self.get_logger().info('Stance Controller started')
        self.get_logger().info(f'Idle timeout: {self.idle_timeout}s')
        self.get_logger().info(f'Idle stance: all joints at 0.0 (default SDF position)')
        self.get_logger().info('Waiting for gait engine to start...')
        
    def joint_state_callback(self, msg):
        """Monitor incoming joint states from gait engine"""
        # Check if this message is from the gait engine (has leg joint names)
        # vs from this node (idle stance)
        is_from_gait = any(joint in msg.name for joint in ['leg1_joint1', 'leg2_joint1', 'leg3_joint1', 'leg4_joint1'])
        
        if is_from_gait:
            self.last_gait_time = time.time()
            self.ever_received_gait = True
            
            if not self.gait_active:
                self.gait_active = True
                self.get_logger().info('Gait engine active - idle stance disabled')
    
    def check_and_publish_idle(self):
        """Check if gait has stopped and publish idle stance if needed"""
        # Don't do anything until we've seen gait engine at least once
        if not self.ever_received_gait:
            return
            
        time_since_gait = time.time() - self.last_gait_time
        
        # If timeout exceeded and gait was previously active
        if time_since_gait > self.idle_timeout and self.gait_active:
            self.gait_active = False
            self.get_logger().info('Gait engine stopped - engaging idle stance')
        
        # Publish idle stance when gait is not active
        if not self.gait_active:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.idle_joint_names
            msg.position = self.idle_positions
            self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()