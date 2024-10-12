import rclpy
import numpy as np
import time

from rclpy.node import Node 
from clearpath_platform_msgs.msg import Feedback
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        # Jackal Parameters
        self.declare_parameter('wheels_length', 0.36)
        self.declare_parameter('wheels_radius', 0.098)
        self.wheels_length = self.get_parameter('wheels_length').get_parameter_value().double_value
        self.wheels_radius = self.get_parameter('wheels_radius').get_parameter_value().double_value
        
        # Internal Variables
        self.differential_drive_jacobian_matrix = lambda theta : np.array([[np.cos(theta)/2, np.cos(theta)/2],[np.sin(theta)/2, np.sin(theta)/2],[-1/self.wheels_radius, 1/self.wheels_radius]])

        self.x, self.y, self.theta = 0, 0, 0

        self.last_time = self.get_clock().now()

        # Encoders Subscription
        self.create_subscription(Feedback,
                                    '/platform/motors/feedback',
                                    self.encoders_callback,
                                    QoSProfile(
                                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                        history=QoSHistoryPolicy.KEEP_LAST,
                                        depth=5
                                    )
                                )

        # Odometry Publisher
        self.publisher = self.create_publisher(Odometry, 
                                                '/platform/odom', 
                                                QoSProfile(
                                                    reliability=QoSReliabilityPolicy.RELIABLE,
                                                    history=QoSHistoryPolicy.KEEP_LAST,
                                                    depth=10
                                                )
                                            )

    def encoders_callback(self, msg):
        # Read encoder velocities (rad/s) and saves them in (m/s)
        left_wheel_speed = msg.drivers[0].measured_velocity * self.wheels_radius
        right_wheel_speed = msg.drivers[1].measured_velocity * self.wheels_radius

        # Puts speeds in numpy array
        wheels_speeds = np.array([[left_wheel_speed],
                                 [right_wheel_speed]])
        
        # Performs Forward Kinematics as a Jacobian Matrix Multiplication
        speeds = (self.differential_drive_jacobian_matrix(self.theta) @ wheels_speeds).flatten()

        # Delta Time
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9 

        # Predicts Position
        self.x += speeds[0] * delta_time
        self.y += speeds[1] * delta_time
        self.theta += speeds[2] * delta_time

        # Create and publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*quat)

        # Set velocity
        odom_msg.twist.twist.linear.x = speeds[0]
        odom_msg.twist.twist.linear.y = speeds[1]
        odom_msg.twist.twist.angular.z = speeds[2]

        # Publish odometry message
        self.publisher.publish(odom_msg)

        # Update last time
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    odometry_node = Odometry()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
