import rclpy

from rclpy.node import Node 
from geometry_msgs.msg import Twist
from clearpath_platform_msgs.msg import Drive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TwistMux2CmdDrive(Node):
    def __init__(self):
        super().__init__('twist_mux_2_cmd_drive')

        # Jackal Parameters
        self.declare_parameter('wheels_length', 0.36)
        self.declare_parameter('wheels_radius', 0.098)
        self.wheels_length = self.get_parameter('wheels_length').get_parameter_value().double_value
        self.wheels_radius = self.get_parameter('wheels_radius').get_parameter_value().double_value

        # CMD Drive messages
        self.cmd_drive_msg = Drive()
        self.cmd_drive_msg.mode = Drive.MODE_VELOCITY

        # Timer
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

        # Twist Mux Subscription
        self.create_subscription(Twist,
                                    '/jackal_velocity_controller/cmd_vel_unstamped',
                                    self.twist_mux_callback,
                                    QoSProfile(
                                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                        history=QoSHistoryPolicy.KEEP_LAST,
                                        depth=5
                                    )
                                )

        # Jackal Publisher
        self.publisher = self.create_publisher(Drive,
                                                '/platform/motors/cmd_drive',
                                                QoSProfile(
                                                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                    history=QoSHistoryPolicy.KEEP_LAST,
                                                    depth=5
                                                )
                                            )

    def twist_mux_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Inverse Kinematics
        left_wheel_speed = (2*linear_speed-self.wheels_length*angular_speed)/(2*self.wheels_radius)
        right_wheel_speed = (self.wheels_length/self.wheels_radius)*angular_speed+left_wheel_speed

        # Drive Message for Jackal
        self.cmd_drive_msg.drivers = [left_wheel_speed, right_wheel_speed]

    def timer_callback(self):
        self.publisher.publish(self.cmd_drive_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_mux_2_cmd_drive = TwistMux2CmdDrive()
    rclpy.spin(twist_mux_2_cmd_drive)
    twist_mux_2_cmd_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
