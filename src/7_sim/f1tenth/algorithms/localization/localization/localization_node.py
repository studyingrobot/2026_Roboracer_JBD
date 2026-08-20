import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('input_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('output_odom_topic', '/localization/odom')
        self.declare_parameter('output_pose_topic', '/localization/pose')

        input_odom_topic = self.get_parameter('input_odom_topic').value
        output_odom_topic = self.get_parameter('output_odom_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value

        self.odom_pub = self.create_publisher(
            Odometry,
            output_odom_topic,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            output_pose_topic,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            input_odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('localization_node started')
        self.get_logger().info(f'input_odom_topic  : {input_odom_topic}')
        self.get_logger().info(f'output_odom_topic : {output_odom_topic}')
        self.get_logger().info(f'output_pose_topic : {output_pose_topic}')

    def odom_callback(self, msg: Odometry):
        self.odom_pub.publish(msg)

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    node = LocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
