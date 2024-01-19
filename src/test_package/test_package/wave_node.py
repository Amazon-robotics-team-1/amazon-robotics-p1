import rclpy
import sensor_msgs.msg
from rclpy.node import Node


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(sensor_msgs.msg.JointState, 'joint_states', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        msg = sensor_msgs.msg.JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", \
                    "robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint", "robotiq_85_left_inner_knuckle_joint",\
                    "robotiq_85_right_inner_knuckle_joint", "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"]
        
        msg.position = [0, self.i*0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg.effort =  []
        msg.velocity = []

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i *= -1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()