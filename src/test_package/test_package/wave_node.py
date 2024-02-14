import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from time import sleep

def main(args=None):
    rclpy.init()

    node = rclpy.create_node('minimal_publisher')
    publisher = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    # Create JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.header = Header()

    # Set joint name
    trajectory_msg.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    
    while rclpy.ok():
        initial_point = JointTrajectoryPoint()
        initial_point.positions = [0, 0.6, 0, 0, 0, 0]
        initial_point.time_from_start.sec = 0
        trajectory_msg.points = [initial_point]

        target_point = JointTrajectoryPoint()
        target_point.positions = [0, -0.6, 0, 0, 0, 0]
        target_point.time_from_start.sec = 4  # Time to reach the target position
        trajectory_msg.points.append(target_point)

        target_point = JointTrajectoryPoint()
        target_point.positions = [0, 0.6, 0, 0, 0, 0]
        target_point.time_from_start.sec = 8  # Time to reach the target position
        trajectory_msg.points.append(target_point)

        # Publish the trajectory message
        publisher.publish(trajectory_msg)

        sleep(10)  # Adjust the sleep duration based on the desired wave frequency
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()