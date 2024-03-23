#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("initial_position_node");

class JointStateToTrajectoryPublisher : public rclcpp::Node {
  public:
    JointStateToTrajectoryPublisher(): Node("initial_position_node") {
        // Subscribe to the joint_states topic
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&JointStateToTrajectoryPublisher::jointStateCallback, this, std::placeholders::_1));

        // Create a publisher for the joint_trajectory topic
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_trajectory_controller/joint_trajectory", 10);

        // Define the fixed trajectory message
        fixed_trajectory_msg_.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
        fixed_trajectory_msg_.points.resize(1);
        fixed_trajectory_msg_.points[0].positions = {-0.0180993754003973, -0.07014020626163475, -3.132451619464205, -2.2288977869315385, 0.00573852490129445, -0.9132400157333942, 1.5578977055292613};
        fixed_trajectory_msg_.points[0].time_from_start = rclcpp::Duration::from_seconds(4);

        fixed_joint_map_ = {
            {"joint_1", -0.0180993754003973},
            {"joint_2", -0.07014020626163475},
            {"joint_3", -3.132451619464205},
            {"joint_4", -2.2288977869315385},
            {"joint_5", 0.00573852490129445},
            {"joint_6", -0.9132400157333942},
            {"joint_7", 1.5578977055292613},
        };

        // Create a timer for publishing the fixed trajectory message
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), // Publish every 2 seconds
            std::bind(&JointStateToTrajectoryPublisher::publishFixedTrajectory, this));
    }

  private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Check if the received joint positions match the fixed trajectory
        if (isMatchingFixedTrajectory(msg)) {
            RCLCPP_INFO(LOGGER, "Robot has successfully reached the initial pose.");
            rclcpp::shutdown();
            return;
        }
    }

    void publishFixedTrajectory() {
        // Publish the fixed trajectory message
        joint_trajectory_publisher_->publish(fixed_trajectory_msg_);
    }

    bool isMatchingFixedTrajectory(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Define tolerance for deviation
        const double tolerance = 0.01; // Adjust as needed

        // Compare each joint position with the corresponding position in the fixed trajectory
        for (size_t i = 0; i < msg->position.size(); ++i) {
            // Check if the absolute difference between joint positions exceeds tolerance
            std::string current_joint = msg->name[i];
            if (fixed_joint_map_.find(current_joint) != fixed_joint_map_.end()) {
                if (std::abs(msg->position[i] - fixed_joint_map_[current_joint] > tolerance)) {
                    return false;
                }
            }
        }

        return true; // All joint positions are within tolerance
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    trajectory_msgs::msg::JointTrajectory fixed_trajectory_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, double> fixed_joint_map_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateToTrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
