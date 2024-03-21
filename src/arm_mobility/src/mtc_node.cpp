// ROS
#include <rclcpp/rclcpp.hpp>
// MTC pick/place demo implementation
#include <arm_mobility/pick_place_task.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_mobility");
geometry_msgs::msg::Pose received_object_pose;
bool is_pose_received = false;

void objectPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) { 
  if (!is_pose_received) { 
    RCLCPP_INFO(LOGGER, "Received object pose");
    received_object_pose = *msg; 
    is_pose_received = true; 
  } else { 
    RCLCPP_INFO(LOGGER, "Pose received. Ignoring objectPositionCallback.");
  }
}

int main(int argc, char** argv) { 
  rclcpp::init(argc, argv); 
  rclcpp::NodeOptions node_options; 
  node_options.automatically_declare_parameters_from_overrides(true); 
  auto node = rclcpp::Node::make_shared("mtc_node", node_options); 
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  auto object_pose_subscription = node->create_subscription<geometry_msgs::msg::Pose>("object_3d_pose", 10, objectPositionCallback);

  while (rclcpp::ok() && !is_pose_received) { 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  };

  moveit_task_constructor_demo::setupPlanningScene(received_object_pose);
  // Construct and run pick/place task 
  moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task");
  if (!pick_place_task.init(node)) {
    RCLCPP_INFO(LOGGER, "Initialization failed");
    return 1; 
  }

  if (pick_place_task.plan(10)) { 
    RCLCPP_INFO(LOGGER, "Planning succeded"); 
    // commented out so the plan is not executed by default. user should execute using RViz.
    // try { 
    //   pick_place_task.execute();
    //   RCLCPP_INFO(LOGGER, "Execution complete"); 
    // } catch (const std::exception& e) { 
    //   RCLCPP_INFO(LOGGER, "Execution disabled"); 
    // } 
  } else { 
    RCLCPP_INFO(LOGGER, "Planning failed");
  }
  // Keep introspection alive
  spinning_thread.join();
  return 0;
}