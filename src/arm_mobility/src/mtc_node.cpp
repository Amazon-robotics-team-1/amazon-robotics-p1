#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_mobility");
namespace mtc = moveit::task_constructor;

class MTCTaskNode {
 public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

  bool isObjectPositionReceived();

 private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr object_position_subscriber_;
  bool object_position_received_;
  geometry_msgs::msg::Pose received_object_pose_;
  void objectPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }, object_position_received_{ false } {
    object_position_subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose>("object_3d_pose", 10, std::bind(&MTCTaskNode::objectPositionCallback, this, std::placeholders::_1));
}

void MTCTaskNode::objectPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  RCLCPP_INFO(LOGGER, "Received object pose");
  object_position_received_ = true;
  received_object_pose_ = *msg;
}

bool MTCTaskNode::isObjectPositionReceived() {
  return object_position_received_;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene() {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  // geometry_msgs::msg::Pose pose;
  // pose.position.x = 0.5;
  // pose.position.y = -0.25;
  // object.pose = pose;
  object.pose = received_object_pose_;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask() {
  task_ = createTask();

  try {
    task_.init();
  }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask() {
  mtc::Task task;
  task.stages()->setName("demo task"); // TODO: rename task.
  task.loadRobotModel(node_);

  const auto& gripper_name = "gripper";
  const auto& arm_name = "manipulator";

//   // Set task properties
  task.setProperty("r_arm", arm_name);
  task.setProperty("r_grip", gripper_name);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_go_home = std::make_unique<mtc::stages::MoveTo>("go home", sampling_planner);
  // clang-format on

  stage_go_home->setGroup(arm_name);
  stage_go_home->setGoal("home");
  task.add(std::move(stage_go_home));

  return task;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Wait for the object position to be received
  while (rclcpp::ok() && !mtc_task_node->isObjectPositionReceived()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add a small delay to avoid high CPU usage
  }
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}