#include <Eigen/Geometry>
#include <arm_mobility/pick_place_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_mobility");

namespace moveit_task_constructor_demo {

void setupPlanningScene(const geometry_msgs::msg::Pose& received_pose) {
    rclcpp::sleep_for(std::chrono::microseconds(100));  // wait for apply planning service?
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1, 0.02 };
    object.pose = received_pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}


bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node) {
    RCLCPP_INFO(LOGGER, "Initializing task pipeline");

    // Reset ROS introspection before constructing the new object
    task_.reset();
    task_.reset(new moveit::task_constructor::Task());

    Task& t = *task_;
    t.stages()->setName(task_name_);
    t.loadRobotModel(node);

    const auto& arm_group_name = "manipulator"; // manipulator
    const auto& hand_group_name = "gripper"; // gripper
    const auto& hand_frame = "robotiq_85_base_link"; // ik_frame: robotiq_85_base_link, bracelet_link (X), end_effector_link

    // Sampling planner
    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

    // Cartesian planner
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    // Set task properties
    t.setProperty("group", arm_group_name);
    t.setProperty("eef", hand_group_name);
    t.setProperty("hand", hand_group_name);
    t.setProperty("hand_grasping_frame", hand_frame);
    t.setProperty("ik_frame", hand_frame);

    /****************************************************
     *                                                  *
     *               Current State                      *
     *                                                  *
     ***************************************************/
    {
        auto current_state = std::make_unique<stages::CurrentState>("current state");

        // Verify that object is not attached
        auto applicability_filter =
            std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
        applicability_filter->setPredicate([object = "object"](const SolutionBase& s, std::string& comment) {
            if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                comment = "object with id 'object' is already attached and cannot be picked";
                return false;
            }
            return true;
        });
        t.add(std::move(applicability_filter));
    }

    /****************************************************
     *                                                  *
     *               Open Hand                          *
     *                                                  *
     ***************************************************/
    Stage* initial_state_ptr = nullptr;
    {  // Open Hand
        auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("Open");
        initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
        t.add(std::move(stage));
    }

    /****************************************************
     *                                                  *
     *               Move to Pick                       *
     *                                                  *
     ***************************************************/
    {  // Move-to pre-grasp
        auto stage = std::make_unique<stages::Connect>(
            "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        t.add(std::move(stage));
    }

    /****************************************************
     *                                                  *
     *               Pick Object                        *
     *                                                  *
     ***************************************************/
    Stage* pick_stage_ptr = nullptr;
    {
        auto grasp = std::make_unique<SerialContainer>("pick object");
        t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

        /****************************************************
  ---- *               Approach Object                    *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.0, 0.15);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  ---- *               Generate Grasp Pose                *
         ***************************************************/
        {
            // Sample grasp pose
            auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("Open");
            stage->setObject("object");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

            // Compute IK
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.15;  // changed 0.1 to 0.15 here.

            auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));
        }

        /****************************************************
  ---- *               Allow Collision (hand object)   *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(
                "object",
                t.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  ---- *               Close Hand                      *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("Close");
            grasp->insert(std::move(stage));
        }

        /****************************************************
  .... *               Attach Object                      *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  .... *               Lift object                        *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.0, 0.3);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

        // Add grasp container to task
        t.add(std::move(grasp));
    }

    /******************************************************
    *                                                    *
    *          Move to Place                             *
    *                                                    *
    *****************************************************/
    {
        auto stage = std::make_unique<stages::Connect>(
            "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        t.add(std::move(stage));
    }

    /******************************************************
    *                                                    *
    *          Place Object                              *
    *                                                    *
    *****************************************************/
    {
        auto place = std::make_unique<SerialContainer>("place object");
        t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
        place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

        /******************************************************
    ---- *          Lower Object                              *
        *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.0, 0.15);

            // Set downward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        /******************************************************
    ---- *          Generate Place Pose                       *
        *****************************************************/
        {
            // Generate Place Pose
            auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject("object");

            // Set target pose
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = "base_link";
            p.pose.position.x = -0.4;
            p.pose.orientation.w = 1.0;
            //  p.pose = vectorToPose(params.place_pose);
            //  p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
            stage->setPose(p);
            stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

            // Compute IK
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.15;  // changed 0.1 to 0.15 here.
            auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        /******************************************************
    ---- *          Open Hand                              *
        *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("Open");
            place->insert(std::move(stage));
        }

        /******************************************************
    ---- *          Forbid collision (hand, object)        *
        *****************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions("object", *t.getRobotModel()->getJointModelGroup(hand_group_name),
                                    false);
            place->insert(std::move(stage));
        }

        /******************************************************
    ---- *          Detach Object                             *
        *****************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
            stage->detachObject("object", hand_frame);
            place->insert(std::move(stage));
        }

        /******************************************************
    ---- *          Retreat Motion                            *
        *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.12, .25);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "retreat");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        // Add place container to task
        t.add(std::move(place));
    }

    /******************************************************
     *                                                    *
     *          Move to Home                              *
     *                                                    *
     *****************************************************/
    {
        auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
        const std::map<std::string, double> final_pose = {
            {"joint_1", -0.0180993754003973},
            {"joint_2", -0.07014020626163475},
            {"joint_3", -3.132451619464205},
            {"joint_4", -2.2288977869315385},
            {"joint_5", 0.00573852490129445},
            {"joint_6", -0.9132400157333942},
            {"joint_7", 1.5578977055292613}
        };
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        // stage->setGoal("home");
        stage->setGoal(final_pose);
        stage->restrictDirection(stages::MoveTo::FORWARD);
        t.add(std::move(stage));
    }

    // prepare Task structure for planning
    try {
        t.init();
    } catch (InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
        return false;
    }

    return true;
}

bool PickPlaceTask::plan(const std::size_t max_solutions) {
    RCLCPP_INFO(LOGGER, "Start searching for task solutions");

    return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTask::execute() {
    RCLCPP_INFO(LOGGER, "Executing solution trajectory");
    moveit_msgs::msg::MoveItErrorCodes execute_result;

    execute_result = task_->execute(*task_->solutions().front());

    if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
        return false;
    }

    return true;
}


}  // namespace moveit_task_constructor_demo
