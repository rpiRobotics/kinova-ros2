#include <kinova_driver/joint_trajectory_action_server.h>
#include <rclcpp/utilities.hpp>

using namespace kinova;

JointTrajectoryActionController::JointTrajectoryActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name, std::string &joint_namespace):
    nh_(n),
    has_active_goal_(false),
    joint_namespace_(joint_namespace)
{
    // Normalize so joint_namespace_ never has a leading/trailing '/'.
    // This lets us always compare against "<namespace>/<joint_name>" consistently.
    while (!joint_namespace_.empty() && joint_namespace_.front() == '/')
        joint_namespace_.erase(joint_namespace_.begin());
    while (!joint_namespace_.empty() && joint_namespace_.back() == '/')
        joint_namespace_.pop_back();

    std::string robot_type = robot_name;
    std::string address;

    // address = "/" + robot_name + "_driver/robot_type";
    // nh_.getParam(address,robot_type);
    // if (robot_type == "")
    // {
    //     ROS_ERROR_STREAM("Parameter "<<address<<" not found, make sure robot driver node is running");
    // }

    address = robot_name + "/follow_joint_trajectory";

    action_server_follow_ = rclcpp_action::create_server<FJTAS>(
        nh_,
        address,
        std::bind(&JointTrajectoryActionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointTrajectoryActionController::handle_cancel, this, std::placeholders::_1),
        std::bind(&JointTrajectoryActionController::handle_accepted, this, std::placeholders::_1));

    int arm_joint_num = robot_type[3]-'0';
    joint_names_.resize(arm_joint_num);

    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_name + "_joint_" + std::to_string(i+1);
    }

    goal_time_constraint_ = 0;
    if (!nh_->has_parameter("constraints/goal_time"))
        nh_->declare_parameter("constraints/goal_time", goal_time_constraint_);
    nh_->get_parameter("constraints/goal_time", goal_time_constraint_);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        std::string ns = std::string("constraints/") + joint_names_[i];
        double g = -1.0;
        double t = -1.0;

        if (!nh_->has_parameter(ns + "/goal"))
            nh_->declare_parameter(ns + "/goal", g);
        nh_->get_parameter(ns + "/goal", g);
        if (!nh_->has_parameter(ns + "/trajectory"))
            nh_->declare_parameter(ns + "/trajectory", t);
        nh_->get_parameter(ns + "/trajectory", t);

        goal_constraints_[joint_names_[i]] = g;
        trajectory_constraints_[joint_names_[i]] = t;
    }

    stopped_velocity_tolerance_ = 0.05;
    if (!nh_->has_parameter("constraints/stopped_velocity_tolerance"))
        nh_->declare_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);
    nh_->get_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);

    pub_controller_command_ = nh_->create_publisher<trajectory_msgs::msg::JointTrajectory>
            (robot_name + "_driver/trajectory_controller/command", 1);
    sub_controller_state_ = nh_->create_subscription<control_msgs::action::FollowJointTrajectory_Feedback>(robot_name + "_driver/trajectory_controller/state",
            1, std::bind(&JointTrajectoryActionController::controllerStateCB, this, std::placeholders::_1));
    watchdog_timer_ = nh_->create_wall_timer(std::chrono::seconds(1), std::bind(&JointTrajectoryActionController::watchdog, this));

    long unsigned int started_waiting_for_controller = nh_->get_clock()->now().seconds();
    while (rclcpp::ok() && last_controller_state_ == empty_controller_state_)
    {
        if (started_waiting_for_controller != nh_->get_clock()->now().seconds() &&
                nh_->get_clock()->now().seconds() > started_waiting_for_controller + 30)
        {
            RCLCPP_WARN(nh_->get_logger(), "Waited for the controller for 30 seconds, but it never showed up. Continue waiting the feedback of trajectory state on topic /trajectory_controller/state ...");
            started_waiting_for_controller = nh_->get_clock()->now().seconds();
        }
        rclcpp::spin_some(nh_);
        rclcpp::Rate(1).sleep();
    }

    RCLCPP_INFO(nh_->get_logger(), "Start Follow_Joint_Trajectory_Action server!");
    RCLCPP_INFO(nh_->get_logger(), "Waiting for an plan execution (goal) from Moveit");
}


JointTrajectoryActionController::~JointTrajectoryActionController()
{
    pub_controller_command_.reset();
    sub_controller_state_.reset();
    watchdog_timer_.reset();
}


static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
        if (count(b.begin(), b.end(), a[i]) != 1)
            return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
        if (count(a.begin(), a.end(), b[i]) != 1)
            return false;
    }

    return true;
}


// Strips a leading "<joint_namespace_>/" from a single joint name, if present.
// If joint_namespace_ is empty, or the name isn't namespaced, the name is
// returned unchanged.
std::string JointTrajectoryActionController::stripNamespace(const std::string &name) const
{
    if (joint_namespace_.empty())
        return name;

    const std::string prefix = joint_namespace_ + "/";
    if (name.compare(0, prefix.size(), prefix) == 0)
        return name.substr(prefix.size());

    return name;
}

// Vector overload: strips the namespace from every joint name in the list.
std::vector<std::string> JointTrajectoryActionController::stripNamespace(const std::vector<std::string> &names) const
{
    std::vector<std::string> stripped;
    stripped.reserve(names.size());
    for (const auto &name : names)
        stripped.push_back(stripNamespace(name));

    return stripped;
}


void JointTrajectoryActionController::watchdog()
{
    rclcpp::Time now = nh_->get_clock()->now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
        bool should_abort = false;
        if (!last_controller_state_.header.stamp.sec)
        {
            should_abort = true;
            RCLCPP_WARN(nh_->get_logger(), "Aborting goal because we have never heard a controller state message.");
        }
        else if ((now - last_controller_state_.header.stamp) > rclcpp::Duration(5,0))
        {
            should_abort = true;
            RCLCPP_WARN(nh_->get_logger(), "Aborting goal because we haven't heard from the controller in %.3lf seconds",
                     (now.seconds() - last_controller_state_.header.stamp.sec));
        }

        if (should_abort)
        {
            // Stops the controller.
            trajectory_msgs::msg::JointTrajectory empty;
            empty.joint_names = joint_names_;
            //pub_controller_command_->publish(empty);

            // Marks the current goal as aborted.
            active_goal_->abort(active_result_);
            has_active_goal_ = false;
        }
    }
}


rclcpp_action::GoalResponse JointTrajectoryActionController::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal>goal)
{
    RCLCPP_INFO(nh_->get_logger(), "Received goal request");
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryActionController::handle_cancel(const std::shared_ptr<GoalHandleFJTAS> gh)
{
    if (active_goal_ == gh)
    {
        // Marks the current goal as canceled.
        active_goal_->canceled(active_result_);
        has_active_goal_ = false;
    }
    RCLCPP_INFO(nh_->get_logger(), "Received request to cancel goal");
    (void) gh;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryActionController::handle_accepted(const std::shared_ptr<GoalHandleFJTAS> goal_handle)
{
    // Cancel any previous goal BEFORE active_goal_ is overwritten below —
    // otherwise this check would end up canceling the goal we're about to
    // accept instead of a genuinely stale one.
    if (has_active_goal_ && active_goal_ && active_goal_ != goal_handle)
    {
        RCLCPP_INFO(nh_->get_logger(), "Canceling previous goal in favor of the new one");
        active_goal_->canceled(std::make_shared<FJTAS::Result>());
    }

    // Must be set here (not in goalCBFollow) so it's ready before any
    // controller feedback can arrive; goalCBFollow runs on a detached thread
    // and might not have run yet by the time feedback starts coming in.
    active_goal_ = goal_handle;
    has_active_goal_ = true;
    first_fb_ = true;
    RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server accepted goal!");
 	// this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread
    {
        std::bind(&JointTrajectoryActionController::goalCBFollow, this, std::placeholders::_1), goal_handle
    }.detach();
}

void JointTrajectoryActionController::goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh)
{
    const auto goal = gh->get_goal();
    RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server received goal!");

    active_result_ = std::make_shared<FJTAS::Result>();

    // The incoming goal's joint names may be namespaced (e.g. "oarbot_silver/j2n6s300_joint_1")
    // while joint_names_ is stored without the namespace prefix, so strip it before comparing.
    std::vector<std::string> incoming_joint_names = stripNamespace(goal->trajectory.joint_names);

    // Ensures that the joints in the goal match the joints we are commanding.
    if (!setsEqual(joint_names_, incoming_joint_names))
    {
        RCLCPP_ERROR(nh_->get_logger(), "Joints on incoming goal don't match our joints");
        gh->abort(active_result_);
        has_active_goal_ = false;
        return;
    }


    // Sends the trajectory along to the controller, using the de-namespaced
    // joint names so downstream consumers (and our own bookkeeping below)
    // don't have to know about the namespace at all.
    current_traj_ = goal->trajectory;
    current_traj_.joint_names = incoming_joint_names;
    pub_controller_command_->publish(current_traj_);
    RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server published goal via command publisher!");
}

void JointTrajectoryActionController::controllerStateCB(const control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(nh_->get_logger(), "Joint_trajectory_action_server received feedback of trajectory state from topic: /trajectory_controller/state");

    last_controller_state_ = *msg;

    rclcpp::Time now = nh_->get_clock()->now();

    if (!has_active_goal_)
        return;
    if (current_traj_.points.empty())
        return;

    //joint trajectory message seems to have header.stamp = 0
    // using first feedback msg as guide for starting timestamp
    if (first_fb_)
    {
        start_time_ = msg->header.stamp;
        first_fb_ = false;
    }

    if (now - start_time_ < current_traj_.points[0].time_from_start)
    {
        return;
    }

    // The controller may echo back namespaced joint names too, so strip
    // before comparing against our (un-namespaced) joint_names_ and before
    // using them as keys into goal_constraints_.
    std::vector<std::string> feedback_joint_names = stripNamespace(msg->joint_names);

    if (!setsEqual(joint_names_, feedback_joint_names))
    {
        RCLCPP_ERROR_ONCE(nh_->get_logger(), "Joint names from the controller don't match our joint names.");
        return;
    }

    int last = current_traj_.points.size() - 1;
    rclcpp::Time end_time = start_time_ + current_traj_.points[last].time_from_start;

    if (end_time - now < rclcpp::Duration(goal_time_constraint_,0))
    {
        // Checks that we have ended inside the goal constraints
        bool inside_goal_constraints = true;
        for (size_t i = 0; i < feedback_joint_names.size() && inside_goal_constraints; ++i)
        {
            // computing error from goal pose
            double abs_error = fabs(msg->actual.positions[i] - current_traj_.points[last].positions[i]);
            double goal_constraint = goal_constraints_[feedback_joint_names[i]];
            if (goal_constraint >= 0 && abs_error > goal_constraint)
                inside_goal_constraints = false;
            // It's important to be stopped if that's desired.
            if ( !(msg->desired.velocities.empty()) && (fabs(msg->desired.velocities[i]) < 1e-6) )
            {
                if (fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
                    inside_goal_constraints = false;
            }
        }
        if (inside_goal_constraints)
        {
            active_goal_->succeed(active_result_);
            has_active_goal_ = false;
            first_fb_ = true;
        }
        else if (now - end_time < rclcpp::Duration(goal_time_constraint_,0))
        {
            // Still have some time left to make it.
        }
        else
        {
            RCLCPP_WARN(nh_->get_logger(), "Aborting because we wound up outside the goal constraints");
            active_goal_->abort(active_result_);
            has_active_goal_ = false;
        }
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto args = rclcpp::remove_ros_arguments(argc, argv);

    if (args.size() < 2)
    {
        std::cerr << "No kinova_robot_name provided!" << std::endl;
        return -1;
    }

    std::string robot_name = args[1];

    // Optional: the namespace prefix that will show up on incoming joint
    // names, e.g. "oarbot_silver" for joints named "oarbot_silver/j2n6s300_joint_1".
    // Defaults to empty (no namespace) if not provided.
    std::string joint_namespace = (args.size() >= 3) ? args[2] : "";

    auto node = std::make_shared<rclcpp::Node>(
        "follow_joint_trajectory_action_server");

    kinova::JointTrajectoryActionController jtac(node, robot_name, joint_namespace);

    rclcpp::spin(node);
    rclcpp::shutdown();
}