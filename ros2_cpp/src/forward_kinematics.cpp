#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
// Moveit Core
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>

class ForwardKinematics
{
public:
    void generate_inverse(const std::shared_ptr<rclcpp::Node>& node)
    {
        // 1. Setup:
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
        // 1.1 Set Planner and a planning time limit
        arm_move_group.setPlannerId("RRTConnect");
        arm_move_group.setPlanningTime(5.0);
        // 2. For now define a vector of some Forward Kinemtics based random pose on each joints:
        std::vector<double> arm_joint_goal {2.61799388, 0.41887902, -4.69493569, -1.72787596, 3.15904595, 5.74213324};
        // 3. Set the joint positions
        bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
        // 4. Check if joint values are set correctly or if joint values are out of bounds
        if(!arm_within_bounds)
        {
            RCLCPP_WARN(node->get_logger(), "Arm joint goal is out of bounds...");
            return;
        }
        // 5. Create an instance or object from planning interface
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        // 6. Plan the scene with new joint positions provided to arm_within_bounds earlier
        auto plan_result = arm_move_group.plan(arm_plan);
        // 7. Check for SUCC OR FAIL of Planner
        if(plan_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            // 8. If planning was successful, execute the motion...
            arm_move_group.move();
            // DEBUG
            auto current_pose = arm_move_group.getCurrentPose();
            RCLCPP_INFO(node->get_logger(),
                        "Current pose: x=%.15f y=%.15f z=%.15f qx=%.15f qy=%.15f qz=%.15f qw=%.15f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z,
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);
            // END OF DEBUG
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Planning Failed...");
            return;
        }

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Forward_Kinematics");
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Need to spin the node in seperate thread for moveit to work correctly.
    // This allows moveit to access /joint_states topic subscription so that it can get the information
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() {executor.spin();}).detach();

    // Now we can run the function or robot logic
    ForwardKinematics ik;
    ik.generate_inverse(node);

    rclcpp::shutdown();
    return 0;
}