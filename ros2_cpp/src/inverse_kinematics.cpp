/** @file inverse_kinematics.cpp
 * This script still uses moveit.launch.py, then we can run this script by providing Pose instead of Joint vector
 * Total of 7 steps carried out
 */
/** @author Sumukha B <sumukha.b@siemens-healthineers.com> */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
// Moveit Core
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>

class InverseKinematics
{
public:
    void generate_inverse(const std::shared_ptr<rclcpp::Node>& node)
    {
        // 1. Setup:
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
        // 1.1 Set Planner and a planning time limit
        arm_move_group.setPlannerId("RRTConnect");
        arm_move_group.setPlanningTime(15.0);

        // DEBUG: Getting the current pose. COMMENT THIS IF YOU ALREADY KNOW THE STEP 2

        RCLCPP_INFO(node->get_logger(),
                    "End effector link: %s",
                    arm_move_group.getEndEffectorLink().c_str());


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

        // 2. Provide IK Target (x,y,z + quaternions) IF YOU ALREADY KNOW THE POSE, OTHERWISE GO FOR DEBUG APPRACH DOWN BELOW
        geometry_msgs::msg::Pose target_pose;

        target_pose.position.x = -0.1644; //0.1568, -0.0538
        target_pose.position.y = 0.1240; // 0.1310, 0.1257
        target_pose.position.z = 1.000; // 1.0157

        target_pose.orientation.x = 0.5015;
        target_pose.orientation.y = -0.4981;
        target_pose.orientation.z = 0.4957;
        target_pose.orientation.w = -0.4987;

        // 2. DEBUG: Slightly modify the current pose thats it
        /*geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x = 0.724;*/
        // END OF DEBUG 2

        // 3. Set IK target
        arm_move_group.setPoseTarget(target_pose);
        // arm_move_group.setPositionTarget(0.14830002924321, 0.133647414256070, 1.015735229400975);

        // 4. Create an instance or object from planning interface
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        // 5. Plan the scene with new joint positions provided to arm_within_bounds earlier
        auto plan_result = arm_move_group.plan(arm_plan);
        // 6. Check for SUCC OR FAIL of Planner
        if(plan_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            // 7. If planning was successful, execute the motion...
            arm_move_group.move();
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Planning Failed...");
            return;
        }
        // 7. Move to Home if earlier planning was success
        arm_move_group.setNamedTarget("home");
        moveit::planning_interface::MoveGroupInterface::Plan home;
        auto home_result = arm_move_group.plan(home);
        if(home_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            arm_move_group.move();
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Planning Failed for Home state...");
            return;
        }

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("InverseKinematics");
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    InverseKinematics ik;
    ik.generate_inverse(node);

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}