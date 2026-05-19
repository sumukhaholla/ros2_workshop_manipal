#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <ros2_msgs/srv/get_centroid.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // --- 1. Node options: allow use_sim_time to be injected from launch file ---
    // rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);

    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("use_sim_time", true);
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("moveit_planner", node_options);

    // -- 2. Spin the node in a BACKGROUND thread so MoveIt subscriptions fire ---
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // --- 3. Call the centroid service ---
    auto client = node->create_client<ros2_msgs::srv::GetCentroid>("get_centroid");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) { executor.cancel(); executor_thread.join(); return 1; }
        RCLCPP_ERROR(node->get_logger(), "Waiting for get_centroid service...");
    }

    auto request = std::make_shared<ros2_msgs::srv::GetCentroid::Request>();
    auto future = client->async_send_request(request);

    // --- 4. Wait for centroid response ---
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "Centroid service timed out");
        executor.cancel(); executor_thread.join();
        rclcpp::shutdown(); return 1;
    }
    auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN(node->get_logger(), "No centroid available");
        executor.cancel(); executor_thread.join();
        rclcpp::shutdown(); return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Centroid: x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f",
                response->x, response->y, response->z, response->qx, response->qy, response->qw);

    // --- 5. MoveGroupInterface — node is already spinning in background ---
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    // arm.setPlannerId("RRTConnect");
    arm.setPlanningTime(5.0);

    // getCurrentState() will now work because executor is spinning in background
    if (!arm.getCurrentState(10.0)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get current state");
        executor.cancel(); executor_thread.join();
        rclcpp::shutdown(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Got current state");

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector: %s", arm.getEndEffectorLink().c_str());
    auto current = arm.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), 
        "Current EEF pose: x=%.4f y=%.4f z=%.4f | qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
        current.pose.position.x, current.pose.position.y, current.pose.position.z,
        current.pose.orientation.x, current.pose.orientation.y,
        current.pose.orientation.z, current.pose.orientation.w);

    // --- 6. Plan and execute ---
    geometry_msgs::msg::Pose target;
    target.position.x = response->x;
    target.position.y = response->y;
    target.position.z = 1.000;
    target.orientation.x = response->qx;
    target.orientation.y = response->qy;
    target.orientation.z = response->qz;
    target.orientation.w = response->qw;

    arm.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        arm.move();
        RCLCPP_INFO(node->get_logger(), "Motion executed");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed");
    }

    // --- 7. Attaching objects to gripper ---
    auto attach_client =
        node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");

    while (!attach_client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Waiting for ATTACHLINK service...");
    }

    auto attach_request =
        std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();

    attach_request->model1_name = "Model";
    attach_request->link1_name  = "link_6__1__1";
    attach_request->model2_name = "medical_scalp_clone";
    attach_request->link2_name  = "link";

    auto attach_future = attach_client->async_send_request(attach_request);

    if (attach_future.wait_for(5s) == std::future_status::ready) {
        RCLCPP_INFO(node->get_logger(), "Object attached successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to attach object");
    }
    
    // --- 8. Go to home ---
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan home;
    auto home_result = arm.plan(home);
    if(home_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.move();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning Failed for Home state...");
        return 1;
    }

    // --- 9. Go to incision place ---
    arm.clearPoseTargets();
    std::vector<double> arm_joint_goal {-2.9147, -0.6807, -0.8727, 1.9373, 0.0349, 3.1067};
    bool arm_within_bounds = arm.setJointValueTarget(arm_joint_goal);
    if(!arm_within_bounds)
    {
        RCLCPP_WARN(node->get_logger(), "Arm joint for incesion goal is out of bounds");
        return 1;
    }
    moveit::planning_interface::MoveGroupInterface::Plan incision_plan;
    auto plan_result = arm.plan(incision_plan);
    if(plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.move();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning Failed");
        return 1;
    }

    // --- 10. Go to place position
    arm.clearPoseTargets();
    arm_joint_goal = {0.0349, 0.1047, -0.0524, 0.1222, 0.0000, 0.0349};
    arm_within_bounds = arm.setJointValueTarget(arm_joint_goal);
    if(!arm_within_bounds)
    {
        RCLCPP_WARN(node->get_logger(), "Arm joint for place goal is out of bounds");
        return 1;
    }
    moveit::planning_interface::MoveGroupInterface::Plan place_plan;
    plan_result = arm.plan(place_plan);
    if(plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.move();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning Failed");
        return 1;
    }

    // --- 11. Detach an object from gripper ---
    auto detach_client =
        node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    while (!detach_client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Waiting for DETACHLINK service...");
    }

    auto detach_request =
        std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();

    detach_request->model1_name = "Model";
    detach_request->link1_name  = "link_6__1__1";
    detach_request->model2_name = "medical_scalp_clone";
    detach_request->link2_name  = "link";

    auto detach_future = detach_client->async_send_request(detach_request);

    if (detach_future.wait_for(5s) == std::future_status::ready) {
        RCLCPP_INFO(node->get_logger(), "Object detached successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to detach object");
    }

    // --- 12. Go to home position ---
    arm.setNamedTarget("home");
    home_result = arm.plan(home);
    if(home_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        arm.move();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning Failed for Home state...");
        return 1;
    }

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}