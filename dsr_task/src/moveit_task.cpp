// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>

// class MoveItTaskNode : public rclcpp::Node
// {
// public:
//     MoveItTaskNode() : Node("moveit_task_node")
//     {
//         // Move group interface
//         auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             shared_from_this(),  // rclcpp::Node의 shared_from_this()를 호출
//             "arm"                // 플래닝 그룹 이름
//         );

//         // Get basic information
//         RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group->getPlanningFrame().c_str());
//         RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group->getEndEffectorLink().c_str());

//         // Set a target pose
//         geometry_msgs::msg::Pose target_pose;
//         target_pose.orientation.w = 1.0;
//         target_pose.position.x = 0.4;
//         target_pose.position.y = 0.1;
//         target_pose.position.z = 0.4;

//         move_group->setPoseTarget(target_pose);

//         // Plan and execute
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         auto success = move_group->plan(plan);

//         if (success == moveit::core::MoveItErrorCode::SUCCESS)
//         {
//             RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
//             move_group->execute(plan);
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "Plan failed!");
//         }
//     }
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveItTaskNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
