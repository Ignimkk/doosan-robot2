#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // ROS 초기화 및 노드 생성
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // ROS 로거 생성
  auto const logger = rclcpp::get_logger("hello_moveit");

  // MoveIt MoveGroup 인터페이스 생성
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // 목표 Pose 설정
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // 목표 Pose로의 계획 생성
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // 계획 실행
  if (success)
  {
    move_group_interface.execute(plan);
    rclcpp::sleep_for(std::chrono::seconds(5));  // 실행 완료 시간을 주기 위해 슬립 추가
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // 콜백을 처리하기 위해 노드 스핀
  rclcpp::spin_some(node);

  // ROS 종료
  rclcpp::shutdown();
  return 0;
}