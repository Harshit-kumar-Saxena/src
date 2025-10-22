// #include <memory>
// #include <chrono>
// #include <vector>
// #include <string>
// #include "rclcpp/rclcpp.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
// #include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// using namespace std::chrono_literals;

// class TrajectoryPublisher : public rclcpp::Node
// {
// public:
//   TrajectoryPublisher()
//   : Node("trajectory_publisher")
//   {
//     traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//       "/arm_controller/joint_trajectory", 10);

//     timer_ = this->create_wall_timer(
//       2s, std::bind(&TrajectoryPublisher::publish_trajectory, this));
//   }

// private:
//   void publish_trajectory()
//   {
//     auto traj_msg = trajectory_msgs::msg::JointTrajectory();
//     traj_msg.joint_names = {
//       "base_link_link1", "link1_link2", "link2_link3",
//       "link3_link4", "link4_link5", "link5_tcp_connector"
//     };

//     trajectory_msgs::msg::JointTrajectoryPoint point;
//     point.positions = {0.0, -0.5, 0.5, -0.3, 0.2, 0.0};  // Example positions
//     point.time_from_start = rclcpp::Duration(3s);

//     traj_msg.points.push_back(point);

//     RCLCPP_INFO(this->get_logger(), "Publishing trajectory point...");
//     traj_pub_->publish(traj_msg);
//   }

//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<TrajectoryPublisher>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
