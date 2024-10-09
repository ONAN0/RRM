#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/msg/command.hpp"
#include "rrm_msgs/srv/command.hpp"
class Teleop : public rclcpp::Node
{
public:
  Teleop() : Node("Teleop"), joint_id_(0)
  {
    publisher_ = this->create_publisher<rrm_msgs::msg::Command>("move_command", 10);
    client_ = this->create_client<rrm_msgs::srv::Command>("move_command");

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(), "Teleop initialized");

    position_[0] = 0.0;
    position_[1] = 0.0;
    position_[2] = 0.0;
  }
  void move(int joint_change, double position_change)
  {
    // TODO tu neskor napiseme ROS publisher
    joint_id_ += joint_change;
    joint_id_ = (joint_id_ + 3) % 3;

    position_[joint_id_] += position_change;

    rrm_msgs::msg::Command message;
    message.joint_id = joint_id_;
    message.position = position_[joint_id_];
    publisher_->publish(message);
  }

  bool moveWithSpeed(const std::vector<double> &positions, double max_velocity)
  {
    auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
    request->positions = positions;
    request->velocities = {0.5, 0.5, 0.5};

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed"); // Log an error if the service call failed
      return false;
    }

    auto response = result.get();
    RCLCPP_INFO(this->get_logger(), "Service call succeeded: result_code = %d, message = %s",response->result_code, response->message.c_str()); // Log the response
    return !result_code;
  }

private:
  rclcpp::Publisher<rrm_msgs::msg::Command>::SharedPtr publisher_;
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;
  int joint_id_;
  double position_[3];
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Teleop robot;

  system("stty raw");

  while (1)
  {
    system("clear");

    char move = getchar();

    if (move == 'Q' || move == 'q')
    {
      system("clear");
      break;
    }

    // Spracovanie pohybu hráča
    switch (move)
    {
    case 'W':
    case 'w':
      robot.move(0, 0.1);
      break;
    case 'S':
    case 's':
      robot.move(0, -0.1);
      break;
    case 'A':
    case 'a':
      robot.move(-1, 0);
      break;
    case 'D':
    case 'd':
      robot.move(1, 0);
      break;
    }
  }

  // Nastavenie terminalu do módu "cooked" resetuje nastavenie "raw"
  system("stty cooked");

  return 0;
}