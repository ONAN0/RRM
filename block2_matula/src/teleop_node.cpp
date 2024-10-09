#include <vector>
#include "rclcpp/rclcpp.hpp"
// #include "rrm_msgs/msg/command.hpp"
#include "rrm_msgs/srv/command.hpp"
#include <algorithm>

class Teleop : public rclcpp::Node
{
public:
  Teleop() : Node("Teleop"), joint_id_(0)
  {
    // publisher_ = this->create_publisher<rrm_msgs::msg::Command>("move_command", 10);
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

  bool move(const std::vector<double> &positions, double max_velocity)
  {
    if (positions.size() != 3)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid number of positions. Expected 3 positions.");
      return false;
    }

    std::vector<double> distances(3);
    for (size_t i = 0; i < 3; ++i)
    {
      distances[i] = std::abs(positions[i] - position_[i]);
    }

    double max_distance = *std::max_element(distances.begin(), distances.end());

    double time_required = max_distance / max_velocity;

    std::vector<double> velocities(3);
    for (size_t i = 0; i < 3; ++i)
    {
      velocities[i] = distances[i] / time_required;
    }

    auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
    request->positions = positions;
    request->velocities = velocities;

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed"); // Log an error if the service call failed
      return false;
    }

    auto response = result.get();
    RCLCPP_INFO(this->get_logger(), "Service call succeeded: result_code = %d, message = %s", response->result_code, response->message.c_str()); // Log the response
    return !(response->result_code);
  }

private:
  // rclcpp::Publisher<rrm_msgs::msg::Command>::SharedPtr publisher_;
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;
  int joint_id_;
  double position_[3];
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Teleop robot;

  std::vector<double> positions(3);
  double max_velocity;
  
  for (int i = 0; i < 3; ++i)
  {
    std::cout << "Enter target position for joint " << (i + 1) << ": ";
    std::cin >> positions[i];
  }

  std::cout << "Enter maximum velocity: ";
  std::cin >> max_velocity;

  robot.move(positions, max_velocity);

  rclcpp::shutdown();
  return 0;
}