#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "matula_interface/srv/teach_point.hpp"

// header file pre msg interface
class JointLogger : public rclcpp::Node
{
public:
    JointLogger();
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void teach_point_callback(const std::shared_ptr<matula_interface::srv::TeachPoint::Request> request, std::shared_ptr<matula_interface::srv::TeachPoint::Response> response);

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Service<matula_interface::srv::TeachPoint>::SharedPtr service_;
};

JointLogger::JointLogger() : Node("joint_logger")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointLogger::joint_states_callback, this, std::placeholders::_1));
    service_ = this->create_service<matula_interface::srv::TeachPoint>("teach_point", std::bind(&JointLogger::teach_point_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void JointLogger::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    system("clear");
    RCLCPP_INFO(this->get_logger(), "First joint name: %s, position: %f", msg->name[0].c_str(), msg->position[0]);
    RCLCPP_INFO(this->get_logger(), "Second joint name: %s, position: %f", msg->name[1].c_str(), msg->position[1]);
    RCLCPP_INFO(this->get_logger(), "Third joint name: %s, position: %f", msg->name[2].c_str(), msg->position[2]);
}

void JointLogger::teach_point_callback(const std::shared_ptr<matula_interface::srv::TeachPoint::Request> request, std::shared_ptr<matula_interface::srv::TeachPoint::Response> response)
{
    float velocity = request->velocity;
    // TODO: vasa funkcionalita
    RCLCPP_INFO(this->get_logger(), "Request received: %f, sending response", request->velocity);
    response->result = true;
    response->message = "Response from joint logger";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<JointLogger> logger = std::make_shared<JointLogger>();
    // auto logger = std::make_shared<JointLogger>(); //alt. zapis
    rclcpp::spin(logger); // function that blocks the thread and allows the node to process callbacks
    rclcpp::shutdown();
    return 0;
}