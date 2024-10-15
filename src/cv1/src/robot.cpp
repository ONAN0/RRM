#include "cv1/robot.hpp"
#include "rclcpp/rclcpp.hpp"
Robot::Robot() : x_coordinate_(0.0), y_coordinate_(0.0)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello I'm robot");
}
void Robot::move(const double x, const double y)
{
    this->x_coordinate_ += x;
    this->y_coordinate_ += y;
}
double Robot::getX() const
{
    return x_coordinate_;
}
double Robot::getY() const
{
    return y_coordinate_;
}