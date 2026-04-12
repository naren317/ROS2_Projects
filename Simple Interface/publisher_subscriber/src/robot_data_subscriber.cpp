#include "rclcpp/rclcpp.hpp"
#include "custom_interface/msg/robot_data.hpp"
#include <format>

class DataSubscriber : public rclcpp::Node
{
private:
     rclcpp::Subscription<custom_interface::msg::RobotData>::SharedPtr m_subscriber;

public:
      DataSubscriber() : Node ("DataSubscriber")
      {
         m_subscriber = create_subscription<custom_interface::msg::RobotData>("robot_data", 10
         , std::bind(&DataSubscriber::OnDataReceived, this, std::placeholders::_1));
      }

     void OnDataReceived(const custom_interface::msg::RobotData& data);
};

void DataSubscriber::OnDataReceived(const custom_interface::msg::RobotData& data)
{
   //std::tm* tm = std::localtime(&data.last_updated);
   //auto timeStamp = std::format("Hour: {}, Min: {}, Sec: {}", tm->tm_hour, tm->tm_min, tm->tm_sec);
   //RCLCPP_INFO(get_logger(), "Received: %s, Time stamp: %s", data.robot_name.c_str(), timeStamp.c_str());
}

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<DataSubscriber>());
   rclcpp::shutdown();
   return EXIT_SUCCESS;
}
