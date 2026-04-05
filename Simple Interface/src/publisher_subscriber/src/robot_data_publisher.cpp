#include "rclcpp/rclcpp.hpp"
#include "custom_interface/msg/robot_data.hpp"

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node
{
   rclcpp::Publisher<custom_interface::msg::RobotData>::SharedPtr m_publisher;
   rclcpp::TimerBase::SharedPtr m_timer;

   public:
   void SendData();
   DataPublisher() : Node("DataPublisher")
   {
      m_publisher = create_publisher<custom_interface::msg::RobotData>("robot_data", 10);

      m_timer = create_wall_timer(1s, std::bind(&DataPublisher::SendData, this));
   }
};

void DataPublisher::SendData()
{
   custom_interface::msg::RobotData data;
   data.robot_name= "Default Robot";
   data.last_updated = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
   if (m_publisher)
   {
       m_publisher->publish(data);
       RCLCPP_INFO(get_logger(), "Robot name: %s, time stamp: %lu", data.robot_name.c_str(), data.last_updated);
   }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataPublisher>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
