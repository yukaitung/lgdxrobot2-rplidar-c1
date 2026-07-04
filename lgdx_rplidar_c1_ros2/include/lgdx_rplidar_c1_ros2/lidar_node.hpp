#ifndef LIDAR_NODE_HPP
#define LIDAR_NODE_HPP

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"
#include "config.hpp"
#include "scan.hpp"

class LidarNode : public rclcpp::Node
{
  public:
    LidarNode();
    void Initalise();

    boost::asio::awaitable<void> Main();
    boost::asio::awaitable<bool> SelfCheck();

  private:
    const int kMaxHealthRetry = 3;
    const int kHealthRetryWaitMs = 500;

    int health_retry_count_ = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    
    std::shared_ptr<boost::asio::io_context> io_context_;
    std::shared_ptr<SerialPort> serial_port_;
    std::unique_ptr<Config> config_;
    std::unique_ptr<Scan> scan_;
};

#endif // LIDAR_NODE_HPP