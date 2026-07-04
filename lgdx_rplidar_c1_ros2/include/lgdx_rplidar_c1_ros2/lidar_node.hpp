#ifndef LIDAR_NODE_HPP
#define LIDAR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"
#include "config.hpp"

class LidarNode : public rclcpp::Node
{
  public:
    LidarNode();
    void Initalise();
    
    boost::asio::awaitable<void> Main();
    boost::asio::awaitable<void> LidarInitalise();

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<boost::asio::io_context> io_context_;
    std::shared_ptr<SerialPort> serial_port_;
    std::unique_ptr<Config> config_;
};

#endif // LIDAR_NODE_HPP