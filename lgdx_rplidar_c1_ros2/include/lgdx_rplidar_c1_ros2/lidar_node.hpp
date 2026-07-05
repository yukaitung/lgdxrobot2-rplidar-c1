#ifndef LIDAR_NODE_HPP
#define LIDAR_NODE_HPP

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"
#include "config.hpp"
#include "scan/scan_base.hpp"

class LidarNode : public rclcpp::Node
{
  public:
    LidarNode();
    void Initalise();

    void ConnectSerialPort();
    
    boost::asio::awaitable<void> Main();
    boost::asio::awaitable<bool> SelfCheck();
    void PublishScan(const std::vector<LidarScanData> &scans, 
      size_t valid_scan_start, size_t valid_scans_count,
      const float angle_max, const float angle_min,
      const rclcpp::Time &start_time, const float scan_time);

  private:
    const int kMaxHealthRetry = 3;
    const int kHealthRetryWaitMs = 500;
    const int kRetryWaitSecond = 3;
    const float kScanMinDistance = 0.05f;

    int health_retry_count_ = 0;
    std::string frame_id_;
    bool inverted_;

    LidarScanMode current_scan_mode_;
    float scan_frequency_ = 10.0f; // default frequent is 10 hz (by motor pwm value)
    bool angle_compensate_ = false;
    size_t angle_compensate_multiple = 1; // It stand of angle compensate at per 1 degree

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    
    std::shared_ptr<boost::asio::io_context> io_context_;
    std::shared_ptr<SerialPort> serial_port_;
    std::unique_ptr<Config> config_;
    std::unique_ptr<ScanBase> scan_;
};

#endif // LIDAR_NODE_HPP