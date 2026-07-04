#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"

class SerialPort
{
  public:
    SerialPort(rclcpp::Node::SharedPtr node, std::shared_ptr<boost::asio::io_context> io_context);
    ~SerialPort();

    void Connect();
    void StartSerialThread();

    boost::asio::awaitable<void> Write(const std::vector<uint8_t> data);
    boost::asio::awaitable<std::vector<uint8_t>> WriteRead(const std::vector<uint8_t> data);
    boost::asio::awaitable<std::vector<uint8_t>> Read();

    boost::asio::awaitable<void> Stop();
    boost::asio::awaitable<void> Reset();

  private:
    unsigned int kWaitSecond = 5;

    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    std::shared_ptr<boost::asio::io_context> io_context_;
    boost::asio::serial_port serial_;
    std::thread serial_thread_;

    std::array<uint8_t, 512> read_buffer_ = {0};

    std::string port_name_;
    unsigned int port_baudrate_;

    void StopBlk();
};

#endif // SERIAL_PORT_HPP