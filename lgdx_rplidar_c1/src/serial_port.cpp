#include "lgdx_rplidar_c1/serial_port.hpp"
#include "lgdx_rplidar_c1/exceptions/serial_port_exception.hpp"

SerialPort::SerialPort(rclcpp::Node::SharedPtr node, std::shared_ptr<boost::asio::io_context> io_context) :
  logger_(node->get_logger()),
  io_context_(io_context),
  serial_(*io_context)
{
  port_name_ = node->get_parameter("serial_port").as_string();
  port_baudrate_ = node->get_parameter("serial_baudrate").as_int();
}

SerialPort::~SerialPort()
{
  if(serial_thread_.joinable())
  {
    serial_thread_.join();
  }
}

void SerialPort::Connect()
{
  RCLCPP_INFO(logger_, "Attempting to connect to %s, baudrate: %d", port_name_.c_str(), port_baudrate_);

  if (serial_.is_open())
  {
    serial_.close();
  }

  boost::system::error_code error;
  serial_.open(port_name_, error);
  if(error) 
  {
    throw SerialPortException(error.message());
  }
  serial_.set_option(boost::asio::serial_port_base::baud_rate(port_baudrate_));
  RCLCPP_INFO(logger_, "Connected to %s", port_name_.c_str());
}

void SerialPort::StartSerialThread()
{
  if(serial_thread_.joinable()) 
  {
    io_context_->restart();
    serial_thread_.join();
  }
  std::thread thread{[this](){ io_context_->run(); }};
  serial_thread_.swap(thread);
}

boost::asio::awaitable<void> SerialPort::Write(const std::vector<uint8_t> data)
{
  try
  {
    co_await serial_.async_write_some(boost::asio::buffer(data), boost::asio::use_awaitable);
  }
  catch(const std::exception& e)
  {
    throw SerialPortException(e.what());
  }
}

boost::asio::awaitable<std::vector<uint8_t>> SerialPort::WriteRead(const std::vector<uint8_t> data)
{
  try
  {
    co_await serial_.async_write_some(boost::asio::buffer(data), boost::asio::use_awaitable);
    auto size = co_await serial_.async_read_some(boost::asio::buffer(read_buffer_), boost::asio::use_awaitable);
    std::vector<uint8_t> response(read_buffer_.begin(), read_buffer_.begin() + size);
    co_return response;
  }
  catch(const std::exception& e)
  {
    throw SerialPortException(e.what());
  }
}

boost::asio::awaitable<std::vector<uint8_t>> SerialPort::Read()
{
  try
  {
    auto size = co_await serial_.async_read_some(boost::asio::buffer(read_buffer_), boost::asio::use_awaitable);
    std::vector<uint8_t> response(read_buffer_.begin(), read_buffer_.begin() + size);
    co_return response;
  }
  catch(const std::exception& e)
  {
    throw SerialPortException(e.what());
  }
}

boost::asio::awaitable<void> SerialPort::Stop()
{
  std::vector<uint8_t> command = {0xA5, 0x25};
  co_await WriteRead(command);
}

boost::asio::awaitable<void> SerialPort::Reset()
{
  std::vector<uint8_t> command = {0xA5, 0x40};
  co_await WriteRead(command);
}

void SerialPort::Shutdown()
{
  std::vector<uint8_t> command = {0xA5, 0x25};
  boost::system::error_code error;
  serial_.write_some(boost::asio::buffer(command), error);
  if(error) 
  {
    // Do not throw an exception, just print the error message
    RCLCPP_ERROR(logger_, "Serial write throws an error: %s", error.message().c_str());
  }
}