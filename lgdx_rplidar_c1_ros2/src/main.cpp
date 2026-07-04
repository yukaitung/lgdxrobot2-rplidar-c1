#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

using boost::asio::awaitable;

class Test : public rclcpp::Node
{
  public:
    Test() : Node("test_node"),
      io_context_(),
      serial_(io_context_)
    {
      RCLCPP_INFO(this->get_logger(), "Hello World!");
      timer_ = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
    }

    ~Test()
    {
      serial_.close();
      io_context_.stop();
    }

    template <typename CompletionToken>
    auto TestCoAwait(CompletionToken&& token)
    {
      return boost::asio::async_compose<CompletionToken, void()>
      (
        [this](auto&& self)
        {
          std::thread(
            [self = std::move(self), this]() mutable
            {
              RCLCPP_INFO(this->get_logger(), "Hello World!!!!!");
              self.complete();
            }
          ).detach();
        },
        token
      );
    }

    void Initalise()
    {
      timer_->cancel();
      this->Connect();
      boost::asio::co_spawn(io_context_, Test::Send(), boost::asio::detached);
      io_context_.run();
    }

    void Connect()
    {
      boost::system::error_code error;
      serial_.open("/dev/ttyUSB0", error);
      if(error) 
      {
        RCLCPP_ERROR(this->get_logger(), "Serial connection throws an error: %s.", error.message().c_str());
        return;
      }
      serial_.set_option(boost::asio::serial_port_base::baud_rate(460800));
    }

    awaitable<void> Send()
    {
      co_await TestCoAwait(boost::asio::use_awaitable);
      std::vector<uint8_t> buffer{ 0xA5, 0x50 };
      try
      {
        co_await serial_.async_write_some(boost::asio::buffer(buffer), boost::asio::use_awaitable);
        std::array<uint8_t, 512> readBuffer = {0};
        std::size_t size = co_await serial_.async_read_some(boost::asio::buffer(readBuffer), boost::asio::use_awaitable); 
        RCLCPP_INFO(this->get_logger(), "Read %ld bytes", size);

        uint8_t majorModel = readBuffer[7] >> 4;
        uint8_t minorModel = readBuffer[7] & 0x0F;
        uint8_t firmwareMajor = readBuffer[8];
        uint8_t firmwareMinor = readBuffer[9];
        uint8_t hardwareMajor = readBuffer[10];
        RCLCPP_INFO(this->get_logger(), "Major Model: %d", majorModel);
        RCLCPP_INFO(this->get_logger(), "Minor Model: %d", minorModel);
        RCLCPP_INFO(this->get_logger(), "Firmware Version: %d.%d", firmwareMajor, firmwareMinor);
        RCLCPP_INFO(this->get_logger(), "Hardware Version: %d", hardwareMajor);
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "Serial write throws an error: %s", e.what());
      }
    }

  private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  rclcpp::shutdown();
  return 0;
}