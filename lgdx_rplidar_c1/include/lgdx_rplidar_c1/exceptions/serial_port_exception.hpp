#ifndef SERIAL_PORT_EXCEPTION_HPP
#define SERIAL_PORT_EXCEPTION_HPP

#include <exception>
#include <string>

class SerialPortException : public std::exception
{
  public:
    explicit SerialPortException(const std::string &message) : message_(message) {}
    const char* what() const noexcept override { return message_.c_str(); }
  private:
    std::string message_;
};

#endif // SERIAL_PORT_EXCEPTION_HPP