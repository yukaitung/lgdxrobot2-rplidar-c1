#ifndef GET_CONFIG_EXCEPTION_HPP
#define GET_CONFIG_EXCEPTION_HPP

#include <exception>
#include <string>

class GetConfigException : public std::exception
{
  public:
    explicit GetConfigException(const std::string &message) : message_(message) {}
    const char* what() const noexcept override { return message_.c_str(); }
  private:
    std::string message_;
};

#endif // GET_CONFIG_EXCEPTION_HPP