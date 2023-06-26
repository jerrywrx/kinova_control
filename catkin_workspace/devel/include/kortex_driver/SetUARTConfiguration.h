// Generated by gencpp from file kortex_driver/SetUARTConfiguration.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETUARTCONFIGURATION_H
#define KORTEX_DRIVER_MESSAGE_SETUARTCONFIGURATION_H

#include <ros/service_traits.h>


#include <kortex_driver/SetUARTConfigurationRequest.h>
#include <kortex_driver/SetUARTConfigurationResponse.h>


namespace kortex_driver
{

struct SetUARTConfiguration
{

typedef SetUARTConfigurationRequest Request;
typedef SetUARTConfigurationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetUARTConfiguration
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetUARTConfiguration > {
  static const char* value()
  {
    return "44945f03b0c880ed6bb57eedba493047";
  }

  static const char* value(const ::kortex_driver::SetUARTConfiguration&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetUARTConfiguration > {
  static const char* value()
  {
    return "kortex_driver/SetUARTConfiguration";
  }

  static const char* value(const ::kortex_driver::SetUARTConfiguration&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetUARTConfigurationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetUARTConfiguration >
template<>
struct MD5Sum< ::kortex_driver::SetUARTConfigurationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetUARTConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetUARTConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetUARTConfigurationRequest> should match
// service_traits::DataType< ::kortex_driver::SetUARTConfiguration >
template<>
struct DataType< ::kortex_driver::SetUARTConfigurationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetUARTConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetUARTConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetUARTConfigurationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetUARTConfiguration >
template<>
struct MD5Sum< ::kortex_driver::SetUARTConfigurationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetUARTConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetUARTConfigurationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetUARTConfigurationResponse> should match
// service_traits::DataType< ::kortex_driver::SetUARTConfiguration >
template<>
struct DataType< ::kortex_driver::SetUARTConfigurationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetUARTConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetUARTConfigurationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETUARTCONFIGURATION_H
