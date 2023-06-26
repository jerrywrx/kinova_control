// Generated by gencpp from file kortex_driver/GetAllSafetyConfiguration.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETALLSAFETYCONFIGURATION_H
#define KORTEX_DRIVER_MESSAGE_GETALLSAFETYCONFIGURATION_H

#include <ros/service_traits.h>


#include <kortex_driver/GetAllSafetyConfigurationRequest.h>
#include <kortex_driver/GetAllSafetyConfigurationResponse.h>


namespace kortex_driver
{

struct GetAllSafetyConfiguration
{

typedef GetAllSafetyConfigurationRequest Request;
typedef GetAllSafetyConfigurationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetAllSafetyConfiguration
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetAllSafetyConfiguration > {
  static const char* value()
  {
    return "332c0fb8c1c499e5ec5c674488e0110f";
  }

  static const char* value(const ::kortex_driver::GetAllSafetyConfiguration&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetAllSafetyConfiguration > {
  static const char* value()
  {
    return "kortex_driver/GetAllSafetyConfiguration";
  }

  static const char* value(const ::kortex_driver::GetAllSafetyConfiguration&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyConfigurationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyConfiguration >
template<>
struct MD5Sum< ::kortex_driver::GetAllSafetyConfigurationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllSafetyConfigurationRequest> should match
// service_traits::DataType< ::kortex_driver::GetAllSafetyConfiguration >
template<>
struct DataType< ::kortex_driver::GetAllSafetyConfigurationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyConfigurationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetAllSafetyConfiguration >
template<>
struct MD5Sum< ::kortex_driver::GetAllSafetyConfigurationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetAllSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyConfigurationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetAllSafetyConfigurationResponse> should match
// service_traits::DataType< ::kortex_driver::GetAllSafetyConfiguration >
template<>
struct DataType< ::kortex_driver::GetAllSafetyConfigurationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetAllSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::GetAllSafetyConfigurationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETALLSAFETYCONFIGURATION_H
