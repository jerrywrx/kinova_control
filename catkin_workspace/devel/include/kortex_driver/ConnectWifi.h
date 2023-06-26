// Generated by gencpp from file kortex_driver/ConnectWifi.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONNECTWIFI_H
#define KORTEX_DRIVER_MESSAGE_CONNECTWIFI_H

#include <ros/service_traits.h>


#include <kortex_driver/ConnectWifiRequest.h>
#include <kortex_driver/ConnectWifiResponse.h>


namespace kortex_driver
{

struct ConnectWifi
{

typedef ConnectWifiRequest Request;
typedef ConnectWifiResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ConnectWifi
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ConnectWifi > {
  static const char* value()
  {
    return "fc936133533cfced4ae662bec0d72a39";
  }

  static const char* value(const ::kortex_driver::ConnectWifi&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ConnectWifi > {
  static const char* value()
  {
    return "kortex_driver/ConnectWifi";
  }

  static const char* value(const ::kortex_driver::ConnectWifi&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ConnectWifiRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ConnectWifi >
template<>
struct MD5Sum< ::kortex_driver::ConnectWifiRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ConnectWifi >::value();
  }
  static const char* value(const ::kortex_driver::ConnectWifiRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ConnectWifiRequest> should match
// service_traits::DataType< ::kortex_driver::ConnectWifi >
template<>
struct DataType< ::kortex_driver::ConnectWifiRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ConnectWifi >::value();
  }
  static const char* value(const ::kortex_driver::ConnectWifiRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ConnectWifiResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ConnectWifi >
template<>
struct MD5Sum< ::kortex_driver::ConnectWifiResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ConnectWifi >::value();
  }
  static const char* value(const ::kortex_driver::ConnectWifiResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ConnectWifiResponse> should match
// service_traits::DataType< ::kortex_driver::ConnectWifi >
template<>
struct DataType< ::kortex_driver::ConnectWifiResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ConnectWifi >::value();
  }
  static const char* value(const ::kortex_driver::ConnectWifiResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONNECTWIFI_H
