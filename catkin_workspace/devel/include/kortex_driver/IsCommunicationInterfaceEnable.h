// Generated by gencpp from file kortex_driver/IsCommunicationInterfaceEnable.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ISCOMMUNICATIONINTERFACEENABLE_H
#define KORTEX_DRIVER_MESSAGE_ISCOMMUNICATIONINTERFACEENABLE_H

#include <ros/service_traits.h>


#include <kortex_driver/IsCommunicationInterfaceEnableRequest.h>
#include <kortex_driver/IsCommunicationInterfaceEnableResponse.h>


namespace kortex_driver
{

struct IsCommunicationInterfaceEnable
{

typedef IsCommunicationInterfaceEnableRequest Request;
typedef IsCommunicationInterfaceEnableResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct IsCommunicationInterfaceEnable
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnable > {
  static const char* value()
  {
    return "7b7c3f92182fedb31e77cfcc39090ac1";
  }

  static const char* value(const ::kortex_driver::IsCommunicationInterfaceEnable&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::IsCommunicationInterfaceEnable > {
  static const char* value()
  {
    return "kortex_driver/IsCommunicationInterfaceEnable";
  }

  static const char* value(const ::kortex_driver::IsCommunicationInterfaceEnable&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnableRequest> should match
// service_traits::MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnable >
template<>
struct MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnableRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::IsCommunicationInterfaceEnableRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::IsCommunicationInterfaceEnableRequest> should match
// service_traits::DataType< ::kortex_driver::IsCommunicationInterfaceEnable >
template<>
struct DataType< ::kortex_driver::IsCommunicationInterfaceEnableRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::IsCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::IsCommunicationInterfaceEnableRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnableResponse> should match
// service_traits::MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnable >
template<>
struct MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnableResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::IsCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::IsCommunicationInterfaceEnableResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::IsCommunicationInterfaceEnableResponse> should match
// service_traits::DataType< ::kortex_driver::IsCommunicationInterfaceEnable >
template<>
struct DataType< ::kortex_driver::IsCommunicationInterfaceEnableResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::IsCommunicationInterfaceEnable >::value();
  }
  static const char* value(const ::kortex_driver::IsCommunicationInterfaceEnableResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ISCOMMUNICATIONINTERFACEENABLE_H
