// Generated by gencpp from file kortex_driver/GetWifiCountryCodeResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETWIFICOUNTRYCODERESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETWIFICOUNTRYCODERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/CountryCode.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetWifiCountryCodeResponse_
{
  typedef GetWifiCountryCodeResponse_<ContainerAllocator> Type;

  GetWifiCountryCodeResponse_()
    : output()  {
    }
  GetWifiCountryCodeResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::CountryCode_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetWifiCountryCodeResponse_

typedef ::kortex_driver::GetWifiCountryCodeResponse_<std::allocator<void> > GetWifiCountryCodeResponse;

typedef boost::shared_ptr< ::kortex_driver::GetWifiCountryCodeResponse > GetWifiCountryCodeResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetWifiCountryCodeResponse const> GetWifiCountryCodeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "29ff9348c5c8343d487a90668267a29e";
  }

  static const char* value(const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x29ff9348c5c8343dULL;
  static const uint64_t static_value2 = 0x487a90668267a29eULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetWifiCountryCodeResponse";
  }

  static const char* value(const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "CountryCode output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/CountryCode\n"
"\n"
"uint32 identifier\n"
;
  }

  static const char* value(const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetWifiCountryCodeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetWifiCountryCodeResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::CountryCode_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETWIFICOUNTRYCODERESPONSE_H
