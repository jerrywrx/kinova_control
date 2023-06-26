// Generated by gencpp from file kortex_driver/ProtectionZoneNotification.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_PROTECTIONZONENOTIFICATION_H
#define KORTEX_DRIVER_MESSAGE_PROTECTIONZONENOTIFICATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ProtectionZoneHandle.h>
#include <kortex_driver/Timestamp.h>
#include <kortex_driver/UserProfileHandle.h>
#include <kortex_driver/Connection.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ProtectionZoneNotification_
{
  typedef ProtectionZoneNotification_<ContainerAllocator> Type;

  ProtectionZoneNotification_()
    : event(0)
    , handle()
    , timestamp()
    , user_handle()
    , connection()  {
    }
  ProtectionZoneNotification_(const ContainerAllocator& _alloc)
    : event(0)
    , handle(_alloc)
    , timestamp(_alloc)
    , user_handle(_alloc)
    , connection(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _event_type;
  _event_type event;

   typedef  ::kortex_driver::ProtectionZoneHandle_<ContainerAllocator>  _handle_type;
  _handle_type handle;

   typedef  ::kortex_driver::Timestamp_<ContainerAllocator>  _timestamp_type;
  _timestamp_type timestamp;

   typedef  ::kortex_driver::UserProfileHandle_<ContainerAllocator>  _user_handle_type;
  _user_handle_type user_handle;

   typedef  ::kortex_driver::Connection_<ContainerAllocator>  _connection_type;
  _connection_type connection;





  typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> const> ConstPtr;

}; // struct ProtectionZoneNotification_

typedef ::kortex_driver::ProtectionZoneNotification_<std::allocator<void> > ProtectionZoneNotification;

typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneNotification > ProtectionZoneNotificationPtr;
typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneNotification const> ProtectionZoneNotificationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator2> & rhs)
{
  return lhs.event == rhs.event &&
    lhs.handle == rhs.handle &&
    lhs.timestamp == rhs.timestamp &&
    lhs.user_handle == rhs.user_handle &&
    lhs.connection == rhs.connection;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2ba41a2c5a55d12e33dfcd059ebe2676";
  }

  static const char* value(const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2ba41a2c5a55d12eULL;
  static const uint64_t static_value2 = 0x33dfcd059ebe2676ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ProtectionZoneNotification";
  }

  static const char* value(const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 event\n"
"ProtectionZoneHandle handle\n"
"Timestamp timestamp\n"
"UserProfileHandle user_handle\n"
"Connection connection\n"
"================================================================================\n"
"MSG: kortex_driver/ProtectionZoneHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Timestamp\n"
"\n"
"uint32 sec\n"
"uint32 usec\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Connection\n"
"\n"
"UserProfileHandle user_handle\n"
"string connection_information\n"
"uint32 connection_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.event);
      stream.next(m.handle);
      stream.next(m.timestamp);
      stream.next(m.user_handle);
      stream.next(m.connection);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ProtectionZoneNotification_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ProtectionZoneNotification_<ContainerAllocator>& v)
  {
    s << indent << "event: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.event);
    s << indent << "handle: ";
    s << std::endl;
    Printer< ::kortex_driver::ProtectionZoneHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.handle);
    s << indent << "timestamp: ";
    s << std::endl;
    Printer< ::kortex_driver::Timestamp_<ContainerAllocator> >::stream(s, indent + "  ", v.timestamp);
    s << indent << "user_handle: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfileHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.user_handle);
    s << indent << "connection: ";
    s << std::endl;
    Printer< ::kortex_driver::Connection_<ContainerAllocator> >::stream(s, indent + "  ", v.connection);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_PROTECTIONZONENOTIFICATION_H
