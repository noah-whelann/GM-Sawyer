// Generated by gencpp from file chess_tracking/ScreenshotRequest.msg
// DO NOT EDIT!


#ifndef CHESS_TRACKING_MESSAGE_SCREENSHOTREQUEST_H
#define CHESS_TRACKING_MESSAGE_SCREENSHOTREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace chess_tracking
{
template <class ContainerAllocator>
struct ScreenshotRequest_
{
  typedef ScreenshotRequest_<ContainerAllocator> Type;

  ScreenshotRequest_()
    {
    }
  ScreenshotRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ScreenshotRequest_

typedef ::chess_tracking::ScreenshotRequest_<std::allocator<void> > ScreenshotRequest;

typedef boost::shared_ptr< ::chess_tracking::ScreenshotRequest > ScreenshotRequestPtr;
typedef boost::shared_ptr< ::chess_tracking::ScreenshotRequest const> ScreenshotRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chess_tracking::ScreenshotRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace chess_tracking

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::chess_tracking::ScreenshotRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chess_tracking/ScreenshotRequest";
  }

  static const char* value(const ::chess_tracking::ScreenshotRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::chess_tracking::ScreenshotRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScreenshotRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chess_tracking::ScreenshotRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::chess_tracking::ScreenshotRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CHESS_TRACKING_MESSAGE_SCREENSHOTREQUEST_H
