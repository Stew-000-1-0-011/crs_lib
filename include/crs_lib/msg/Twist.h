// Generated by gencpp from file crs_lib/Twist.msg
// DO NOT EDIT!


#ifndef CRS_LIB_MESSAGE_TWIST_H
#define CRS_LIB_MESSAGE_TWIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace crs_lib
{
template <class ContainerAllocator>
struct Twist_
{
  typedef Twist_<ContainerAllocator> Type;

  Twist_()
    : header()
    , linear_x(0.0)
    , linear_y(0.0)
    , angular_z(0.0)  {
    }
  Twist_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , linear_x(0.0)
    , linear_y(0.0)
    , angular_z(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _linear_x_type;
  _linear_x_type linear_x;

   typedef float _linear_y_type;
  _linear_y_type linear_y;

   typedef float _angular_z_type;
  _angular_z_type angular_z;





  typedef boost::shared_ptr< ::crs_lib::Twist_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crs_lib::Twist_<ContainerAllocator> const> ConstPtr;

}; // struct Twist_

typedef ::crs_lib::Twist_<std::allocator<void> > Twist;

typedef boost::shared_ptr< ::crs_lib::Twist > TwistPtr;
typedef boost::shared_ptr< ::crs_lib::Twist const> TwistConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crs_lib::Twist_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crs_lib::Twist_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::crs_lib::Twist_<ContainerAllocator1> & lhs, const ::crs_lib::Twist_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.linear_x == rhs.linear_x &&
    lhs.linear_y == rhs.linear_y &&
    lhs.angular_z == rhs.angular_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::crs_lib::Twist_<ContainerAllocator1> & lhs, const ::crs_lib::Twist_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace crs_lib

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::crs_lib::Twist_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crs_lib::Twist_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crs_lib::Twist_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crs_lib::Twist_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crs_lib::Twist_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crs_lib::Twist_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crs_lib::Twist_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5999df108e48bbbdb99eef7365cae6d9";
  }

  static const char* value(const ::crs_lib::Twist_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5999df108e48bbbdULL;
  static const uint64_t static_value2 = 0xb99eef7365cae6d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::crs_lib::Twist_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crs_lib/Twist";
  }

  static const char* value(const ::crs_lib::Twist_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crs_lib::Twist_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 linear_x\n"
"float32 linear_y\n"
"float32 angular_z\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::crs_lib::Twist_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crs_lib::Twist_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.linear_x);
      stream.next(m.linear_y);
      stream.next(m.angular_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Twist_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crs_lib::Twist_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crs_lib::Twist_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "linear_x: ";
    Printer<float>::stream(s, indent + "  ", v.linear_x);
    s << indent << "linear_y: ";
    Printer<float>::stream(s, indent + "  ", v.linear_y);
    s << indent << "angular_z: ";
    Printer<float>::stream(s, indent + "  ", v.angular_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRS_LIB_MESSAGE_TWIST_H
