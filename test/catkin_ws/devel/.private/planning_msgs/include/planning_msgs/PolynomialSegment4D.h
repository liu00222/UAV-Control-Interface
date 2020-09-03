// Generated by gencpp from file planning_msgs/PolynomialSegment4D.msg
// DO NOT EDIT!


#ifndef PLANNING_MSGS_MESSAGE_POLYNOMIALSEGMENT4D_H
#define PLANNING_MSGS_MESSAGE_POLYNOMIALSEGMENT4D_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace planning_msgs
{
template <class ContainerAllocator>
struct PolynomialSegment4D_
{
  typedef PolynomialSegment4D_<ContainerAllocator> Type;

  PolynomialSegment4D_()
    : header()
    , num_coeffs(0)
    , segment_time()
    , x()
    , y()
    , z()
    , yaw()  {
    }
  PolynomialSegment4D_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , num_coeffs(0)
    , segment_time()
    , x(_alloc)
    , y(_alloc)
    , z(_alloc)
    , yaw(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _num_coeffs_type;
  _num_coeffs_type num_coeffs;

   typedef ros::Duration _segment_time_type;
  _segment_time_type segment_time;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _x_type;
  _x_type x;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _y_type;
  _y_type y;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _z_type;
  _z_type z;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> const> ConstPtr;

}; // struct PolynomialSegment4D_

typedef ::planning_msgs::PolynomialSegment4D_<std::allocator<void> > PolynomialSegment4D;

typedef boost::shared_ptr< ::planning_msgs::PolynomialSegment4D > PolynomialSegment4DPtr;
typedef boost::shared_ptr< ::planning_msgs::PolynomialSegment4D const> PolynomialSegment4DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace planning_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'planning_msgs': ['/home/liu00222/catkin_ws/src/mav_comm/planning_msgs/msg'], 'mav_msgs': ['/home/liu00222/catkin_ws/src/mav_comm/mav_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c85fa40c94ff35d242df13d4d3a57809";
  }

  static const char* value(const ::planning_msgs::PolynomialSegment4D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc85fa40c94ff35d2ULL;
  static const uint64_t static_value2 = 0x42df13d4d3a57809ULL;
};

template<class ContainerAllocator>
struct DataType< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "planning_msgs/PolynomialSegment4D";
  }

  static const char* value(const ::planning_msgs::PolynomialSegment4D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
int32 num_coeffs        # order of the polynomial + 1, should match size of x[]\n\
duration segment_time   # duration of the segment\n\
float64[] x             # coefficients for the x-axis, INCREASING order\n\
float64[] y             # coefficients for the y-axis, INCREASING order\n\
float64[] z             # coefficients for the z-axis, INCREASING order\n\
float64[] yaw           # coefficients for the yaw,    INCREASING order\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::planning_msgs::PolynomialSegment4D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.num_coeffs);
      stream.next(m.segment_time);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PolynomialSegment4D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::planning_msgs::PolynomialSegment4D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::planning_msgs::PolynomialSegment4D_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "num_coeffs: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_coeffs);
    s << indent << "segment_time: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.segment_time);
    s << indent << "x[]" << std::endl;
    for (size_t i = 0; i < v.x.size(); ++i)
    {
      s << indent << "  x[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.x[i]);
    }
    s << indent << "y[]" << std::endl;
    for (size_t i = 0; i < v.y.size(); ++i)
    {
      s << indent << "  y[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.y[i]);
    }
    s << indent << "z[]" << std::endl;
    for (size_t i = 0; i < v.z.size(); ++i)
    {
      s << indent << "  z[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.z[i]);
    }
    s << indent << "yaw[]" << std::endl;
    for (size_t i = 0; i < v.yaw.size(); ++i)
    {
      s << indent << "  yaw[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.yaw[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLANNING_MSGS_MESSAGE_POLYNOMIALSEGMENT4D_H
