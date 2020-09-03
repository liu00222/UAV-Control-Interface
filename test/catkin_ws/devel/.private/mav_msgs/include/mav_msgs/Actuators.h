// Generated by gencpp from file mav_msgs/Actuators.msg
// DO NOT EDIT!


#ifndef MAV_MSGS_MESSAGE_ACTUATORS_H
#define MAV_MSGS_MESSAGE_ACTUATORS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace mav_msgs
{
template <class ContainerAllocator>
struct Actuators_
{
  typedef Actuators_<ContainerAllocator> Type;

  Actuators_()
    : header()
    , angles()
    , angular_velocities()
    , normalized()  {
    }
  Actuators_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , angles(_alloc)
    , angular_velocities(_alloc)
    , normalized(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _angles_type;
  _angles_type angles;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _angular_velocities_type;
  _angular_velocities_type angular_velocities;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _normalized_type;
  _normalized_type normalized;





  typedef boost::shared_ptr< ::mav_msgs::Actuators_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mav_msgs::Actuators_<ContainerAllocator> const> ConstPtr;

}; // struct Actuators_

typedef ::mav_msgs::Actuators_<std::allocator<void> > Actuators;

typedef boost::shared_ptr< ::mav_msgs::Actuators > ActuatorsPtr;
typedef boost::shared_ptr< ::mav_msgs::Actuators const> ActuatorsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mav_msgs::Actuators_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mav_msgs::Actuators_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mav_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'mav_msgs': ['/home/liu00222/catkin_ws/src/mav_comm/mav_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mav_msgs::Actuators_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mav_msgs::Actuators_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mav_msgs::Actuators_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mav_msgs::Actuators_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mav_msgs::Actuators_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mav_msgs::Actuators_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mav_msgs::Actuators_<ContainerAllocator> >
{
  static const char* value()
  {
    return "25741daf38ed25442e3a66a855ee8d9c";
  }

  static const char* value(const ::mav_msgs::Actuators_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x25741daf38ed2544ULL;
  static const uint64_t static_value2 = 0x2e3a66a855ee8d9cULL;
};

template<class ContainerAllocator>
struct DataType< ::mav_msgs::Actuators_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mav_msgs/Actuators";
  }

  static const char* value(const ::mav_msgs::Actuators_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mav_msgs::Actuators_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
# This message defines lowest level commands to be sent to the actuator(s). \n\
\n\
float64[] angles             # Angle of the actuator in [rad]. \n\
                             # E.g. servo angle of a control surface(not angle of the surface!), orientation-angle of a thruster.      \n\
float64[] angular_velocities # Angular velocities of the actuator in [rad/s].\n\
                             # E.g. \"rpm\" of rotors, propellers, thrusters \n\
float64[] normalized         # Everything that does not fit the above, normalized between [-1 ... 1].\n\
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

  static const char* value(const ::mav_msgs::Actuators_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mav_msgs::Actuators_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.angles);
      stream.next(m.angular_velocities);
      stream.next(m.normalized);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Actuators_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mav_msgs::Actuators_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mav_msgs::Actuators_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "angles[]" << std::endl;
    for (size_t i = 0; i < v.angles.size(); ++i)
    {
      s << indent << "  angles[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angles[i]);
    }
    s << indent << "angular_velocities[]" << std::endl;
    for (size_t i = 0; i < v.angular_velocities.size(); ++i)
    {
      s << indent << "  angular_velocities[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angular_velocities[i]);
    }
    s << indent << "normalized[]" << std::endl;
    for (size_t i = 0; i < v.normalized.size(); ++i)
    {
      s << indent << "  normalized[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.normalized[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAV_MSGS_MESSAGE_ACTUATORS_H
