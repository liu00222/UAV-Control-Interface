// Generated by gencpp from file mav_msgs/DroneState.msg
// DO NOT EDIT!


#ifndef MAV_MSGS_MESSAGE_DRONESTATE_H
#define MAV_MSGS_MESSAGE_DRONESTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace mav_msgs
{
template <class ContainerAllocator>
struct DroneState_
{
  typedef DroneState_<ContainerAllocator> Type;

  DroneState_()
    : header()
    , position()
    , linear_velocity()
    , linear_acceleration()
    , orientation()
    , angular_velocity()
    , angular_acceleration()  {
    }
  DroneState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , linear_velocity(_alloc)
    , linear_acceleration(_alloc)
    , orientation(_alloc)
    , angular_velocity(_alloc)
    , angular_acceleration(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_velocity_type;
  _linear_velocity_type linear_velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_acceleration_type;
  _angular_acceleration_type angular_acceleration;





  typedef boost::shared_ptr< ::mav_msgs::DroneState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mav_msgs::DroneState_<ContainerAllocator> const> ConstPtr;

}; // struct DroneState_

typedef ::mav_msgs::DroneState_<std::allocator<void> > DroneState;

typedef boost::shared_ptr< ::mav_msgs::DroneState > DroneStatePtr;
typedef boost::shared_ptr< ::mav_msgs::DroneState const> DroneStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mav_msgs::DroneState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mav_msgs::DroneState_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::mav_msgs::DroneState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mav_msgs::DroneState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mav_msgs::DroneState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mav_msgs::DroneState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mav_msgs::DroneState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mav_msgs::DroneState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mav_msgs::DroneState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a32a14422af99ac7618c93ffa3457ceb";
  }

  static const char* value(const ::mav_msgs::DroneState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa32a14422af99ac7ULL;
  static const uint64_t static_value2 = 0x618c93ffa3457cebULL;
};

template<class ContainerAllocator>
struct DataType< ::mav_msgs::DroneState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mav_msgs/DroneState";
  }

  static const char* value(const ::mav_msgs::DroneState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mav_msgs::DroneState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
# This message defines the drone state.\n\
\n\
# We use the coordinate frames with the following convention:\n\
#   x: forward\n\
#   y: left\n\
#   z: up\n\
\n\
geometry_msgs/Vector3 position 					                        # Drone position [m].\n\
geometry_msgs/Vector3 linear_velocity                           # Drone linear velocity [m/s].\n\
geometry_msgs/Vector3 linear_acceleration                       # Drone linear acceleration [m/s^2].\n\
\n\
geometry_msgs/Quaternion orientation                            # Drone orientation expressed in quaternions.\n\
geometry_msgs/Vector3 angular_velocity                          # Drone angular velocity [rad/s].\n\
geometry_msgs/Vector3 angular_acceleration                      # Drone angular acceleration [rad/s^2].\n\
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
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::mav_msgs::DroneState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mav_msgs::DroneState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.linear_velocity);
      stream.next(m.linear_acceleration);
      stream.next(m.orientation);
      stream.next(m.angular_velocity);
      stream.next(m.angular_acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DroneState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mav_msgs::DroneState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mav_msgs::DroneState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "linear_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_velocity);
    s << indent << "linear_acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_acceleration);
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
    s << indent << "angular_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_velocity);
    s << indent << "angular_acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_acceleration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAV_MSGS_MESSAGE_DRONESTATE_H