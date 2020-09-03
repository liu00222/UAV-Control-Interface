// Generated by gencpp from file bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEFLYINGSTATECHANGED_H
#define BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEFLYINGSTATECHANGED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bebop_msgs
{
template <class ContainerAllocator>
struct Ardrone3PilotingStateFlyingStateChanged_
{
  typedef Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> Type;

  Ardrone3PilotingStateFlyingStateChanged_()
    : header()
    , state(0)  {
    }
  Ardrone3PilotingStateFlyingStateChanged_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , state(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _state_type;
  _state_type state;



  enum {
    state_landed = 0u,
    state_takingoff = 1u,
    state_hovering = 2u,
    state_flying = 3u,
    state_landing = 4u,
    state_emergency = 5u,
    state_usertakeoff = 6u,
    state_motor_ramping = 7u,
    state_emergency_landing = 8u,
  };


  typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> const> ConstPtr;

}; // struct Ardrone3PilotingStateFlyingStateChanged_

typedef ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<std::allocator<void> > Ardrone3PilotingStateFlyingStateChanged;

typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged > Ardrone3PilotingStateFlyingStateChangedPtr;
typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged const> Ardrone3PilotingStateFlyingStateChangedConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bebop_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'bebop_msgs': ['/home/liu00222/catkin_ws/src/bebop_autonomy/bebop_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f628b761a9125ace909b8b2c789eb09e";
  }

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf628b761a9125aceULL;
  static const uint64_t static_value2 = 0x909b8b2c789eb09eULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/Ardrone3PilotingStateFlyingStateChanged";
  }

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Ardrone3PilotingStateFlyingStateChanged\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Flying state.\n\
\n\
Header header\n\
\n\
# Drone flying state\n\
uint8 state_landed=0  # Landed state\n\
uint8 state_takingoff=1  # Taking off state\n\
uint8 state_hovering=2  # Hovering / Circling (for fixed wings) state\n\
uint8 state_flying=3  # Flying state\n\
uint8 state_landing=4  # Landing state\n\
uint8 state_emergency=5  # Emergency state\n\
uint8 state_usertakeoff=6  # User take off state. Waiting for user action to take off.\n\
uint8 state_motor_ramping=7  # Motor ramping state (for fixed wings).\n\
uint8 state_emergency_landing=8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.\n\
uint8 state\n\
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

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Ardrone3PilotingStateFlyingStateChanged_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::Ardrone3PilotingStateFlyingStateChanged_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEFLYINGSTATECHANGED_H
