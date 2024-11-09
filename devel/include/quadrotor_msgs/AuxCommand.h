// Generated by gencpp from file quadrotor_msgs/AuxCommand.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_AUXCOMMAND_H
#define QUADROTOR_MSGS_MESSAGE_AUXCOMMAND_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct AuxCommand_
{
  typedef AuxCommand_<ContainerAllocator> Type;

  AuxCommand_()
    : current_yaw(0.0)
    , kf_correction(0.0)
    , angle_corrections()
    , enable_motors(false)
    , use_external_yaw(false)  {
      angle_corrections.assign(0.0);
  }
  AuxCommand_(const ContainerAllocator& _alloc)
    : current_yaw(0.0)
    , kf_correction(0.0)
    , angle_corrections()
    , enable_motors(false)
    , use_external_yaw(false)  {
  (void)_alloc;
      angle_corrections.assign(0.0);
  }



   typedef double _current_yaw_type;
  _current_yaw_type current_yaw;

   typedef double _kf_correction_type;
  _kf_correction_type kf_correction;

   typedef boost::array<double, 2>  _angle_corrections_type;
  _angle_corrections_type angle_corrections;

   typedef uint8_t _enable_motors_type;
  _enable_motors_type enable_motors;

   typedef uint8_t _use_external_yaw_type;
  _use_external_yaw_type use_external_yaw;





  typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const> ConstPtr;

}; // struct AuxCommand_

typedef ::quadrotor_msgs::AuxCommand_<std::allocator<void> > AuxCommand;

typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand > AuxCommandPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::AuxCommand const> AuxCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::AuxCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::AuxCommand_<ContainerAllocator2> & rhs)
{
  return lhs.current_yaw == rhs.current_yaw &&
    lhs.kf_correction == rhs.kf_correction &&
    lhs.angle_corrections == rhs.angle_corrections &&
    lhs.enable_motors == rhs.enable_motors &&
    lhs.use_external_yaw == rhs.use_external_yaw;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::AuxCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "94f75840e4b1e03675da764692f2c839";
  }

  static const char* value(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x94f75840e4b1e036ULL;
  static const uint64_t static_value2 = 0x75da764692f2c839ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/AuxCommand";
  }

  static const char* value(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 current_yaw\n"
"float64 kf_correction\n"
"float64[2] angle_corrections# Trims for roll, pitch\n"
"bool enable_motors\n"
"bool use_external_yaw\n"
;
  }

  static const char* value(const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_yaw);
      stream.next(m.kf_correction);
      stream.next(m.angle_corrections);
      stream.next(m.enable_motors);
      stream.next(m.use_external_yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AuxCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::AuxCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::AuxCommand_<ContainerAllocator>& v)
  {
    s << indent << "current_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.current_yaw);
    s << indent << "kf_correction: ";
    Printer<double>::stream(s, indent + "  ", v.kf_correction);
    s << indent << "angle_corrections[]" << std::endl;
    for (size_t i = 0; i < v.angle_corrections.size(); ++i)
    {
      s << indent << "  angle_corrections[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angle_corrections[i]);
    }
    s << indent << "enable_motors: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enable_motors);
    s << indent << "use_external_yaw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_external_yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_AUXCOMMAND_H
