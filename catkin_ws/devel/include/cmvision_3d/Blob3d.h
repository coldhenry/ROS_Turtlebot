// Generated by gencpp from file cmvision_3d/Blob3d.msg
// DO NOT EDIT!


#ifndef CMVISION_3D_MESSAGE_BLOB3D_H
#define CMVISION_3D_MESSAGE_BLOB3D_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace cmvision_3d
{
template <class ContainerAllocator>
struct Blob3d_
{
  typedef Blob3d_<ContainerAllocator> Type;

  Blob3d_()
    : name()
    , red(0)
    , green(0)
    , blue(0)
    , area(0)
    , center()
    , top_left()
    , bottom_right()  {
    }
  Blob3d_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , red(0)
    , green(0)
    , blue(0)
    , area(0)
    , center(_alloc)
    , top_left(_alloc)
    , bottom_right(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef uint32_t _red_type;
  _red_type red;

   typedef uint32_t _green_type;
  _green_type green;

   typedef uint32_t _blue_type;
  _blue_type blue;

   typedef uint32_t _area_type;
  _area_type area;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _center_type;
  _center_type center;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _top_left_type;
  _top_left_type top_left;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _bottom_right_type;
  _bottom_right_type bottom_right;




  typedef boost::shared_ptr< ::cmvision_3d::Blob3d_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cmvision_3d::Blob3d_<ContainerAllocator> const> ConstPtr;

}; // struct Blob3d_

typedef ::cmvision_3d::Blob3d_<std::allocator<void> > Blob3d;

typedef boost::shared_ptr< ::cmvision_3d::Blob3d > Blob3dPtr;
typedef boost::shared_ptr< ::cmvision_3d::Blob3d const> Blob3dConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cmvision_3d::Blob3d_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cmvision_3d::Blob3d_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cmvision_3d

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'cmvision_3d': ['/home/turtlebot/catkin_ws_copy/src/cmvision_3d/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cmvision_3d::Blob3d_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cmvision_3d::Blob3d_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cmvision_3d::Blob3d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cmvision_3d::Blob3d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cmvision_3d::Blob3d_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cmvision_3d::Blob3d_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cmvision_3d::Blob3d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b7ac1acee51124a3194784be5dd98a9c";
  }

  static const char* value(const ::cmvision_3d::Blob3d_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb7ac1acee51124a3ULL;
  static const uint64_t static_value2 = 0x194784be5dd98a9cULL;
};

template<class ContainerAllocator>
struct DataType< ::cmvision_3d::Blob3d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cmvision_3d/Blob3d";
  }

  static const char* value(const ::cmvision_3d::Blob3d_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cmvision_3d::Blob3d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n\
uint32 red\n\
uint32 green\n\
uint32 blue\n\
uint32 area\n\
geometry_msgs/Point center\n\
geometry_msgs/Point top_left\n\
geometry_msgs/Point bottom_right\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::cmvision_3d::Blob3d_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cmvision_3d::Blob3d_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.red);
      stream.next(m.green);
      stream.next(m.blue);
      stream.next(m.area);
      stream.next(m.center);
      stream.next(m.top_left);
      stream.next(m.bottom_right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Blob3d_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cmvision_3d::Blob3d_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cmvision_3d::Blob3d_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "red: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.red);
    s << indent << "green: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.green);
    s << indent << "blue: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.blue);
    s << indent << "area: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.area);
    s << indent << "center: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.center);
    s << indent << "top_left: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.top_left);
    s << indent << "bottom_right: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.bottom_right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CMVISION_3D_MESSAGE_BLOB3D_H
