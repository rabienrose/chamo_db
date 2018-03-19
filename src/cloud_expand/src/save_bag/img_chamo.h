// Generated by gencpp from file save_bag/img_chamo.msg
// DO NOT EDIT!


#ifndef SAVE_BAG_MESSAGE_IMG_CHAMO_H
#define SAVE_BAG_MESSAGE_IMG_CHAMO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace save_bag
{
template <class ContainerAllocator>
struct img_chamo_
{
  typedef img_chamo_<ContainerAllocator> Type;

  img_chamo_()
    : frame_id(0)
    , absTimestamp(0.0)
    , jpg()  {
    }
  img_chamo_(const ContainerAllocator& _alloc)
    : frame_id(0)
    , absTimestamp(0.0)
    , jpg(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _frame_id_type;
  _frame_id_type frame_id;

   typedef double _absTimestamp_type;
  _absTimestamp_type absTimestamp;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _jpg_type;
  _jpg_type jpg;




  typedef boost::shared_ptr< ::save_bag::img_chamo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::save_bag::img_chamo_<ContainerAllocator> const> ConstPtr;

}; // struct img_chamo_

typedef ::save_bag::img_chamo_<std::allocator<void> > img_chamo;

typedef boost::shared_ptr< ::save_bag::img_chamo > img_chamoPtr;
typedef boost::shared_ptr< ::save_bag::img_chamo const> img_chamoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::save_bag::img_chamo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::save_bag::img_chamo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace save_bag

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'save_bag': ['/home/chamo/Documents/working/ros_ios_chamo/data_receiver/src/save_bag/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::save_bag::img_chamo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::save_bag::img_chamo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::save_bag::img_chamo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::save_bag::img_chamo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::save_bag::img_chamo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::save_bag::img_chamo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::save_bag::img_chamo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "79ef2031e1368e031a2f18950772d20a";
  }

  static const char* value(const ::save_bag::img_chamo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x79ef2031e1368e03ULL;
  static const uint64_t static_value2 = 0x1a2f18950772d20aULL;
};

template<class ContainerAllocator>
struct DataType< ::save_bag::img_chamo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "save_bag/img_chamo";
  }

  static const char* value(const ::save_bag::img_chamo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::save_bag::img_chamo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 frame_id\n\
float64 absTimestamp\n\
uint8[] jpg\n\
";
  }

  static const char* value(const ::save_bag::img_chamo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::save_bag::img_chamo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.frame_id);
      stream.next(m.absTimestamp);
      stream.next(m.jpg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct img_chamo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::save_bag::img_chamo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::save_bag::img_chamo_<ContainerAllocator>& v)
  {
    s << indent << "frame_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.frame_id);
    s << indent << "absTimestamp: ";
    Printer<double>::stream(s, indent + "  ", v.absTimestamp);
    s << indent << "jpg[]" << std::endl;
    for (size_t i = 0; i < v.jpg.size(); ++i)
    {
      s << indent << "  jpg[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.jpg[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAVE_BAG_MESSAGE_IMG_CHAMO_H
