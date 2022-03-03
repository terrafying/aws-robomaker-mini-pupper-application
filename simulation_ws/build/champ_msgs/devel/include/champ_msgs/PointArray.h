// Generated by gencpp from file champ_msgs/PointArray.msg
// DO NOT EDIT!


#ifndef CHAMP_MSGS_MESSAGE_POINTARRAY_H
#define CHAMP_MSGS_MESSAGE_POINTARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <champ_msgs/Point.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/Point.h>

namespace champ_msgs
{
template <class ContainerAllocator>
struct PointArray_
{
  typedef PointArray_<ContainerAllocator> Type;

  PointArray_()
    : lf()
    , rf()
    , lh()
    , rh()  {
    }
  PointArray_(const ContainerAllocator& _alloc)
    : lf(_alloc)
    , rf(_alloc)
    , lh(_alloc)
    , rh(_alloc)  {
  (void)_alloc;
    }



   typedef  ::champ_msgs::Point_<ContainerAllocator>  _lf_type;
  _lf_type lf;

   typedef  ::champ_msgs::Point_<ContainerAllocator>  _rf_type;
  _rf_type rf;

   typedef  ::champ_msgs::Point_<ContainerAllocator>  _lh_type;
  _lh_type lh;

   typedef  ::champ_msgs::Point_<ContainerAllocator>  _rh_type;
  _rh_type rh;





  typedef boost::shared_ptr< ::champ_msgs::PointArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::champ_msgs::PointArray_<ContainerAllocator> const> ConstPtr;

}; // struct PointArray_

typedef ::champ_msgs::PointArray_<std::allocator<void> > PointArray;

typedef boost::shared_ptr< ::champ_msgs::PointArray > PointArrayPtr;
typedef boost::shared_ptr< ::champ_msgs::PointArray const> PointArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::champ_msgs::PointArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::champ_msgs::PointArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::champ_msgs::PointArray_<ContainerAllocator1> & lhs, const ::champ_msgs::PointArray_<ContainerAllocator2> & rhs)
{
  return lhs.lf == rhs.lf &&
    lhs.rf == rhs.rf &&
    lhs.lh == rhs.lh &&
    lhs.rh == rhs.rh;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::champ_msgs::PointArray_<ContainerAllocator1> & lhs, const ::champ_msgs::PointArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace champ_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::champ_msgs::PointArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::champ_msgs::PointArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::champ_msgs::PointArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::champ_msgs::PointArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::champ_msgs::PointArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::champ_msgs::PointArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::champ_msgs::PointArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e70791c6db4935709e33b9966d293c36";
  }

  static const char* value(const ::champ_msgs::PointArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe70791c6db493570ULL;
  static const uint64_t static_value2 = 0x9e33b9966d293c36ULL;
};

template<class ContainerAllocator>
struct DataType< ::champ_msgs::PointArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "champ_msgs/PointArray";
  }

  static const char* value(const ::champ_msgs::PointArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::champ_msgs::PointArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "champ_msgs/Point lf\n"
"champ_msgs/Point rf\n"
"champ_msgs/Point lh\n"
"champ_msgs/Point rh\n"
"================================================================================\n"
"MSG: champ_msgs/Point\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::champ_msgs::PointArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::champ_msgs::PointArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lf);
      stream.next(m.rf);
      stream.next(m.lh);
      stream.next(m.rh);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::champ_msgs::PointArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::champ_msgs::PointArray_<ContainerAllocator>& v)
  {
    s << indent << "lf: ";
    s << std::endl;
    Printer< ::champ_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.lf);
    s << indent << "rf: ";
    s << std::endl;
    Printer< ::champ_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.rf);
    s << indent << "lh: ";
    s << std::endl;
    Printer< ::champ_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.lh);
    s << indent << "rh: ";
    s << std::endl;
    Printer< ::champ_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.rh);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHAMP_MSGS_MESSAGE_POINTARRAY_H
