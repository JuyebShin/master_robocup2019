// Generated by gencpp from file robocup2018_vision/robocupVision.msg
// DO NOT EDIT!


#ifndef ROBOCUP2018_VISION_MESSAGE_ROBOCUPVISION_H
#define ROBOCUP2018_VISION_MESSAGE_ROBOCUPVISION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robocup2018_vision
{
template <class ContainerAllocator>
struct robocupVision_
{
  typedef robocupVision_<ContainerAllocator> Type;

  robocupVision_()
    : Ballx(0)
    , Bally(0)
    , GoalPostBarx(0)
    , GoalPostBary(0)
    , GoalPostx(0)
    , GoalPosty(0)
    , GoalPostArea(0)
    , GoalPostVector(0)
    , GoalPostDist(0)
    , nCntXonLine(0)
    , Linex(0)
    , Liney(0)
    , VisionMode(0)  {
    }
  robocupVision_(const ContainerAllocator& _alloc)
    : Ballx(0)
    , Bally(0)
    , GoalPostBarx(0)
    , GoalPostBary(0)
    , GoalPostx(0)
    , GoalPosty(0)
    , GoalPostArea(0)
    , GoalPostVector(0)
    , GoalPostDist(0)
    , nCntXonLine(0)
    , Linex(0)
    , Liney(0)
    , VisionMode(0)  {
  (void)_alloc;
    }



   typedef int64_t _Ballx_type;
  _Ballx_type Ballx;

   typedef int64_t _Bally_type;
  _Bally_type Bally;

   typedef int64_t _GoalPostBarx_type;
  _GoalPostBarx_type GoalPostBarx;

   typedef int64_t _GoalPostBary_type;
  _GoalPostBary_type GoalPostBary;

   typedef int64_t _GoalPostx_type;
  _GoalPostx_type GoalPostx;

   typedef int64_t _GoalPosty_type;
  _GoalPosty_type GoalPosty;

   typedef int64_t _GoalPostArea_type;
  _GoalPostArea_type GoalPostArea;

   typedef int64_t _GoalPostVector_type;
  _GoalPostVector_type GoalPostVector;

   typedef int64_t _GoalPostDist_type;
  _GoalPostDist_type GoalPostDist;

   typedef int64_t _nCntXonLine_type;
  _nCntXonLine_type nCntXonLine;

   typedef int64_t _Linex_type;
  _Linex_type Linex;

   typedef int64_t _Liney_type;
  _Liney_type Liney;

   typedef int64_t _VisionMode_type;
  _VisionMode_type VisionMode;




  typedef boost::shared_ptr< ::robocup2018_vision::robocupVision_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robocup2018_vision::robocupVision_<ContainerAllocator> const> ConstPtr;

}; // struct robocupVision_

typedef ::robocup2018_vision::robocupVision_<std::allocator<void> > robocupVision;

typedef boost::shared_ptr< ::robocup2018_vision::robocupVision > robocupVisionPtr;
typedef boost::shared_ptr< ::robocup2018_vision::robocupVision const> robocupVisionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robocup2018_vision::robocupVision_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robocup2018_vision::robocupVision_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robocup2018_vision

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'robocup2018_vision': ['/home/robit/catkin_ws/src/robocup2018_vision/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robocup2018_vision::robocupVision_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robocup2018_vision::robocupVision_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robocup2018_vision::robocupVision_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a72ca8e9737f8ab60123c2500f28b2e4";
  }

  static const char* value(const ::robocup2018_vision::robocupVision_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa72ca8e9737f8ab6ULL;
  static const uint64_t static_value2 = 0x0123c2500f28b2e4ULL;
};

template<class ContainerAllocator>
struct DataType< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robocup2018_vision/robocupVision";
  }

  static const char* value(const ::robocup2018_vision::robocupVision_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 Ballx\n\
int64 Bally\n\
\n\
int64 GoalPostBarx\n\
int64 GoalPostBary\n\
\n\
int64 GoalPostx\n\
int64 GoalPosty\n\
\n\
int64 GoalPostArea\n\
\n\
int64 GoalPostVector\n\
int64 GoalPostDist\n\
\n\
int64 nCntXonLine\n\
\n\
int64 Linex\n\
int64 Liney\n\
\n\
int64 VisionMode\n\
";
  }

  static const char* value(const ::robocup2018_vision::robocupVision_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Ballx);
      stream.next(m.Bally);
      stream.next(m.GoalPostBarx);
      stream.next(m.GoalPostBary);
      stream.next(m.GoalPostx);
      stream.next(m.GoalPosty);
      stream.next(m.GoalPostArea);
      stream.next(m.GoalPostVector);
      stream.next(m.GoalPostDist);
      stream.next(m.nCntXonLine);
      stream.next(m.Linex);
      stream.next(m.Liney);
      stream.next(m.VisionMode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct robocupVision_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robocup2018_vision::robocupVision_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robocup2018_vision::robocupVision_<ContainerAllocator>& v)
  {
    s << indent << "Ballx: ";
    Printer<int64_t>::stream(s, indent + "  ", v.Ballx);
    s << indent << "Bally: ";
    Printer<int64_t>::stream(s, indent + "  ", v.Bally);
    s << indent << "GoalPostBarx: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPostBarx);
    s << indent << "GoalPostBary: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPostBary);
    s << indent << "GoalPostx: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPostx);
    s << indent << "GoalPosty: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPosty);
    s << indent << "GoalPostArea: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPostArea);
    s << indent << "GoalPostVector: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPostVector);
    s << indent << "GoalPostDist: ";
    Printer<int64_t>::stream(s, indent + "  ", v.GoalPostDist);
    s << indent << "nCntXonLine: ";
    Printer<int64_t>::stream(s, indent + "  ", v.nCntXonLine);
    s << indent << "Linex: ";
    Printer<int64_t>::stream(s, indent + "  ", v.Linex);
    s << indent << "Liney: ";
    Printer<int64_t>::stream(s, indent + "  ", v.Liney);
    s << indent << "VisionMode: ";
    Printer<int64_t>::stream(s, indent + "  ", v.VisionMode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOCUP2018_VISION_MESSAGE_ROBOCUPVISION_H
