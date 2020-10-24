// Generated by gencpp from file px_comm/SetCameraInfo.msg
// DO NOT EDIT!


#ifndef PX_COMM_MESSAGE_SETCAMERAINFO_H
#define PX_COMM_MESSAGE_SETCAMERAINFO_H

#include <ros/service_traits.h>


#include <px_comm/SetCameraInfoRequest.h>
#include <px_comm/SetCameraInfoResponse.h>


namespace px_comm
{

struct SetCameraInfo
{

typedef SetCameraInfoRequest Request;
typedef SetCameraInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCameraInfo
} // namespace px_comm


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::px_comm::SetCameraInfo > {
  static const char* value()
  {
    return "b7b33d05bd0d56b83943d2370771da4c";
  }

  static const char* value(const ::px_comm::SetCameraInfo&) { return value(); }
};

template<>
struct DataType< ::px_comm::SetCameraInfo > {
  static const char* value()
  {
    return "px_comm/SetCameraInfo";
  }

  static const char* value(const ::px_comm::SetCameraInfo&) { return value(); }
};


// service_traits::MD5Sum< ::px_comm::SetCameraInfoRequest> should match 
// service_traits::MD5Sum< ::px_comm::SetCameraInfo > 
template<>
struct MD5Sum< ::px_comm::SetCameraInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::px_comm::SetCameraInfo >::value();
  }
  static const char* value(const ::px_comm::SetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::px_comm::SetCameraInfoRequest> should match 
// service_traits::DataType< ::px_comm::SetCameraInfo > 
template<>
struct DataType< ::px_comm::SetCameraInfoRequest>
{
  static const char* value()
  {
    return DataType< ::px_comm::SetCameraInfo >::value();
  }
  static const char* value(const ::px_comm::SetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::px_comm::SetCameraInfoResponse> should match 
// service_traits::MD5Sum< ::px_comm::SetCameraInfo > 
template<>
struct MD5Sum< ::px_comm::SetCameraInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::px_comm::SetCameraInfo >::value();
  }
  static const char* value(const ::px_comm::SetCameraInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::px_comm::SetCameraInfoResponse> should match 
// service_traits::DataType< ::px_comm::SetCameraInfo > 
template<>
struct DataType< ::px_comm::SetCameraInfoResponse>
{
  static const char* value()
  {
    return DataType< ::px_comm::SetCameraInfo >::value();
  }
  static const char* value(const ::px_comm::SetCameraInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PX_COMM_MESSAGE_SETCAMERAINFO_H
