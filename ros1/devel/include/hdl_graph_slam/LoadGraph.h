// Generated by gencpp from file hdl_graph_slam/LoadGraph.msg
// DO NOT EDIT!


#ifndef HDL_GRAPH_SLAM_MESSAGE_LOADGRAPH_H
#define HDL_GRAPH_SLAM_MESSAGE_LOADGRAPH_H

#include <ros/service_traits.h>


#include <hdl_graph_slam/LoadGraphRequest.h>
#include <hdl_graph_slam/LoadGraphResponse.h>


namespace hdl_graph_slam
{

struct LoadGraph
{

typedef LoadGraphRequest Request;
typedef LoadGraphResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct LoadGraph
} // namespace hdl_graph_slam


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hdl_graph_slam::LoadGraph > {
  static const char* value()
  {
    return "24518277da746ec8ade3d50c001c2adf";
  }

  static const char* value(const ::hdl_graph_slam::LoadGraph&) { return value(); }
};

template<>
struct DataType< ::hdl_graph_slam::LoadGraph > {
  static const char* value()
  {
    return "hdl_graph_slam/LoadGraph";
  }

  static const char* value(const ::hdl_graph_slam::LoadGraph&) { return value(); }
};


// service_traits::MD5Sum< ::hdl_graph_slam::LoadGraphRequest> should match
// service_traits::MD5Sum< ::hdl_graph_slam::LoadGraph >
template<>
struct MD5Sum< ::hdl_graph_slam::LoadGraphRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hdl_graph_slam::LoadGraph >::value();
  }
  static const char* value(const ::hdl_graph_slam::LoadGraphRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hdl_graph_slam::LoadGraphRequest> should match
// service_traits::DataType< ::hdl_graph_slam::LoadGraph >
template<>
struct DataType< ::hdl_graph_slam::LoadGraphRequest>
{
  static const char* value()
  {
    return DataType< ::hdl_graph_slam::LoadGraph >::value();
  }
  static const char* value(const ::hdl_graph_slam::LoadGraphRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hdl_graph_slam::LoadGraphResponse> should match
// service_traits::MD5Sum< ::hdl_graph_slam::LoadGraph >
template<>
struct MD5Sum< ::hdl_graph_slam::LoadGraphResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hdl_graph_slam::LoadGraph >::value();
  }
  static const char* value(const ::hdl_graph_slam::LoadGraphResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hdl_graph_slam::LoadGraphResponse> should match
// service_traits::DataType< ::hdl_graph_slam::LoadGraph >
template<>
struct DataType< ::hdl_graph_slam::LoadGraphResponse>
{
  static const char* value()
  {
    return DataType< ::hdl_graph_slam::LoadGraph >::value();
  }
  static const char* value(const ::hdl_graph_slam::LoadGraphResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HDL_GRAPH_SLAM_MESSAGE_LOADGRAPH_H