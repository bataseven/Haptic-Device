#ifndef _ROS_aruco_msgs_Marker_h
#define _ROS_aruco_msgs_Marker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"

namespace aruco_msgs
{

  class Marker : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _id_type;
      _id_type id;
      typedef geometry_msgs::PoseWithCovariance _pose_type;
      _pose_type pose;
      typedef double _confidence_type;
      _confidence_type confidence;

    Marker():
      header(),
      id(0),
      pose(),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_confidence.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_confidence.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_confidence.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_confidence.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
     return offset;
    }

    const char * getType(){ return "aruco_msgs/Marker"; };
    const char * getMD5(){ return "3f1f32688aa7271ee1d9fa3bf0c33422"; };

  };

}
#endif