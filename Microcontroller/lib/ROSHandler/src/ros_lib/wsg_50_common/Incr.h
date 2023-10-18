#ifndef _ROS_SERVICE_Incr_h
#define _ROS_SERVICE_Incr_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wsg_50_common
{

static const char INCR[] = "wsg_50_common/Incr";

  class IncrRequest : public ros::Msg
  {
    public:
      typedef const char* _direction_type;
      _direction_type direction;
      typedef float _increment_type;
      _increment_type increment;

    IncrRequest():
      direction(""),
      increment(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_direction = strlen(this->direction);
      varToArr(outbuffer + offset, length_direction);
      offset += 4;
      memcpy(outbuffer + offset, this->direction, length_direction);
      offset += length_direction;
      union {
        float real;
        uint32_t base;
      } u_increment;
      u_increment.real = this->increment;
      *(outbuffer + offset + 0) = (u_increment.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_increment.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_increment.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_increment.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->increment);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_direction;
      arrToVar(length_direction, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_direction; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_direction-1]=0;
      this->direction = (char *)(inbuffer + offset-1);
      offset += length_direction;
      union {
        float real;
        uint32_t base;
      } u_increment;
      u_increment.base = 0;
      u_increment.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_increment.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_increment.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_increment.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->increment = u_increment.real;
      offset += sizeof(this->increment);
     return offset;
    }

    const char * getType(){ return INCR; };
    const char * getMD5(){ return "cca1e2c685538b4c18fbfec9ea6b6b36"; };

  };

  class IncrResponse : public ros::Msg
  {
    public:
      typedef uint8_t _error_type;
      _error_type error;

    IncrResponse():
      error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error);
     return offset;
    }

    const char * getType(){ return INCR; };
    const char * getMD5(){ return "bf8e3bc5443691736455ca47e3128027"; };

  };

  class Incr {
    public:
    typedef IncrRequest Request;
    typedef IncrResponse Response;
  };

}
#endif
