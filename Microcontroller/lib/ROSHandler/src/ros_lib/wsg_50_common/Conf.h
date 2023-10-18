#ifndef _ROS_SERVICE_Conf_h
#define _ROS_SERVICE_Conf_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wsg_50_common
{

static const char CONF[] = "wsg_50_common/Conf";

  class ConfRequest : public ros::Msg
  {
    public:
      typedef float _val_type;
      _val_type val;

    ConfRequest():
      val(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_val;
      u_val.real = this->val;
      *(outbuffer + offset + 0) = (u_val.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_val.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_val.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_val.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->val);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_val;
      u_val.base = 0;
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_val.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->val = u_val.real;
      offset += sizeof(this->val);
     return offset;
    }

    const char * getType(){ return CONF; };
    const char * getMD5(){ return "c9ee899b5f0899afa6060c9ba611652c"; };

  };

  class ConfResponse : public ros::Msg
  {
    public:
      typedef uint8_t _error_type;
      _error_type error;

    ConfResponse():
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

    const char * getType(){ return CONF; };
    const char * getMD5(){ return "bf8e3bc5443691736455ca47e3128027"; };

  };

  class Conf {
    public:
    typedef ConfRequest Request;
    typedef ConfResponse Response;
  };

}
#endif
