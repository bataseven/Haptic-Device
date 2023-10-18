#ifndef _ROS_wsg_50_common_Cmd_h
#define _ROS_wsg_50_common_Cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wsg_50_common
{

  class Cmd : public ros::Msg
  {
    public:
      typedef const char* _mode_type;
      _mode_type mode;
      typedef float _pos_type;
      _pos_type pos;
      typedef float _speed_type;
      _speed_type speed;

    Cmd():
      mode(""),
      pos(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      union {
        float real;
        uint32_t base;
      } u_pos;
      u_pos.real = this->pos;
      *(outbuffer + offset + 0) = (u_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      union {
        float real;
        uint32_t base;
      } u_pos;
      u_pos.base = 0;
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos = u_pos.real;
      offset += sizeof(this->pos);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
     return offset;
    }

    const char * getType(){ return "wsg_50_common/Cmd"; };
    const char * getMD5(){ return "a9d4654d92a2e086717420189f98c76a"; };

  };

}
#endif