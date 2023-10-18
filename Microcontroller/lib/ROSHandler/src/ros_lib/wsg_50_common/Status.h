#ifndef _ROS_wsg_50_common_Status_h
#define _ROS_wsg_50_common_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wsg_50_common
{

  class Status : public ros::Msg
  {
    public:
      typedef const char* _status_type;
      _status_type status;
      typedef float _width_type;
      _width_type width;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _acc_type;
      _acc_type acc;
      typedef float _force_type;
      _force_type force;
      typedef float _force_finger0_type;
      _force_finger0_type force_finger0;
      typedef float _force_finger1_type;
      _force_finger1_type force_finger1;

    Status():
      status(""),
      width(0),
      speed(0),
      acc(0),
      force(0),
      force_finger0(0),
      force_finger1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_status = strlen(this->status);
      varToArr(outbuffer + offset, length_status);
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
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
      union {
        float real;
        uint32_t base;
      } u_acc;
      u_acc.real = this->acc;
      *(outbuffer + offset + 0) = (u_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc);
      union {
        float real;
        uint32_t base;
      } u_force;
      u_force.real = this->force;
      *(outbuffer + offset + 0) = (u_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force);
      union {
        float real;
        uint32_t base;
      } u_force_finger0;
      u_force_finger0.real = this->force_finger0;
      *(outbuffer + offset + 0) = (u_force_finger0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force_finger0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force_finger0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force_finger0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force_finger0);
      union {
        float real;
        uint32_t base;
      } u_force_finger1;
      u_force_finger1.real = this->force_finger1;
      *(outbuffer + offset + 0) = (u_force_finger1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force_finger1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force_finger1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force_finger1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force_finger1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_status;
      arrToVar(length_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
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
      union {
        float real;
        uint32_t base;
      } u_acc;
      u_acc.base = 0;
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc = u_acc.real;
      offset += sizeof(this->acc);
      union {
        float real;
        uint32_t base;
      } u_force;
      u_force.base = 0;
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force = u_force.real;
      offset += sizeof(this->force);
      union {
        float real;
        uint32_t base;
      } u_force_finger0;
      u_force_finger0.base = 0;
      u_force_finger0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force_finger0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force_finger0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force_finger0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force_finger0 = u_force_finger0.real;
      offset += sizeof(this->force_finger0);
      union {
        float real;
        uint32_t base;
      } u_force_finger1;
      u_force_finger1.base = 0;
      u_force_finger1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force_finger1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force_finger1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force_finger1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force_finger1 = u_force_finger1.real;
      offset += sizeof(this->force_finger1);
     return offset;
    }

    const char * getType(){ return "wsg_50_common/Status"; };
    const char * getMD5(){ return "51c764be2e2c02863274a528dd8b3470"; };

  };

}
#endif