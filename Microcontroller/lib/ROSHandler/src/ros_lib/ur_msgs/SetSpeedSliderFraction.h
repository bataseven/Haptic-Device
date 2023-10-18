#ifndef _ROS_SERVICE_SetSpeedSliderFraction_h
#define _ROS_SERVICE_SetSpeedSliderFraction_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

static const char SETSPEEDSLIDERFRACTION[] = "ur_msgs/SetSpeedSliderFraction";

  class SetSpeedSliderFractionRequest : public ros::Msg
  {
    public:
      typedef double _speed_slider_fraction_type;
      _speed_slider_fraction_type speed_slider_fraction;

    SetSpeedSliderFractionRequest():
      speed_slider_fraction(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_speed_slider_fraction;
      u_speed_slider_fraction.real = this->speed_slider_fraction;
      *(outbuffer + offset + 0) = (u_speed_slider_fraction.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_slider_fraction.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_slider_fraction.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_slider_fraction.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_slider_fraction.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_slider_fraction.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_slider_fraction.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_slider_fraction.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_slider_fraction);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_speed_slider_fraction;
      u_speed_slider_fraction.base = 0;
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_slider_fraction.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_slider_fraction = u_speed_slider_fraction.real;
      offset += sizeof(this->speed_slider_fraction);
     return offset;
    }

    const char * getType(){ return SETSPEEDSLIDERFRACTION; };
    const char * getMD5(){ return "64134244ab4dfc72a3406fe06d580274"; };

  };

  class SetSpeedSliderFractionResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetSpeedSliderFractionResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETSPEEDSLIDERFRACTION; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetSpeedSliderFraction {
    public:
    typedef SetSpeedSliderFractionRequest Request;
    typedef SetSpeedSliderFractionResponse Response;
  };

}
#endif
