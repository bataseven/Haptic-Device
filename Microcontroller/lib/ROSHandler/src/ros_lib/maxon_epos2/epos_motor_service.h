#ifndef _ROS_SERVICE_epos_motor_service_h
#define _ROS_SERVICE_epos_motor_service_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace maxon_epos2
{

static const char EPOS_MOTOR_SERVICE[] = "maxon_epos2/epos_motor_service";

  class epos_motor_serviceRequest : public ros::Msg
  {
    public:
      typedef float _position_setpoint_type;
      _position_setpoint_type position_setpoint;

    epos_motor_serviceRequest():
      position_setpoint(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_position_setpoint;
      u_position_setpoint.real = this->position_setpoint;
      *(outbuffer + offset + 0) = (u_position_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_position_setpoint;
      u_position_setpoint.base = 0;
      u_position_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_setpoint = u_position_setpoint.real;
      offset += sizeof(this->position_setpoint);
     return offset;
    }

    const char * getType(){ return EPOS_MOTOR_SERVICE; };
    const char * getMD5(){ return "9e94ec84848c71f46b874cbfbcd69fdf"; };

  };

  class epos_motor_serviceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef float _position_type;
      _position_type position;
      typedef float _velocity_type;
      _velocity_type velocity;

    epos_motor_serviceResponse():
      success(0),
      position(0),
      velocity(0)
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
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
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
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
     return offset;
    }

    const char * getType(){ return EPOS_MOTOR_SERVICE; };
    const char * getMD5(){ return "c255e7e841eac8ea6b6a532815001e2c"; };

  };

  class epos_motor_service {
    public:
    typedef epos_motor_serviceRequest Request;
    typedef epos_motor_serviceResponse Response;
  };

}
#endif
