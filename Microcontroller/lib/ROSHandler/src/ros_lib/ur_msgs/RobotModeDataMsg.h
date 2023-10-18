#ifndef _ROS_ur_msgs_RobotModeDataMsg_h
#define _ROS_ur_msgs_RobotModeDataMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

  class RobotModeDataMsg : public ros::Msg
  {
    public:
      typedef uint64_t _timestamp_type;
      _timestamp_type timestamp;
      typedef bool _is_robot_connected_type;
      _is_robot_connected_type is_robot_connected;
      typedef bool _is_real_robot_enabled_type;
      _is_real_robot_enabled_type is_real_robot_enabled;
      typedef bool _is_power_on_robot_type;
      _is_power_on_robot_type is_power_on_robot;
      typedef bool _is_emergency_stopped_type;
      _is_emergency_stopped_type is_emergency_stopped;
      typedef bool _is_protective_stopped_type;
      _is_protective_stopped_type is_protective_stopped;
      typedef bool _is_program_running_type;
      _is_program_running_type is_program_running;
      typedef bool _is_program_paused_type;
      _is_program_paused_type is_program_paused;

    RobotModeDataMsg():
      timestamp(0),
      is_robot_connected(0),
      is_real_robot_enabled(0),
      is_power_on_robot(0),
      is_emergency_stopped(0),
      is_protective_stopped(0),
      is_program_running(0),
      is_program_paused(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timestamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->timestamp >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->timestamp >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->timestamp >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->timestamp >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timestamp);
      union {
        bool real;
        uint8_t base;
      } u_is_robot_connected;
      u_is_robot_connected.real = this->is_robot_connected;
      *(outbuffer + offset + 0) = (u_is_robot_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_robot_connected);
      union {
        bool real;
        uint8_t base;
      } u_is_real_robot_enabled;
      u_is_real_robot_enabled.real = this->is_real_robot_enabled;
      *(outbuffer + offset + 0) = (u_is_real_robot_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_real_robot_enabled);
      union {
        bool real;
        uint8_t base;
      } u_is_power_on_robot;
      u_is_power_on_robot.real = this->is_power_on_robot;
      *(outbuffer + offset + 0) = (u_is_power_on_robot.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_power_on_robot);
      union {
        bool real;
        uint8_t base;
      } u_is_emergency_stopped;
      u_is_emergency_stopped.real = this->is_emergency_stopped;
      *(outbuffer + offset + 0) = (u_is_emergency_stopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_emergency_stopped);
      union {
        bool real;
        uint8_t base;
      } u_is_protective_stopped;
      u_is_protective_stopped.real = this->is_protective_stopped;
      *(outbuffer + offset + 0) = (u_is_protective_stopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_protective_stopped);
      union {
        bool real;
        uint8_t base;
      } u_is_program_running;
      u_is_program_running.real = this->is_program_running;
      *(outbuffer + offset + 0) = (u_is_program_running.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_program_running);
      union {
        bool real;
        uint8_t base;
      } u_is_program_paused;
      u_is_program_paused.real = this->is_program_paused;
      *(outbuffer + offset + 0) = (u_is_program_paused.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_program_paused);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->timestamp =  ((uint64_t) (*(inbuffer + offset)));
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->timestamp |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->timestamp);
      union {
        bool real;
        uint8_t base;
      } u_is_robot_connected;
      u_is_robot_connected.base = 0;
      u_is_robot_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_robot_connected = u_is_robot_connected.real;
      offset += sizeof(this->is_robot_connected);
      union {
        bool real;
        uint8_t base;
      } u_is_real_robot_enabled;
      u_is_real_robot_enabled.base = 0;
      u_is_real_robot_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_real_robot_enabled = u_is_real_robot_enabled.real;
      offset += sizeof(this->is_real_robot_enabled);
      union {
        bool real;
        uint8_t base;
      } u_is_power_on_robot;
      u_is_power_on_robot.base = 0;
      u_is_power_on_robot.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_power_on_robot = u_is_power_on_robot.real;
      offset += sizeof(this->is_power_on_robot);
      union {
        bool real;
        uint8_t base;
      } u_is_emergency_stopped;
      u_is_emergency_stopped.base = 0;
      u_is_emergency_stopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_emergency_stopped = u_is_emergency_stopped.real;
      offset += sizeof(this->is_emergency_stopped);
      union {
        bool real;
        uint8_t base;
      } u_is_protective_stopped;
      u_is_protective_stopped.base = 0;
      u_is_protective_stopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_protective_stopped = u_is_protective_stopped.real;
      offset += sizeof(this->is_protective_stopped);
      union {
        bool real;
        uint8_t base;
      } u_is_program_running;
      u_is_program_running.base = 0;
      u_is_program_running.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_program_running = u_is_program_running.real;
      offset += sizeof(this->is_program_running);
      union {
        bool real;
        uint8_t base;
      } u_is_program_paused;
      u_is_program_paused.base = 0;
      u_is_program_paused.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_program_paused = u_is_program_paused.real;
      offset += sizeof(this->is_program_paused);
     return offset;
    }

    const char * getType(){ return "ur_msgs/RobotModeDataMsg"; };
    const char * getMD5(){ return "867308ca39e2cc0644b50db27deb661f"; };

  };

}
#endif