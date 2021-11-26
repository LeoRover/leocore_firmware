#ifndef _ROS_leo_msgs_WheelOdom_h
#define _ROS_leo_msgs_WheelOdom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace leo_msgs
{

  class WheelOdom : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _velocity_lin_type;
      _velocity_lin_type velocity_lin;
      typedef float _velocity_ang_type;
      _velocity_ang_type velocity_ang;
      typedef float _position_x_type;
      _position_x_type position_x;
      typedef float _position_y_type;
      _position_y_type position_y;
      typedef float _position_yaw_type;
      _position_yaw_type position_yaw;

    WheelOdom():
      stamp(),
      velocity_lin(0),
      velocity_ang(0),
      position_x(0),
      position_y(0),
      position_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        float real;
        uint32_t base;
      } u_velocity_lin;
      u_velocity_lin.real = this->velocity_lin;
      *(outbuffer + offset + 0) = (u_velocity_lin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_lin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_lin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_lin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_lin);
      union {
        float real;
        uint32_t base;
      } u_velocity_ang;
      u_velocity_ang.real = this->velocity_ang;
      *(outbuffer + offset + 0) = (u_velocity_ang.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_ang.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_ang.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_ang.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_ang);
      union {
        float real;
        uint32_t base;
      } u_position_x;
      u_position_x.real = this->position_x;
      *(outbuffer + offset + 0) = (u_position_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_x);
      union {
        float real;
        uint32_t base;
      } u_position_y;
      u_position_y.real = this->position_y;
      *(outbuffer + offset + 0) = (u_position_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_y);
      union {
        float real;
        uint32_t base;
      } u_position_yaw;
      u_position_yaw.real = this->position_yaw;
      *(outbuffer + offset + 0) = (u_position_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        float real;
        uint32_t base;
      } u_velocity_lin;
      u_velocity_lin.base = 0;
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_lin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_lin = u_velocity_lin.real;
      offset += sizeof(this->velocity_lin);
      union {
        float real;
        uint32_t base;
      } u_velocity_ang;
      u_velocity_ang.base = 0;
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_ang.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_ang = u_velocity_ang.real;
      offset += sizeof(this->velocity_ang);
      union {
        float real;
        uint32_t base;
      } u_position_x;
      u_position_x.base = 0;
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_x = u_position_x.real;
      offset += sizeof(this->position_x);
      union {
        float real;
        uint32_t base;
      } u_position_y;
      u_position_y.base = 0;
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_y = u_position_y.real;
      offset += sizeof(this->position_y);
      union {
        float real;
        uint32_t base;
      } u_position_yaw;
      u_position_yaw.base = 0;
      u_position_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_yaw = u_position_yaw.real;
      offset += sizeof(this->position_yaw);
     return offset;
    }

    virtual const char * getType() override { return "leo_msgs/WheelOdom"; };
    virtual const char * getMD5() override { return "f2e196e93e7dad2f6fa270317c31439e"; };

  };

}
#endif
