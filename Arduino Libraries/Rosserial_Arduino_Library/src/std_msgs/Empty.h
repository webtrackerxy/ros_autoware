#ifndef _ROS_std_msgs_Empty_h
#define _ROS_std_msgs_Empty_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace std_msgs
{

  class Empty : public ros::Msg
  {
    public:

    Empty()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return PSTR( "std_msgs/Empty" ); };
    const char * getMD5(){ return PSTR( "d41d8cd98f00b204e9800998ecf8427e" ); };

  };

}
#endif