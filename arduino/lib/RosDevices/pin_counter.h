/**
 * Copyright © 2018 Julien BAYLE
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the “Software”), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _STARDUST_ENCODER_h
#define _STARDUST_ENCODER_h

#include "ros_pin_sensor_device.h"
#include <std_msgs/Int16.h>

namespace stardust
{
  /* Encoder counter (For ISR routine) */
  int encoder_counter [2] = {0, 0};

  /* Encoder interrrupt methods */
  void ISR_0() { encoder_counter[0]++; }
  void ISR_1() { encoder_counter[1]++; }

  /**
   * This class offers an interface to count pin TTL changes
   * 
   * Hardware must be UNO compatible
   * The pin must support change interrupt (pin 2 or pin 3 on Arduino UNO)
   */
  class PinCounter : public RosPinSensorDevice<std_msgs::Int16>
  {
    public:

      PinCounter(ros::NodeHandle *nh, const char *topic_name, const byte pin) : 
        RosPinSensorDevice<std_msgs::Int16>(nh, topic_name, pin) { }
      
      virtual void setup()
      {
        RosPinSensorDevice::setup();

        if(pin_ == 2)
          attachInterrupt(0, ISR_0, CHANGE);
        if(pin_ == 3)
          attachInterrupt(1, ISR_1, CHANGE);
      }

      virtual void update() {
        if(pin_ == 2 || pin_ == 3)
          msg_.data = encoder_counter[pin_ - 2];
        else
          msg_.data = -1;
      }
  };
}

#endif