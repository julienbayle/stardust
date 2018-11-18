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

#ifndef _STARDUST_ASC712_h
#define _STARDUST_ASC712_h

#include "ros_pin_sensor_device.h"
#include <std_msgs/Float32.h>

namespace stardust
{
  /**
   * This class offers an interface for ACS 712 (Current to voltage sensor)
   * The sensor must be connected to an ADC compatible pin
   */
  class ACS712 : public RosPinSensorDevice<std_msgs::Float32>
  {
    public:
    
      ACS712(ros::NodeHandle *nh, const char *topic_name, const byte pin) : 
        RosPinSensorDevice<std_msgs::Float32>(nh, topic_name, pin) { }
      
      virtual void update() {
        // Current in mA
        // ACS712 5A specs : 512=>0mA, 1024=>5000mA, 0=>-5000mA
        // Resolution : 9,77 mA for 1 bit
        msg_.data = 0;

        // Average the value on 4 measures
        for(int i=0;i<4;i++) {
          msg_.data += (float) (analogRead(pin_) - 512);
        } 
        msg_.data *= 2.44;
      }
  };
}
#endif
