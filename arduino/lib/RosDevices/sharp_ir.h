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

#ifndef _STARDUST_SHARP_GP2Y0A51SK0F_h
#define _STARDUST_SHARP_GP2Y0A51SK0F_h

#include "ros_pin_sensor_device.h"
#include <sensor_msgs/Range.h>

namespace stardust
{
  /**
   * This class offers an interface for SHARP_GP2Y0A51SK0F (IR distance sensor)
   * The sensor must be connected to an ADC compatible pin
   */
  class SHARP_GP2Y0A51SK0F : public RosPinSensorDevice<sensor_msgs::Range>
  {
    public:

      SHARP_GP2Y0A51SK0F(ros::NodeHandle *nh, const char *topic_name, const byte pin):
          RosPinSensorDevice<sensor_msgs::Range>(nh, topic_name, pin) {
            msg_.radiation_type = sensor_msgs::Range::INFRARED;
            msg_.header.frame_id =  topic_name;
            msg_.field_of_view = 0.01; // rad (0.5 degree)
            msg_.min_range = 0.02;     // meter (2 cm)
            msg_.max_range = 0.15;     // meter (15 cm)
      }

      /*
       * Voltage at max distance is 0.4 V 
       * So if measure is below 0.4/(5/1024) = 82, 
       * we cannot determine the real distance.
       * In this case, max distance is returned
       *
       * Linearisation with https://mycurvefit.com/
       * Fit method : distance = a/(measure+b) 
       * 
       * Data :
       * 2.10 V => 430 => 0.02 m
       * 1.05 V => 215 => 0.05 m
       * 0.60V => 123 => 0.10 m
       * 0.40 V => 82 => 0.15 m 
       *
       * Result :
       * distance = 10.48688/(measure-12.80125)
       */
      virtual void update() {
        int measure = analogRead(pin_);
        if(measure < 82)
          msg_.range = 0.15;
        else
          msg_.range = 10.48688/(measure - 12.80125);

        msg_.header.stamp = ros::Time::now();
      }
  };
}
#endif
