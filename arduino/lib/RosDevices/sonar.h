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

#ifndef _STARDUST_ULTRASOUND_h
#define _STARDUST_ULTRASOUND_h

#include "ros_pin_sensor_device.h"
#include <sensor_msgs/Range.h>

namespace stardust
{

  /* HC-SR04 timeout */
  const unsigned long MEASURE_TIMEOUT = 20000UL; // 20ms = ~6.8m at 340m/s

  /* Sound speed in m/us (two roud trip : 340 / 1e6 / 2) */
  const float SOUND_SPEED_2 = 0.00017;

  /**
   * This class offers an interface for HC SR04 sonar (Ultrasonic distance sensor)
   */
  class SonarHcSr04 : public RosPinSensorDevice<sensor_msgs::Range>
  {
    public:

      SonarHcSr04(ros::NodeHandle *nh, const char *topic_name, const byte echo_pin, const byte trigger_pin):
        RosPinSensorDevice<sensor_msgs::Range>(nh, topic_name, echo_pin),
        trigger_pin_(trigger_pin) {
          msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;
          msg_.header.frame_id =  topic_name;
          msg_.field_of_view = 0.5; // rad (30° degrees)
          msg_.min_range = 0.08;    // meter (8 cm)
          msg_.max_range = 0.60;    // meter
      }
    
      void setup(ros::NodeHandle *nh)
      {
        RosPinSensorDevice::setup();

        pinMode(trigger_pin_, OUTPUT);
        analogWrite(trigger_pin_, LOW);
      }

      void update() {
        
        /* Trigger an ultrasonic sound for 10us */
        digitalWrite(trigger_pin_, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin_, LOW);

        /* Mesure echo time */
        long measure = pulseIn(pin_, HIGH, MEASURE_TIMEOUT);

        /* Convert time to a distance */
        if (measure > 0)
          msg_.range = measure * SOUND_SPEED_2;
        else
          msg_.range = 10.0; // 10 meters (ROS will ignore it as it is greater than max range)

        msg_.header.stamp = ros::Time::now();
      }

      private:
    
        const byte trigger_pin_;
  };
}
#endif
