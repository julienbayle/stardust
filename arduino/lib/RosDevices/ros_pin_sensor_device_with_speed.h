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

#ifndef _STARDUST_ROS_PIN_SENSOR_DEVICE_WITH_SPEED_h
#define _STARDUST_ROS_PIN_SENSOR_DEVICE_WITH_SPEED_h

#include "ros_device.h"

namespace stardust
{
   /**
   * This class offers a generic interface for ROS enabled sensors
   */
  template<class MsgT> class RosPinSensorDeviceWithSpeed : public RosDevice
  {
    public:
    
      RosPinSensorDeviceWithSpeed(ros::NodeHandle *nh, const char *topic_name, 
		                  const char *topic_speed_name, const byte pin) : 
        RosDevice(nh),
        pin_(pin), 
        publisher_(topic_name, &msg_),
        speed_publisher_(topic_speed_name, &speed_msg_) { 
      }
      
      virtual void setup() {
        nh_->advertise(publisher_);
        nh_->advertise(speed_publisher_);
        pinMode(pin_, INPUT);
      }

      virtual void publish() {
        publisher_.publish(&msg_);
        speed_publisher_.publish(&speed_msg_);
      }

    protected:
      
      // Sensor message
      MsgT msg_;
      MsgT speed_msg_;

      // Sensor pin
      const byte pin_;
      
      // Sensor ROS publisher
      ros::Publisher publisher_;
      ros::Publisher speed_publisher_;
  };
}
#endif
