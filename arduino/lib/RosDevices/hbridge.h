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

#ifndef _STARDUST_HBRIDGE_h
#define _STARDUST_HBRIDGE_h

#include "ros_device.h"
#include <std_msgs/Int16.h>

namespace stardust
{
  /**
   * This class offers an interface for H Brige hardware like L298N
   * Input : Int16 topic 
   * Ouput : PWM and direction signal for an H Bridge 
   * 
   * PWM pin must support PWM
   */
  class HBridge : public RosDevice
  {
    public:

      HBridge(ros::NodeHandle *nh,
        const char *pwm_topic_name,
        const byte pwm_pin, 
        const byte dir_pin,
        const byte brake_pin):
          RosDevice(nh),
          pwm_pin_(pwm_pin),
          dir_pin_(dir_pin),
          brake_pin_(brake_pin),
          sub_pwm(pwm_topic_name, &HBridge::updatePWM, this ) 
      { }
    
      void setup()
      {
        nh_->subscribe(sub_pwm);

        pinMode(pwm_pin_, OUTPUT);
        analogWrite(pwm_pin_, 0);
  	
        pinMode(dir_pin_, OUTPUT);
        digitalWrite(dir_pin_, LOW);

        pinMode(brake_pin_, OUTPUT);
        digitalWrite(brake_pin_, LOW);
      }

      void updatePWM(const std_msgs::Int16& msg) {
        int pwm = msg.data;
        digitalWrite(dir_pin_, pwm > 0);
        digitalWrite(brake_pin_, pwm < 0);
        analogWrite(pwm_pin_, abs(pwm) > 255 ? 255 : abs(pwm));
      }

      private:

        const byte pwm_pin_;
        const byte dir_pin_;
        const byte brake_pin_;

        ros::Subscriber<std_msgs::Int16, HBridge> sub_pwm;
  };
}
#endif
