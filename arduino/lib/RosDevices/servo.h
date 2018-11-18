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

#ifndef _STARDUST_SERVO_h
#define _STARDUST_SERVO_h

#include "ros_device.h"
#include <std_msgs/UInt8.h>
#include <Servo.h> 

namespace stardust
{
  /**
   * This class offers an interface for servomotors
   * Parameter : Initial servo position (setup)
   * Input : UInt8 topic 
   * Ouput : PWM
   */
  class Servomotor : public RosDevice
  {
    public:

      Servomotor(
        ros::NodeHandle *nh, const char *topic_name,
        const byte pin, const byte initial_position):
          RosDevice(nh),
          pin_(pin),
	        initial_position_(initial_position),
          sub_position_(topic_name, &Servomotor::updatePWM, this ) 
      { }
    
      void setup()
      {
        nh->subscribe(sub_position_);
       
        servo_.attach(pin_);
        servo_.write(initial_position_);
      }

      void updatePWM(const std_msgs::UInt8& msg) {
        servo.write(msg.data);
      }

    private:
  
      const byte pin_;
      const byte initial_position_;
      Servo servo_;
      ros::Subscriber<std_msgs::UInt8, Servomotor> sub_position_;
  };
}
#endif
