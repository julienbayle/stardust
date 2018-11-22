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

#include <ros.h>
#include <hbridge.h>
#include <pin_counter.h>
#include <acs712.h>
#include <voltage.h>

/** Stardust - Arduino code - Robot 2 - UNO board */

// Pin definition
const byte RIGHT_ENCODER = 2;    // HALL SENSOR ENCODER
const byte LEFT_ENCODER  = 3;    // HALL SENSOR ENCODER

const byte RIGHT_PWM   = 6;      // MOTOR ( timer 0 / output 5 & 6)
const byte RIGHT_DIR   = 7;      // MOTOR
const byte RIGHT_BRAKE = 8;      // MOTOR

const byte LEFT_DIR    = 9;      // MOTOR
const byte LEFT_BRAKE  = 10;     // MOTOR
const byte LEFT_PWM    = 11;     // MOTOR ( timer 1 / output 11 & 12)

const byte CURRENT_12V = 0;      // ADC
const byte CURRENT_5V  = 1;      // ADC
const byte VOLTAGE_12V = 2;      // ADC
const byte VOLTAGE_5V  = 3;      // ADC

// Main loop speed (30Hz).
const int INTERVAL = 1000 / 30; 
unsigned long nextLoop = INTERVAL;

// ROS
ros::NodeHandle nh;

// ROS enabled devices
stardust::PinCounter pc_l(&nh, "/r2/left/encoder",  LEFT_ENCODER);
stardust::PinCounter pc_r(&nh, "/r2/right/encoder", RIGHT_ENCODER);
stardust::HBridge    hb_l(&nh, "/r2/left/pwm",  LEFT_PWM,  LEFT_DIR,  LEFT_BRAKE);
stardust::HBridge    hb_r(&nh, "/r2/right/pwm", RIGHT_PWM, RIGHT_DIR, RIGHT_BRAKE);
stardust::ACS712     i_12(&nh, "/r2/i_12v", CURRENT_12V);
stardust::ACS712     i_5 (&nh, "/r2/i_5v",  CURRENT_5V);
stardust::Voltage    u_12(&nh, "/r2/u_12v", VOLTAGE_12V, 5.54);
stardust::Voltage    u5  (&nh, "/r2/u_5v",  VOLTAGE_5V,  5.54);

const int NB_DEVICE = 8;
stardust::RosDevice *devices[NB_DEVICE] = {&pc_l, &pc_r, &hb_l, &hb_r, &i_12, &i_5, &u_12, &u5};

// Setup
void setup() {
  nh.initNode();
  for (int i=0 ; i<NB_DEVICE; i++)
    devices[i]->setup();
}

// Main loop
void loop() {

  if(millis() > nextLoop) {
    for (int i=0 ; i<NB_DEVICE; i++) {
      devices[i]->update();
      devices[i]->publish();
    }
    nextLoop += INTERVAL;
  }
  
  nh.spinOnce();
  delay(1);
}