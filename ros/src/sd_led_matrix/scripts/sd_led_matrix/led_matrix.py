#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Subscribes to robot eye messages and displays it

By default, when no message has to be displayed, 
robot eyes are animated with fun behaviours

"""

import rospy
import roslib
roslib.load_manifest('sd_led_matrix')
import os.path
from PIL import Image, ImageSequence
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT
from luma.core.sprite_system import framerate_regulator
from sd_led_matrix.msg import Eye
from std_msgs.msg import Bool
from luma.core.render import canvas

class RobotEye:
    def __init__(self, img_path):
        rospy.init_node("led_matrix")
        self.default_text = ""
        self.default_image = Image.open(os.path.join(img_path, "adam.gif"))
        self.default_fps = 24
        self.repeat = 0
        self.img_path = img_path
        ns = rospy.get_namespace()
        rospy.Subscriber(ns + "eye", Eye, self.execute_msg_cb)
        rospy.Subscriber(ns + "default_eye", Eye, self.execute_default_eye_cb)
        serial = spi(port=0, device=0, gpio=noop())
        self.device = max7219(serial, width=16, height=16)
        self.device.contrast(0x10)
        rospy.loginfo("Robot eye LCD Matrix ready")
        self.device.preprocess = self.preprocess

    def preprocess(self, image):
        """
        Make a -90° flip for each 8x8 LCD segment and a full rotate of the image
        This is done here as the luma_core has a bug when doing such composition
        """
        image = image.rotate(-180)
        image = image.copy()
        box1 = (8, 0, 16, 8)
        block1 = image.crop(box1)
        box2 = (0, 8, 8, 16)
        block2 = image.crop(box2)
        image.paste(block1, box2)
        image.paste(block2, box1)
        return image

    def execute_default_eye_cb(self, msg):
        self.default_text = msg.text
        img_path = os.path.join(self.img_path, self.default_text)
        if os.path.isfile(img_path):
            self.default_image = Image.open(img_path)
            self.default_text = ""

        self.default_fps = msg.fps or 24
        rospy.loginfo("Default Eye message received %s, %s, %d", self.default_text, self.default_image, self.default_fps)

    def execute_msg_cb(self, msg):
        # If text message is the name of an existing GIF then
        # display the animated GIF
        self.text = msg.text
        img_path = os.path.join(self.img_path, self.text)
        if os.path.isfile(img_path) :
            self.image = Image.open(img_path)
            self.text = ""

        self.repeat = msg.repeat or 1
        self.fps = msg.fps or 10
        rospy.loginfo("Eye message received %s, %d, %d", self.text, self.repeat, self.fps)

    def spin(self):
    
        while not rospy.is_shutdown():
            
            # A specific message should be displayed
            if self.repeat:
                text = self.text
                fps = self.fps
                if not text:
                    image = self.image
                self.repeat -= 1
            else:
                text = self.default_text
                fps = self.default_fps
                if not text:
                    image = self.default_image

            # Display text message
            if text:
                show_message(self.device, text, fill="white", font=proportional(CP437_FONT), scroll_delay=1/float(fps))    

            # Display animated GIF
            else:
                regulator = framerate_regulator(fps=fps)
                for frame in ImageSequence.Iterator(image):
                    with regulator:
                        b = frame.load()
                        #m =  [""] * 8
                        with canvas(self.device) as draw:
                            for i in range(16):
                                for j in range(16):
                                    if b[i,j]<0.5:
                                        draw.point((i, j), fill="black")
                                    else:
                                        draw.point((i, j), fill="white")


if __name__ == "__main__":
    # Local version
    img_path = os.path.join(os.path.dirname(__file__), '..', '..', 'images')
    
    # Distributed code version
    if not os.path.isdir(img_path):
        img_path = os.path.join(os.path.dirname(__file__), '..', '..', 'share', 'sd_led_matrix', 'images')
    
    robot_eye = RobotEye(img_path)
    robot_eye.spin()
