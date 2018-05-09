#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Subscribes to robot eye messages and displays it

By default, when no message has to be displayed, 
robot eyes are animated with fun behaviours

"""

import rospy
import roslib
roslib.load_manifest('starbaby_led_matrix')
import os.path
from PIL import Image, ImageSequence
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT
from luma.core.sprite_system import framerate_regulator
from starbaby_led_matrix.msg import Eye
from std_msgs.msg import Bool
from luma.core.render import canvas

class RobotEye:
    def __init__(self, img_path):
        rospy.init_node("starbaby_led_matrix")
        self.default_text = "?"
        self.repeat=0
        self.img_path = img_path
        rospy.Subscriber("/starbaby/eye", Eye, self.execute_msg_cb)
        rospy.Subscriber("/is_orange",Bool, self.execute_side_cb)
        serial = spi(port=0, device=0, gpio=noop())
        self.device = max7219(serial, width=32, height=8)
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
        for x in range(0, 32, 8):
            box = (x, 0, x + 8, 8)
            rotated_block = image.crop(box).rotate(-90)
            image.paste(rotated_block, box)
        return image

    def execute_side_cb(self, msg):
        if msg.data:
            self.default_text = "Orange"
        else:
            self.default_text = "Vert"

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

                # Display text message
                if self.text:
                    show_message(self.device, self.text, fill="white", font=proportional(CP437_FONT), scroll_delay=1/float(self.fps))    

                # Display animated GIF
                else:
                    regulator = framerate_regulator(fps=self.fps)
                    for frame in ImageSequence.Iterator(self.image):
                        with regulator:
                            #background = Image.new("RGB", self.device.size, "white")
                            #background.paste(frame.resize((32,8), resample=Image.LANCZOS), (0,0))
                            #background.paste(frame)
                            b = frame.load()
                            #m =  [""] * 8
                            with canvas(self.device) as draw:
                                for i in range(32):
                                    for j in range(8):
                                        if b[i,j]<0.5:
                                            draw.point((i, j), fill="white")
                                        else:
                                            draw.point((i, j), fill="black")
                            #self.device.display(frame.convert(self.device.mode))
                            #self.device.display(background.convert(self.device.mode))
                
                self.repeat -= 1

            # Display default robot eye behaviour
            else:
                show_message(self.device, self.default_text, fill="white", font=proportional(CP437_FONT))


if __name__ == "__main__":
    img_path = os.path.join(os.path.dirname(__file__), '..', '..', 'images')
    robot_eye = RobotEye(img_path)
    robot_eye.spin()
