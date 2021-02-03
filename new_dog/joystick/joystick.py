#!/usr/bin/env python3
# -*-coding:utf-8-*-
import rospy
import pygame
import time
from sensor_msgs.msg import Joy
JOYSTICK_NAME ="Microsoft X-Box 360 pad"
GAIT_NUM = 1

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
size = [500, 700]
USE_SCREEN = 0
USE_PYGAME = 0
USE_ROSJOYSTICK = 1
if USE_SCREEN:
    screen = pygame.display.set_mode(size)
    clock = pygame.time.Clock()
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

class JoyStick():
    def __init__(self):
        rospy.init_node('Joystick_node', anonymous=True)
        if USE_SCREEN:
            pygame.init()
            self.textPrint = TextPrint()
        if USE_PYGAME:
            pygame.init()
            pygame.joystick.init()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        self.gait_num = 0

        self.angular_velocity = 0
        self.velocity = 0
        if USE_ROSJOYSTICK:
            self.Joysubscriber = rospy.Subscriber("/joy",Joy,self.joycallback)


    def main(self):
        while not rospy.is_shutdown():
            if USE_PYGAME:
                if self.joystick.get_name() == JOYSTICK_NAME:
                    pygame.event.get()
                    axis = [self.joystick.get_axis(0), self.joystick.get_axis(4)]
                    self.angular_velocity = -axis[0] * 0.4
                    self.velocity = axis[1] * 0.6
                    bottons = [self.joystick.get_button(0)]
                    if bottons[0] == 1:
                        self.gait_num += 1
                        if self.gait_num > GAIT_NUM:
                            self.gait_num = 0
                        time.sleep(0.3)
            rospy.set_param("current_gait",self.gait_num)
            rospy.set_param("command_vel",self.velocity)
            rospy.set_param("command_omega", self.angular_velocity)
            if USE_SCREEN:
                screen.fill(WHITE)
                self.textPrint.reset()
                self.textPrint.print(screen, "angular_velocity: {}".format(self.angular_velocity))
                self.textPrint.print(screen, "velocity: {}".format(self.velocity))
                self.textPrint.print(screen, "gait_num: {}".format(self.gait_num))
                # Go ahead and update the screen with what we've drawn.
                pygame.display.flip()
                # Limit to 20 frames per second
                clock.tick(20)
            rospy.Rate(20).sleep()

    def joycallback(self,msg):
        self.velocity = - msg.axes[4] * 0.6
        self.angular_velocity = msg.axes[0] * 0.4
        gait_button = msg.buttons[0]
        if gait_button == 1:
            self.gait_num += 1
            if self.gait_num > GAIT_NUM:
                self.gait_num = 0
            time.sleep(0.3)



dog_joystick = JoyStick()
dog_joystick.main()
