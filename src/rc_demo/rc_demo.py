#!/usr/bin/env python
import pygame
import sys
import math
# from dataclasses import dataclass

import rospy
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

white = (255, 255, 255)
red = (255, 50, 50)
green = (50, 255, 50)
blue = (50, 50, 255)
black = (0, 0, 0)
screen = None

# @dataclass 
class Stick:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y

        self.power = 5.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.updated_x = False
        self.updated_y = False
        self.radius = radius
        self.y_lock = False
        

    def release(self):
        if not self.updated_x:
            if abs(self.pos_x) < self.power:
                self.pos_x = 0
            elif self.pos_x > 0:
                self.pos_x -= self.power
            elif self.pos_x < 0:
                self.pos_x += self.power

        if not self.updated_y:
            if not self.y_lock or abs(self.pos_y) < self.power * 5:
                if abs(self.pos_y) < self.power:
                    self.pos_y = 0
                elif self.pos_y > 0:
                    self.pos_y -= self.power
                elif self.pos_y < 0:
                    self.pos_y += self.power

        self.updated_x = False
        self.updated_y = False
    
    def set_pose(self, x=float, y=float):
        dist = math.sqrt(x * x + y * y)

        if (dist <= self.radius):
            self.pos_x = x
            self.pos_y = y
        else:
            self.pos_x = x / math.sqrt(x * x + y * y) * self.radius
            self.pos_y = y / math.sqrt(x * x + y * y) * self.radius
            
        self.updated_x
        self.updated_y

    def get_center(self):
        return (self.center_x, self.center_y)

    def get_pixel(self):
        return (self.center_x + self.pos_x, self.center_y + self.pos_y)

class MAVROSCommander:
    def __init__(self):
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    def set_mode(self, mode):
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = mode
        resp1 = self.set_mode_client(0, offb_set_mode.custom_mode)
        return resp1.mode_sent

    def set_arm(self, value):
        arm_cmd = CommandBool()
        arm_cmd.value = value
        arm_client_1 = self.arming_client(arm_cmd.value)
        return arm_client_1.success

class Controller:
    def __init__(self):
        self.pub = rospy.Publisher('/rc_demo', String, queue_size=1)
        self.string_msg = String()
        self.pygame_img = None

        self.stick_1 = Stick(100,100,60)
        self.stick_2 = Stick(300+120,100,60)
        self.stick_1.y_lock = True
        self.is_lock = 0

        self.my_font = pygame.font.SysFont('Comic Sans MS', 30)
        self.text_surface_1 = self.my_font.render('Forward Lock', True, blue)
        self.text_surface_2 = self.my_font.render('Lock', True, blue)

        self.shoud_exit = False

        self.stick_1_pressed = False
        self.stick_2_pressed = False

        self.commander = MAVROSCommander()

    def get_diff(self, pos, center):
        return (pos[0] - center[0], pos[1] - center[1])

    def get_dist(self, pos, center):
        diff = self.get_diff(pos, center)
        return math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])
    
    def control_callback(self, event):
        key_event = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYUP:
                if key_event[pygame.K_TAB]:
                    self.commander.set_mode("OFFBOARD")
                    self.commander.set_arm(True)
                    print('set offboard & arm!')

        # mouse pressed event
        if pygame.mouse.get_pressed()[0]:
            mouse_pos = pygame.mouse.get_pos()

            if self.stick_1_pressed:
                diff = self.get_diff(mouse_pos, self.stick_1.get_center())
                self.stick_1.set_pose(diff[0], diff[1])

            elif self.stick_2_pressed:
                diff = self.get_diff(mouse_pos, self.stick_2.get_center())
                self.stick_2.set_pose(diff[0], diff[1])

            else:
                dist_1 = self.get_dist(mouse_pos, self.stick_1.get_center())
                dist_2 = self.get_dist(mouse_pos, self.stick_2.get_center())

                if dist_1 < self.stick_1.radius + 25:
                    self.stick_1_pressed = True
                elif dist_2 < self.stick_2.radius + 25:
                    self.stick_2_pressed = True
        else:
            self.stick_1_pressed = False
            self.stick_2_pressed = False

        # ESC
        if key_event[pygame.K_ESCAPE]:
            self.shoud_exit = True
            rospy.signal_shutdown('close')
            return

        screen.fill(black)

        if self.pygame_img != None:
            screen.blit(self.pygame_img, (0,0))

        pygame.draw.circle(screen, white, self.stick_1.get_center(), self.stick_1.radius, width=1)
        pygame.draw.circle(screen, white, self.stick_2.get_center(), self.stick_2.radius, width=1)
        pygame.draw.circle(screen, white, self.stick_1.get_pixel(), 15)
        pygame.draw.circle(screen, red, self.stick_2.get_pixel(), 15)

        self.stick_1.release()
        self.stick_2.release()

        pygame.display.update()

        # Publish
        if self.stick_2_pressed:
            roll = float(-self.stick_2.pos_x) / float(self.stick_2.radius)
            pitch = float(-self.stick_2.pos_y) / float(self.stick_2.radius)
        else: # skip motion
            roll = 0
            pitch = 0
            
        if self.stick_1_pressed:
            yaw = float(-self.stick_1.pos_x) / float(self.stick_1.radius)
            throttle = float(-self.stick_1.pos_y) / float(self.stick_1.radius)
        else: # skip motion
            yaw = 0
            throttle = float(-self.stick_1.pos_y) / float(self.stick_1.radius)

        self.string_msg.data = "{roll},{pitch},{yaw},{throttle}".format(roll=roll,pitch=pitch,yaw=yaw,throttle=throttle)
        self.pub.publish(self.string_msg)


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    SCREEN_WIDTH = 520
    SCREEN_HEIGHT = 200
    
    rospy.init_node("key_control_node", anonymous=True)

    pygame.init()
    pygame.display.set_caption("Simple PyGame Example")
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    pygame.font.init()

    controller = Controller()
    clock = pygame.time.Clock()
    while not controller.shoud_exit:
        clock.tick(60)
        controller.control_callback(1)
