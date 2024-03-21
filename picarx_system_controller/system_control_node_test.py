#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sunfounder_controller import SunFounderController
from picarx import Picarx
from robot_hat import utils, Music
from vilib import Vilib
from time import sleep

current_line_state = None
last_line_state = "stop"
LINE_TRACK_SPEED = 10
LINE_TRACK_ANGLE_OFFSET = 20

AVOID_OBSTACLES_SPEED = 40
SafeDistance = 40       # > 40 safe
DangerDistance = 20     # > 20 && < 40 turn around, < 20 backward

DETECT_COLOR = 'red'    # red, green, blue, yellow , orange, purple

class PicarX_Controller(Node):
    def __init__(self):
        super().__init__('picarx_system_controller')
        # reset robot_hat
        utils.reset_mcu()
        sleep(0.2)
        
        self.get_logger().info('PicarX System Controller Node has been started')
        # init SunFounder Controller class
        self.sc = SunFounderController()
        self.sc.set_name('Picarx-001')
        self.sc.set_type('Picarx')
        self.sc.start()
        self.picarx = Picarx()
        self.speed = 0
        
        self.ip = utils.get_ip()
        self.get_logger().info(f'IP Address: {self.ip}')
        self.sc.set('video', 'http://'+self.ip+':9000/mjpg')
        
        # Initalize the video stream
        Vilib.camera_start(vflip=False, hflip=False)
        Vilib.display(local=False, web=True)
        
        # Create timer for callback
        self.create_timer(0, self.timer_callback)
        
    def timer_callback(self):
        # --- send data ---
        self.sc.set("A", self.speed)
        grayscale_data = self.picarx.get_grayscale_data()
        self.sc.set("D", grayscale_data)
        distance = self.picarx.get_distance()
        self.sc.set("F", distance)
        # --- control ---
        # line_track and avoid_obstacles
        line_track_switch = self.sc.get('I')
        avoid_obstacles_switch = self.sc.get('E')
        if line_track_switch == True:
            self.speed = LINE_TRACK_SPEED
            self.line_track()
        elif avoid_obstacles_switch == True:
            self.speed = AVOID_OBSTACLES_SPEED
            self.avoid_obstacles()
    
        # joystick moving
        if line_track_switch is not True and avoid_obstacles_switch is not True:
            Joystick_K_Val = self.sc.get('K')
            if Joystick_K_Val != None:
                dir_angle = utils.mapping(Joystick_K_Val[0], -100, 100, -30, 30)
                self.speed = Joystick_K_Val[1]
                self.picarx.set_dir_servo_angle(dir_angle)
                if self.speed > 0:
                    self.picarx.forward(self.speed)
                elif self.speed < 0:
                    self.speed = -self.speed
                    self.picarx.backward(self.speed)
                else:
                    self.picarx.stop()
        # camera servos control
        Joystick_Q_Val = self.sc.get('Q')
        if Joystick_Q_Val is not None:
            pan = min(90, max(-90, Joystick_Q_Val[0]))
            tilt = min(65, max(-35, Joystick_Q_Val[1]))
            self.picarx.set_cam_pan_angle(pan)
            self.picarx.set_cam_tilt_angle(tilt)
        # image recognition
        # zperacha: comment out vilib dependencies
    
        if self.sc.get('N') is True:
            Vilib.color_detect(DETECT_COLOR)
        else:
            Vilib.color_detect_switch(False)
        if self.sc.get('O') is True:
            Vilib.human_detect_switch(True)  
        else:
            Vilib.human_detect_switch(False)  
        if self.sc.get('P') is True:
            Vilib.object_detect_switch(True) 
        else:
            Vilib.object_detect_switch(False)
           
        # zperacha: video stream using opencv
      
    def avoid_obstacles(self):
        distance = self.picarx.get_distance()
        if distance >= SafeDistance:
            self.picarx.set_dir_servo_angle(0)
            self.picarx.forward(AVOID_OBSTACLES_SPEED)
        elif distance >= DangerDistance:
            self.picarx.set_dir_servo_angle(30)
            self.picarx.forward(AVOID_OBSTACLES_SPEED)
            sleep(0.1)
        else:
            self.picarx.set_dir_servo_angle(-30)
            self.picarx.backward(AVOID_OBSTACLES_SPEED)
            sleep(0.5) 
    
    def get_status(self, val_list):
        _state = self.picarx.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
        if _state == [0, 0, 0]:
            return 'stop'
        elif _state[1] == 1:
            return 'forward'
        elif _state[0] == 1:
            return 'right'
        elif _state[2] == 1:
            return 'left'
    
    def outHandle(self):
        global last_line_state, current_line_state
        if last_line_state == 'left':
            self.picarx.set_dir_servo_angle(-30)
            self.picarx.backward(10)
        elif last_line_state == 'right':
            self.picarx.set_dir_servo_angle(30)
            self.picarx.backward(10)
        while True:
            gm_val_list = self.picarx.get_grayscale_data()
            gm_state = self.get_status(gm_val_list)
            currentSta = gm_state
            if currentSta != last_line_state:
                break
        sleep(0.001)
        
    def line_track(self):
        global last_line_state
        gm_val_list = self.picarx.get_grayscale_data()
        gm_state = self.get_status(gm_val_list)
        if gm_state != "stop":
            last_line_state = gm_state
        if gm_state == 'forward':
            self.picarx.set_dir_servo_angle(0)
            self.picarx.forward(LINE_TRACK_SPEED) 
        elif gm_state == 'left':
            self.picarx.set_dir_servo_angle(LINE_TRACK_ANGLE_OFFSET)
            self.picarx.forward(LINE_TRACK_SPEED) 
        elif gm_state == 'right':
            self.picarx.set_dir_servo_angle(-LINE_TRACK_ANGLE_OFFSET)
            self.picarx.forward(LINE_TRACK_SPEED) 
        else:
            self.outHandle()
                
                              
def main(args=None):
    rclpy.init(args=args)
    picarx = PicarX_Controller()
    rclpy.spin(picarx)
    # picarx.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()