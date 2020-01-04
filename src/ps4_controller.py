import pygame
import datetime
from logging import getLogger
import sys
import time
import numpy as np

from delta_controller import DeltaController
from ax12_driver import AX12Driver

class PS4Controller(DeltaController):
    def __init__(self, model, config, logger=None):
        super().__init__(model, config, logger)
        self.maxspeed = self.config['maxspeed']
        self.move_max = np.array(config['move_max'])
        self.move_min = np.array(config['move_min'])

        self.controller = None
        self.axis_data = None
        self.button_data = None
        self.hat_data = None
        self.stick_vec = [0.0,0.0,0.0]
        self.end_flag = False
        self.current_time = None
        self.prev_time = None
        self.button4_on = False
        self.button5_on = False

        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        if not self.axis_data:
            self.axis_data = {}
            self.axis_data[0] = 0.0
            self.axis_data[1] = 0.0
            self.axis_data[2] = 0.0
            self.axis_data[3] = 0.0

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False
                
        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        self.logger = logger
        if self.logger is None:
            self.logger = getLogger(__name__)
        
    def update_event(self):
        
        # JOYSTICKの操作イベントを取得
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value,2)
                print(self.axis_data)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.button_data[event.button] = True
                if(event.button == 11 or event.button == 12):
                    self.end_flag = True
            elif event.type == pygame.JOYBUTTONUP:
                self.button_data[event.button] = False
                
            elif event.type == pygame.JOYHATMOTION:
                self.hat_data[event.hat] = event.value
        

    def update(self):
        
        self.update_event()                
        
        # 前回の処理実行時刻からの差分を計算
        if(self.current_time is None):
            self.current_time = time.time() * 1000
            return
        else:
            self.prev_time = self.current_time
            self.current_time = time.time() * 1000
        diff = self.current_time - self.prev_time
        up_time = self.current_time - self.start_time
        # print(str(diff))

        # PS4コントローラの操作値からx,y,z軸それぞれの速度情報を得る
        self.stick_vec = np.array([self.axis_data[0], -self.axis_data[1], -self.axis_data[3]])
        speed = self.stick_vec * self.maxspeed
        # Z軸方向はスピードをx0.6にする
        speed = speed * np.array([1.0, 1.0, 0.6])

        # 現在位置と速度ベクトルから目標位置を計算する
        goal_position = self.model.get_position() + speed * diff/1000.0

        # 移動可能範囲のチェック
        goal_position = np.maximum(goal_position, self.move_min)
        goal_position = np.minimum(goal_position, self.move_max)

        # モデルを更新する
        self.model.set_position(goal_position)

        
        if self.config['record_type'] == 'motor_angle':
            # ポジションの履歴を保存する
            # TODO ツールの状態も保存する
            thetas = self.model.get_thetas()
            servo_data = AX12Driver.theta2posi(thetas)
            self.add_history(up_time, servo_data)
        elif self.config['record_type'] == 'hand_position':
            self.add_history(up_time, self.model.get_position())
        
        return self.model

    def get_view_mode(self):
        view_mode = None
        if(self.button_data[1]):
            # default
            view_mode = "default" # {"azim": -60, "elev": 30}
        elif(self.button_data[0]):
            # x-z平面
            view_mode = "xz"
            # view_mode = {"azim": -90, "elev": 0}
        elif(self.button_data[3]):
            view_mode = "xy"
            # x-y平面
            #view_mode = {"azim": -90, "elev": 90}
        elif(self.button_data[2]):
            view_mode = "yz"
            # y-z平面
            #view_mode = {"azim": 0, "elev": 0}
        return view_mode

    def is_working(self):
        return not self.end_flag
