import pygame
import datetime
from logging import getLogger
import sys
import time
import numpy as np

from delta_controller import DeltaController
from ax12_driver import AX12Driver
from ps4_controller import PS4Controller

class DirectController(PS4Controller):

    def __init__(self, model, config, driver, logger=None):
        super().__init__(model, config, logger)
        self.driver = driver
        self.driver.disable_torq()
        
    def update(self):
        self.update_event()
        
        # サーボモータのポジション情報を得て履歴として保存する
        thetas = self.driver.get_current_thetas()
        servo_data = AX12Driver.theta2posi(thetas)

        # モデルを更新する
        self.model.set_thetas(thetas)

        up_time = (time.time() * 1000) - self.start_time
        if self.config['record_type'] == 'motor_angle':
            self.add_history(up_time, servo_data)
        elif self.config['record_type'] == 'hand_position':
            self.add_history(up_time, self.model.get_position())
        
        return self.model

