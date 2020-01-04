from logging import getLogger
import time
import numpy as np
import json

from delta_controller import DeltaController
from ax12_driver import AX12Driver
from ps4_controller import PS4Controller

class FileController(PS4Controller):

    def __init__(self, model, config, filename, logger=None):
        super().__init__(model, config, logger)
        self.current_data = [0, 512, 512, 512, 0] # 原点データ
        self.next_data = [0, 512, 512, 512, 0] # 原点データ
        self.play_start_time = None

        with open(filename, "r") as f:
            self.record_data = json.load(f)        

        self.record_type = self.record_data['type']
        self.history = self.record_data['data']

        self.playable_time = self.history[-1][0] + 1000

        if self.record_type == 'motor_angle':
            self.current_data = [0, 512, 512, 512, 0] # モーター原点データ
            self.next_data = [0, 512, 512, 512, 0] # モーター原点データ
        elif self.record_type == 'hand_position':
            origin_position = self.model.get_position()
            print("origin")
            print(origin_position)
            self.current_data = [0, origin_position[0], origin_position[1], origin_position[2], 0] # ハンド原点データ
            self.next_data = [0, origin_position[0], origin_position[1], origin_position[2], 0] # ハンド原点データ
        else:
            raise Exception

    def update(self):
        
        self.update_event()
        self.current_time = time.time() * 1000
        if self.play_start_time is None:
            self.play_start_time = self.current_time
        self.up_time = self.current_time - self.play_start_time

        while(True):
            # if(self.current_data[0] < self.up_time and self.up_time < self.next_data[0]):
            if(self.current_data[0] <= self.up_time and self.up_time < self.next_data[0]):
                break
            else:
                if(len(self.history) > 0):
                    self.current_data = self.next_data
                    self.next_data = self.history.pop(0)
                else:
                    self.current_data = self.next_data
                    break

        position = np.array([self.current_data[1], self.current_data[2], self.current_data[3]])
        print(self.current_data)
        if self.record_type == 'motor_angle':        
            thetas = AX12Driver.posi2theta(position)
            self.model.set_thetas(thetas)
        elif self.record_type == 'hand_position':
            self.model.set_position(position)

        # 再生時間を超過した場合
        if self.playable_time < self.up_time:
            self.end_flag = True
        
        return self.model

    def is_working(self):
        return not self.end_flag
