
import time
import json
from logging import getLogger


class DeltaController:
    
    def __init__(self, model, config, logger=None):
        self.model = model
        self.config = config
        self.start_time = time.time() * 1000

        self.record_type = self.config['record_type']

        self.history = {
            'type': self.record_type,
            'data': []
        }

        self.logger = logger
        if self.logger is None:
            self.logger = getLogger(__name__)
        
    def update(self):
        pass

    def get_view_mode(self):
        pass

    def add_history(self, up_time, servo_data):
        # サーボモータの角度情報を履歴として保存する
        history_data = [up_time]
        history_data = history_data + servo_data.tolist()
        if(len(self.history) <= 60000):
            self.history['data'].append(history_data)
        else:
            print("history overflow")

    def save_history(self, filename):
        print("save_history")
        """
        サーボモータの角度情報の履歴をファイルに出力する
        """
        # print(self.history)
        with open(filename, mode="w", encoding="utf-8") as f:
            json.dump(self.history, f)
