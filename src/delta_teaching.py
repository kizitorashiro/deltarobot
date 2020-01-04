import json

from delta_base import DeltaBase

class DeltaTeaching(DeltaBase):
    def __init__(self, config):
        super().__init__(config)
        self.history = []

    def add_history(self, servo_data):
        # サーボモータの角度情報を履歴として保存する
        history_data = [self.up_time]
        history_data = history_data + servo_data.tolist()
        if(len(self.history) <= 60000):
            self.history.append(history_data)
        else:
            print("history overflow")


    def save_history(self, filename):
        """
        サーボモータの角度情報の履歴をファイルに出力する
        """
        with open(filename, mode="w", encoding="utf-8") as f:
            json.dump(self.history, f)
