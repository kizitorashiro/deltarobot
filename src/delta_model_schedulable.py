import numpy as np
import datetime
import time

from delta_model import DeltaModel

class DeltaModelSchedulable(DeltaModel):
    def __init__(self, config, logger=None):
        super().__init__(config['arm_length'], logger=logger)
        self.maxspeed = config['maxspeed']
        self.move_max = config['move_max']
        self.move_min = config['move_min']

        self.target_position = None
        self.speed_rate = None
        self.speed_vector = None
        self.prev_time = None
        self.current_time = None
        self.is_scheduled = False
        self.distance = None
        self.original_position = None
        
    def is_moving(self):
        diff = self.target_position - self.position
        # diff = np.abs(diff)
        if np.sum(diff) == 0:
            return False
        else:
            return True

    def schedule_position(self, position, speed_rate):
        if self.is_scheduled:
            return False
        else:
            self.is_scheduled = True
            self.target_position = position + 0
            self.speed_rate = speed_rate
            self.prev_time = time.time() * 1000
            self.current_time = None
            self.original_position = self.position + 0
            
            direction_vector = self.target_position - self.position
            self.logger.info("[sched_posi] target    {0}".format(self.target_position))
            self.logger.info("[sched_posi] current   {0}".format(self.position))
            self.logger.info("[sched_posi] direction {0}".format(direction_vector))
            norm = np.linalg.norm(direction_vector)
            self.distance = norm
            self.logger.info("[sched_posi] distance  {0}".format(self.distance))
            unit_vector = direction_vector / norm
            self.logger.info("[sched_posi] unit_vec  {0}".format(unit_vector))
            self.speed_vector = unit_vector * self.speed_rate * self.maxspeed
            self.logger.info("[sched_posi] speed_vec {0}".format(self.speed_vector))

            return True

    def set_position(self, position):
        """
        ハンドのポジションから逆運動学でモデルを更新する。
        """
        if self.is_scheduled:
            if self.is_moving():
                self.current_time = time.time() * 1000
                diff = self.current_time - self.prev_time
                self.prev_time = self.current_time
                
                # ハンド位置を移動速度と経過時間から計算
                tmp_position = self.position + self.speed_vector * diff/1000.0

                # 目標位置から行き過ぎていないかチェック
                diff_vector = tmp_position - self.original_position
                norm = np.linalg.norm(diff_vector)
                if norm > self.distance:
                    self.position = self.target_position + 0
                else:
                    self.position = tmp_position

                # 移動可能範囲のチェック
                self.position = np.maximum(self.position, self.move_min)
                self.position = np.minimum(self.position, self.move_max)

                self.thetas = self.solve_inverse_kinematics(self.position)
                return self.update_vectors()
            else:
                self.is_scheduled = False
                return self.A_vectors, self.B_vectors, self.C_vectors, self.D_vectors
        else:
            return super().set_position(position)
