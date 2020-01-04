import threading

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

import datetime
import time
import sys
import json

from ps4_controller import PS4Controller
from ax12_driver import AX12Driver
from delta_model import DeltaModel
from delta_model_schedulable import DeltaModelSchedulable

class DeltaBase:
    def __init__(self, config):
        """
        各種パラメータをコンストラクの引数から取得して設定
        """
        # configパラメータから各種設定値を読み込み
        self.config = config
        #self.model = DeltaModel(config['arm_length'])
        self.model = DeltaModelSchedulable(config)
        self.move_max = np.array(config['move_max'])
        self.move_min = np.array(config['move_min'])
        self.model.set_thetas(np.array([0,0,0]))
        self.position = self.model.get_position()
        self.maxspeed = config['maxspeed']
        self.plot_range = config['plot_range']
        # self.interval = config['interval']
        self.tty = config['tty']
        self.load_limit = config['load_limit']

        # self.controller_type = config['controller_type']

        # その他のメンバ変数を初期化
        self.prev_time = None
        self.current_time = None
        self.driver = None
        self.controller = None
            
        self.view_mode = {"azim": -60, "elev": 30}

        self.lines = []
        
    def init_controller(self):
        """
        コントローラの初期化。サブクラスにて実装する。
        """
        pass

    def init_plot(self):
        """
        pyplotにモデルを描画し、モデルを構成する線分情報を取得する。
        本メソッドはモデルの描画開始する際に一度だけ実行する。以降はupdate_lines()で線分情報を更新していく
        """
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 逆運動学の計算
        A_vectors, B_vectors, C_vectors, D_vectors = self.model.set_position(self.position)
        
        # pyplot上に描画
        index = np.arange(4) % 3 # [0,1,2,0]
        self.ax.clear()
        # 胴体の三角形を構成する線分の描画
        self.ax.plot(A_vectors[index,0], A_vectors[index,1], A_vectors[index,2])
        # ハンドの三角形を構成する線分の描画
        self.ax.plot(C_vectors[index,0], C_vectors[index,1], C_vectors[index,2])
        # ３つのアームを構成する線分の描画
        for i in range(3):
            self.ax.plot(
                [A_vectors[i,0], B_vectors[i,0], C_vectors[i,0]],
                [A_vectors[i,1], B_vectors[i,1], C_vectors[i,1]],
                [A_vectors[i,2], B_vectors[i,2], C_vectors[i,2]]
            )
        self.lines = self.ax.get_lines()

        ax_pos = self.ax.get_position()

        self.A_vectors = A_vectors
        self.B_vectors = B_vectors
        self.C_vectors = C_vectors
        self.D_vectors = D_vectors

        self.ax.set_xlim3d(self.plot_range["x"])
        self.ax.set_ylim3d(self.plot_range["y"])
        self.ax.set_zlim3d(self.plot_range["z"])
        
        return self.lines            


    def update_plot(self,i):
        """
        pyplot上でデルタロボットを表現する線分情報(lines)を更新し、描画に反映する
        """
        A_vectors = self.A_vectors
        B_vectors = self.B_vectors
        C_vectors = self.C_vectors
        D_vectors = self.D_vectors

        next_view_mode = self.controller.get_view_mode()
        if next_view_mode is not None:
            if next_view_mode == "default":
                # default
                self.view_mode = {"azim": -60, "elev": 30}
            elif next_view_mode == "xz":
                # x-z平面
                self.view_mode = {"azim": -90, "elev": 0}
            elif next_view_mode == "xy":
                # x-y平面
                self.view_mode = {"azim": -90, "elev": 90}
            elif next_view_mode == "yz":
                # y-z平面
                self.view_mode = {"azim": 0, "elev": 0}

        self.ax.view_init(**self.view_mode)
                
        index = np.arange(4) % 3 # [0,1,2,0]

        # 胴体の三角形を構成する線分
        # x,y座標
        self.lines[0].set_data(A_vectors[index,0], A_vectors[index,1])
        # z座標
        self.lines[0].set_3d_properties(A_vectors[index,2])

        # ハンドの三角形を構成する線分
        # x,y座標
        self.lines[1].set_data(C_vectors[index,0], C_vectors[index,1])
        # z座標
        self.lines[1].set_3d_properties(C_vectors[index,2])

        # ３つのアームを構成する線分
        for i in range(3):
            # x,y座標
            self.lines[i+2].set_data(
                [A_vectors[i,0],B_vectors[i,0],C_vectors[i,0]],
                [A_vectors[i,1],B_vectors[i,1],C_vectors[i,1]])
            # z座標
            self.lines[i+2].set_3d_properties([A_vectors[i,2],B_vectors[i,2],C_vectors[i,2]])
        
        ax_pos = self.ax.get_position()
        
        self.fig.canvas.draw()
        
        return self.lines
        
    def actuate(self):
        raise NotImplementedError()

    def update(self, i):
        """
        制御ループ内で実行される処理
        """

        # 制御ループの実行時間を計測
        diff_time = 0
        if(self.current_time is None):
            self.current_time = time.time() * 1000
            diff_time = 0
        else:
            self.prev_time = self.current_time
            self.current_time = time.time() * 1000
            diff_time = self.current_time - self.prev_time

        # for debug
        print("{0}".format(diff_time))

        self.controller.update()
        if self.controller.is_working() == False:
            return False
        
        self.A_vectors, self.B_vectors, self.C_vectors, self.D_vectors = self.model.get_vectors()

        result, position, load = self.actuate()

        # viewにpyplotが指定されている場合はグラフを描画
        if self.config['view'] == 'pyplot':
            self.update_plot(0)

        return result
    
    def init_driver(self):
        """
        dynamixelモーターの初期化
        """
        print("init_driver")
        self.driver = AX12Driver(self.tty, self.load_limit)
        self.driver.connect()
        self.driver.goto_origin()
        print("enable_torq")
                
    def start(self):
        """
        制御開始
        """
        if(self.tty is not None):
            print("init_driver start")
            self.init_driver()

        # コントローラの初期化
        self.init_controller()

        # pyplotの描画領域の初期化
        if self.config['view'] == 'pyplot':
            self.init_plot()
            plt.show(block=False)

        # メインループ
        while self.update(0):
            pass

    def up(self):
        while True:
            self.update(0)

    def end(self):
        """
        制御終了
        """
        try:
            if(self.driver is not None):
                self.driver.disable_torq()
                self.driver.disconnect()
        except:
            pass
    
    def save_history(self, filename):
        print("base save")
        self.controller.save_history(filename)
    
