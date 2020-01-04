import numpy as np
from logging import getLogger

class DeltaModel:
    """
    デルタロボットの3Dモデル
    """

    def __init__(self, arm_length, logger=None):
        """
        初期化。モデルの肩(A)、上腕(B)、前腕(C)、手の長さ(D)を設定する
        """
        self.A = arm_length['A']
        self.B = arm_length['B']
        self.C = arm_length['C']
        self.D = arm_length['D']

        angles = np.array([np.pi * (2.0/3.0) * i for i in range(3)])
        self.unit_vectors = np.array([np.cos(angles), np.sin(angles), np.zeros(3)]).T
        self.ez = np.array([0, 0, 1])
        self.A_vectors = self.A * self.unit_vectors
        self.B_vectors = None
        self.C_vectors = None
        self.D_vectors = None
        self.thetas = None
        self.position = None

        self.logger = logger
        if self.logger is None:
            self.logger = getLogger(__name__)
        
    def get_thetas(self):
        return self.thetas

    def get_position(self):
        return self.position

    def get_vectors(self):
        return self.A_vectors, self.B_vectors, self.C_vectors, self.D_vectors

    def solve_inverse_kinematics(self, position):
        """
        逆運動学の計算。手の中心座標(x,y,z)から肩の角度(theta)を計算する
        元ネタ
        https://tony-mooori.blogspot.com/2016/09/kinematics.html
        """
        # 逆運動学の式の可読性を上げるために分かりやすい変数に代入
        abs = np.abs
        cos = np.cos
        sin = np.sin
        pi = np.pi
        atan = np.arctan
        sqrt = np.sqrt
        A = self.A
        B = self.B
        C = self.C
        D = self.D
        x = position[0]
        y = position[1]
        z = position[2]

        # 逆運動学の計算
        thetas = np.zeros(3)
        
        angles = np.array([pi * (2.0/3.0) * i for i in range(3)])
        for i in range(3):
            phi_0 = angles[i]
            P = -A**2 + 2*A*D + 2*A*x*cos(phi_0) + 2*A*y*sin(phi_0) - B**2 + C**2 - D**2 - 2*D*x*cos(phi_0) - 2*D*y*sin(phi_0) - x**2 - y**2 - z**2
            Q = -2*B*z
            R = -2*A*B + 2*B*D + 2*B*x*cos(phi_0) + 2*B*y*sin(phi_0)
            theta_0 = -2*atan((Q - sqrt(-P**2 + Q**2 + R**2))/(P - R))
            theta_1 = -2*atan((Q + sqrt(-P**2 + Q**2 + R**2))/(P - R))
            thetas[i] = theta_1 if abs(theta_0) > abs(theta_1) else theta_0
        
        return thetas
    
    def solve_forward_kinematics(self, thetas):
        """
        順運動学の計算
        数式の元ネタ
        http://hypertriangle.com/~alex/delta-robot-tutorial/
        """
        
        # ここから ==========================================
        # ソースコードの元ネタ
        # https://github.com/awesomebytes/delta_robot/blob/master/src/delta_kinematics.py

        # 元ネタのソースはmathを使っているが、numpyを使いたいので別名定義
        math = np

        sqrt3 = math.sqrt(3.0)
        pi = 3.141592653
        tan60 = sqrt3
        sin30 = 0.5

        # 元ネタのモデルとは、胴体とハンドの長さの定義が異なるので変更
        # t = (f-e)*tan30/2
        t = self.A - self.D
        rf = self.B
        re = self.C

        # 元ネタのソースは引数の単位が度だが、ラジアンを直接渡すので変更
        # dtr = pi/180.0
        #theta1 *= dtr
        #theta2 *= dtr
        #theta3 *= dtr
        theta1 = thetas[0]
        theta2 = thetas[1]
        theta3 = thetas[2]

        y1 = -(t + rf*math.cos(theta1))
        z1 = -rf*math.sin(theta1)

        y2 = (t + rf*math.cos(theta2))*sin30
        x2 = y2*tan60
        z2 = -rf*math.sin(theta2)

    
        y3 = (t + rf*math.cos(theta3))*sin30
        x3 = -y3*tan60
        z3 = -rf*math.sin(theta3)
    
        dnm = (y2-y1)*x3-(y3-y1)*x2
    
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3
    
        # x = (a1*z + b1)/dnm
        a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
        b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0
    
        # y = (a2*z + b2)/dnm;
        a2 = -(z2-z1)*x3+(z3-z1)*x2
        b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0
    
        # a*z^2 + b*z + c = 0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
        c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)
    
        # discriminant
        d = b*b - 4.0*a*c
        if d < 0:
            return (-1, 0, 0 ,0) # non-existing point
    
        z0 = -0.5*(b+math.sqrt(d))/a
        x0 = (a1*z0 + b1)/dnm
        y0 = (a2*z0 + b2)/dnm

        # ここまで ==========================================

        # 元のモデルと軸設定が異なるので変換 (z軸を中心に90度回転)
        rotateMat = np.array([
            [0, -1],
            [1,  0],
        ])
        new_xy = np.dot(rotateMat, np.array([x0, y0]))
        return np.array([new_xy[0], new_xy[1], z0])

    def update_vectors(self):
        """
        モデルの肩(A)、上腕(B)、前腕(C)、手の長さ(D)の位置ベクトルを更新する
        位置ベクトルを計算する際のポイントはベクトルOAとベクトルDCが常に平行でるという制約条件
        """
        self.B_vectors = np.array([self.A_vectors[i] + self.B * (self.unit_vectors[i] * np.cos(self.thetas[i]) - self.ez * np.sin(self.thetas[i])) for i in range(3)])
        self.D_vectors = self.position + 0
        self.C_vectors = self.D_vectors + self.D * self.unit_vectors
        
        return self.A_vectors, self.B_vectors, self.C_vectors, self.D_vectors

    def set_position(self, position):
        """
        ハンドのポジションから逆運動学でモデルを更新する。
        """
        self.position = position + 0
        self.thetas = self.solve_inverse_kinematics(self.position)
        return self.update_vectors()
        
    def set_thetas(self, thetas):
        """
        アームの角度から順運動学でモデルを更新する。
        """
        self.thetas = thetas + 0
        self.position = self.solve_forward_kinematics(self.thetas)
        return self.update_vectors()

