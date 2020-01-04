#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import time
from enum import Enum

import sys
sys.path.append('../')
from dynamixel_sdk import *                    # Uses Dynamixel SDK library


# Control table address
ADDR_AX12_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX12_GOAL_POSITION      = 30
ADDR_AX12_PRESENT_POSITION   = 36
ADDR_AX12_PRESENT_LOAD       = 40
ADDR_AX12_MOVING_SPEED       = 32

# Data Byte Length
LEN_AX12_GOAL_POSITION       = 2
LEN_AX12_PRESENT_POSITION    = 2
LEN_AX12_PRESENT_LOAD        = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
# DEVICENAME                = '/dev/tty.usbserial-A5052NCY'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 200           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 250            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

class AX12State(Enum):
    DISCONNECT = 1
    CONNECT = 2
    TORQ = 3

class AX12Driver(object):

    def __init__(self, device_name, load_limit = 512):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_AX12_GOAL_POSITION, LEN_AX12_GOAL_POSITION)

        # 負荷の閾値。この値を超えた場合、トルクをOFFして動作を終了する
        self.load_limit = load_limit
    

    def connect(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def disconnect(self):
        # Close port
        self.portHandler.closePort()

    def enable_torq(self):
        for i in range(3):
            print(i)
            # Enable Dynamixel#1 Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_AX12_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                quit()
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                quit()
            else:
                print("Dynamixel#%d has been successfully connected" % i)


    def disable_torq(self):
        for i in range(3):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_AX12_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully disconnected" % i)

    def set_moving_speed(self, speed_value):
        for i in range(3):
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_AX12_MOVING_SPEED, speed_value)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully disconnected" % i)

    def goto_origin(self):
        current_pos = self.get_present_position()
        '''
        for i in range(3):
            if((current_pos[i] < (512-30)) or ((512+30) < current_pos[i]) ):
                print('ERROR')
                raise Exception('fail to goto origin')
        '''
        print('goto_origin')
        self.enable_torq()
        self.set_moving_speed(32)
        self.move_arms_by_position([512, 512, 512])
        self.set_moving_speed(0)

    def angle2posi(self, angle):
        ax12_angle = angle + 150.0
        posi = (ax12_angle / 300.0) * 1024
        return int(posi)

    @staticmethod
    def theta2posi(theta):
        posi = (theta / ((300.0/180.0) * np.pi)) * 1024
        return (posi + 512).astype(np.int)

    @staticmethod
    def posi2theta(position):
        position = position - 512
        angles = position * (300.0/1024.0)
        thetas = (angles / 180.0) * np.pi
        return thetas

    def get_current_thetas(self):
        position = self.get_present_position()
        return AX12Driver.posi2theta(np.array(position))

    def get_present_position(self):
        ''' 
        AX-12Aと通信して3つのモーターの現在位置情報を取得する
        '''
        return self.get_present_2byte_value(ADDR_AX12_PRESENT_POSITION)

    def get_present_load(self):
        '''
        AX-12Aと通信して3つのモータの現在負荷情報を取得する
        負荷が設定値を超える場合はトルクをOFFしてAX-12Aとの通信も解除する
        '''
        # is_overload = False
        load = np.array([0,0,0])
        reg_value = self.get_present_2byte_value(ADDR_AX12_PRESENT_LOAD)
        for i in range(3):
            reg_bytes = reg_value[i].to_bytes(2, 'big')
            direction = (reg_bytes[0] >> 2)
            if direction == 0:
                direction = -1
            load[i] = direction * (((reg_bytes[0] & 0b00000011) * 256) + reg_bytes[1])
        return load

    def get_present_2byte_value(self, addr):
        '''
        AX-12Aと通信してコントロールテーブルの値(2byte)を取得する
        '''
        position = [0,0,0]
        for i in range(3):
            # 通信速度計測
            # starttime = time.time() * 1000

            # シリアル通信
            # s = time.time() * 1000
            position[i], dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, i, addr)
            # e = time.time() * 1000
            # print(" " + str(e - s))
            # 通信速度計測
            # endtime = time.time() * 1000
            # print("  {0}".format(endtime - starttime))

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return position


    def move_arms_by_position(self, dxl_goal_position):
        for i in range(3):
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]))]

            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage        
            dxl_addparam_result = self.groupSyncWrite.addParam(i, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % i)
                quit()

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        load = np.array([0,0,0])
        while 1:
            
            dxl_present_position = self.get_present_position()
            
            # load_limitが設定されている場合は負荷情報を取得
            # load_limitを超過している場合はトルクをOFF、接続断
            
            if self.load_limit > 0:
                load = self.get_present_load()
                if np.max(np.abs(load)) > self.load_limit:
                    self.disable_torq()
                    self.disconnect()
                    print('load is too high {0}'.format(load))
                    print('disable torq and disconnect')
                    return False, dxl_present_position, load
            
            # print("[ID:0] GoalPos:%03d  PresPos:%03d\t[ID:1] GoalPos:%03d  PresPos:%03d\t[ID:2] GoalPos:%03d  PresPos:%03d" % (dxl_goal_position[0], dxl_present_position[0], dxl_goal_position[1], dxl_present_position[1], dxl_goal_position[2], dxl_present_position[2]))
            move_completed = True
            for i in range(3):
                if abs(dxl_goal_position[i] - dxl_present_position[i]) > DXL_MOVING_STATUS_THRESHOLD:
                    print('False %03d', dxl_present_position[i])
                    move_completed = False
            if move_completed:
                return True, dxl_present_position, load
 
    def move_arms(self, thetas):
        goal_position = AX12Driver.theta2posi(thetas)
        return self.move_arms_by_position(goal_position)
        
