"""
SocketCAN backend for DaMiao (DM) motor control.

Drop-in replacement for the serial (USB-CAN adapter) based MotorControl
in DM_CAN.py, using native Linux SocketCAN interfaces (e.g. can0).

Usage:
    from DM_CAN_SocketCAN import MotorControlSocketCAN
    from DM_CAN import Motor, DM_Motor_Type, Control_Type

    control = MotorControlSocketCAN("can0")
    motor = Motor(DM_Motor_Type.DM4310, SlaveID=0x01, MasterID=0x11)
    control.addMotor(motor)
    control.enable(motor)
    control.controlMIT(motor, kp=10, kd=0.5, q=0.0, dq=0.0, tau=0.0)
    print(motor.getPosition(), motor.getVelocity(), motor.getTorque())

Requires the CAN interface to be up, e.g.:
    sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
(DaMiao motors default to 1 Mbps.)
"""

import select
import socket
import struct
import sys
from time import sleep

import numpy as np

from DM_CAN import (
    Motor,
    DM_Motor_Type,
    DM_variable,
    Control_Type,
    float_to_uint,
    uint_to_float,
    float_to_uint8s,
    data_to_uint8s,
    is_in_ranges,
    uint8s_to_uint32,
    uint8s_to_float,
)

# struct can_frame { canid_t can_id; __u8 can_dlc; __u8 pad[3]; __u8 data[8]; }
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000


class MotorControlSocketCAN:
    #                4310           4310_48        4340           4340_48
    Limit_Param = [[12.5, 30, 10], [12.5, 50, 10], [12.5, 8, 28], [12.5, 10, 28],
                   # 6006           8006           8009            10010L         10010
                   [12.5, 45, 20], [12.5, 45, 40], [12.5, 45, 54], [12.5, 25, 200], [12.5, 20, 200],
                   # H3510            DMG62150      DMH6220         DM10422P
                   [12.5, 280, 1], [12.5, 45, 10], [12.5, 45, 10], [12.566, 20, 500]]

    def __init__(self, channel: str = "can0", dry_run: bool = False):
        """
        define MotorControl object 定义电机控制对象
        :param channel: SocketCAN interface name, e.g. "can0"
        :param dry_run: if True, do not send frames; print them to stderr instead
        """
        self.channel = channel
        self.dry_run = dry_run
        self.motors_map = dict()
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((channel,))
        self.sock.setblocking(False)
        print(f"SocketCAN interface {channel} is open", file=sys.stderr)

    def close(self):
        self.sock.close()

    # ------------------------------------------------------------------
    # low level CAN I/O

    def __send_data(self, can_id: int, data):
        """
        send one standard CAN frame 发送一帧标准CAN数据
        :param can_id: 11-bit CAN ID
        :param data: up to 8 bytes
        """
        data = bytes(bytearray(data))
        if self.dry_run:
            print(f"[dry-run] CAN TX id=0x{can_id & 0x7FF:03X} data={data.hex()}", file=sys.stderr)
            return
        frame = struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, len(data), data.ljust(8, b"\x00"))
        self.sock.send(frame)

    def __recv_frames(self, timeout: float = 0.0):
        """drain all pending CAN frames, waiting up to `timeout` for the first"""
        frames = []
        r, _, _ = select.select([self.sock], [], [], timeout)
        while r:
            try:
                raw = self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                break
            can_id, can_dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            if not (can_id & CAN_EFF_FLAG):  # standard frames only
                frames.append((can_id & 0x7FF, data[:can_dlc]))
            r, _, _ = select.select([self.sock], [], [], 0)
        return frames

    # ------------------------------------------------------------------
    # receive / decode (same logic as the serial version)

    def recv(self, timeout: float = 0.002):
        """receive motor feedback frames 接收电机反馈数据"""
        for can_id, data in self.__recv_frames(timeout):
            if len(data) >= 8:
                self.__process_packet(data, can_id)

    def recv_set_param_data(self, timeout: float = 0.002):
        for can_id, data in self.__recv_frames(timeout):
            if len(data) >= 8 and (data[2] == 0x33 or data[2] == 0x55):
                self.__process_set_param_packet(data, can_id)

    def __process_packet(self, data, CANID):
        if CANID != 0x00:
            if CANID in self.motors_map:
                self.__decode_feedback(self.motors_map[CANID], data)
        else:
            MasterID = data[0] & 0x0f
            if MasterID in self.motors_map:
                self.__decode_feedback(self.motors_map[MasterID], data)

    def __decode_feedback(self, motor, data):
        q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
        dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
        tau_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
        Q_MAX, DQ_MAX, TAU_MAX = self.Limit_Param[motor.MotorType]
        recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
        recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
        recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
        motor.recv_data(recv_q, recv_dq, recv_tau)

    def __process_set_param_packet(self, data, CANID):
        masterid = CANID
        slaveId = ((data[1] << 8) | data[0])
        if CANID == 0x00:  # 防止有人把MasterID设为0稳一手
            masterid = slaveId

        if masterid not in self.motors_map:
            if slaveId not in self.motors_map:
                return
            else:
                masterid = slaveId

        RID = data[3]
        if is_in_ranges(RID):
            num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
            self.motors_map[masterid].temp_param_dict[RID] = num
        else:
            num = uint8s_to_float(data[4], data[5], data[6], data[7])
            self.motors_map[masterid].temp_param_dict[RID] = num

    # ------------------------------------------------------------------
    # motor management

    def addMotor(self, Motor):
        """
        add motor to the motor control object 添加电机到电机控制对象
        :param Motor: Motor object 电机对象
        """
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor
        return True

    # ------------------------------------------------------------------
    # control modes

    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        MIT Control Mode Function 达妙电机MIT控制模式函数
        """
        if DM_Motor.SlaveID not in self.motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX, DQ_MAX, TAU_MAX = self.Limit_Param[MotorType]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00] * 8, np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xFF
        data_buf[1] = q_uint & 0xFF
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
        data_buf[4] = kp_uint & 0xFF
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
        data_buf[7] = tau_uint & 0xFF
        self.__send_data(DM_Motor.SlaveID, data_buf)
        self.recv()

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        """
        MIT Control Mode Function with delay 达妙电机MIT控制模式函数带延迟
        """
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        """
        control the motor in position and velocity control mode 电机位置速度控制模式
        """
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00] * 8, np.uint8)
        data_buf[0:4] = float_to_uint8s(P_desired)
        data_buf[4:8] = float_to_uint8s(V_desired)
        self.__send_data(motorid, data_buf)
        self.recv()

    def control_Vel(self, Motor, Vel_desired):
        """
        control the motor in velocity control mode 电机速度控制模式
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00] * 8, np.uint8)
        data_buf[0:4] = float_to_uint8s(Vel_desired)
        self.__send_data(motorid, data_buf)
        self.recv()

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
        """
        control the motor in force-position mixed mode 电机力位混合模式
        :param Pos_des: desired position rad  期望位置 单位为rad
        :param Vel_des: desired velocity 放大100倍
        :param i_des: desired current rang 0-10000 期望电流标幺值放大10000倍
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00] * 8, np.uint8)
        data_buf[0:4] = float_to_uint8s(Pos_des)
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xFF
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xFF
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()

    # ------------------------------------------------------------------
    # commands

    def __control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)

    def enable(self, Motor):
        """
        enable motor 使能电机
        最好在上电后几秒后再使能电机
        """
        self.__control_cmd(Motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()

    def enable_old(self, Motor, ControlMode):
        """
        enable motor old firmware 使能电机旧版本固件
        """
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        enable_id = ((int(ControlMode) - 1) << 2) + Motor.SlaveID
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()

    def disable(self, Motor):
        """
        disable motor 失能电机
        """
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.1)
        self.recv()

    def set_zero_position(self, Motor):
        """
        set the zero position of the motor 设置电机0位
        """
        self.__control_cmd(Motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()

    # ------------------------------------------------------------------
    # parameter read/write

    def __read_RID_param(self, Motor, RID):
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x33, np.uint8(RID),
                             0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, Motor, RID, data):
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x55, np.uint8(RID),
                             0x00, 0x00, 0x00, 0x00], np.uint8)
        if not is_in_ranges(RID):
            data_buf[4:8] = float_to_uint8s(data)
        else:
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        """
        switch the control mode of the motor 切换电机控制模式
        """
        max_retries = 20
        retry_interval = 0.1
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    if abs(self.motors_map[Motor.SlaveID].temp_param_dict[RID] - ControlMode) < 0.1:
                        return True
                    else:
                        return False
        return False

    def save_motor_param(self, Motor):
        """
        save the all parameter to flash 保存所有电机参数
        """
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xAA, 0x00,
                             0x00, 0x00, 0x00, 0x00], np.uint8)
        self.disable(Motor)  # before save disable the motor
        self.__send_data(0x7FF, data_buf)
        sleep(0.001)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        change the PMAX VMAX TMAX of the motor 改变电机的PMAX VMAX TMAX
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self, Motor):
        """
        get the motor status 获得电机状态
        """
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xCC, 0x00,
                             0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)
        self.recv()

    def change_motor_param(self, Motor, RID, data):
        """
        change the RID of the motor 改变电机的参数
        """
        max_retries = 20
        retry_interval = 0.05

        self.__write_motor_param(Motor, RID, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map and RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                if abs(self.motors_map[Motor.SlaveID].temp_param_dict[RID] - data) < 0.1:
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def read_motor_param(self, Motor, RID):
        """
        read only the RID of the motor 读取电机的内部信息例如 版本号等
        """
        max_retries = 20
        retry_interval = 0.05

        self.__read_RID_param(Motor, RID)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    return self.motors_map[Motor.SlaveID].temp_param_dict[RID]
        return None
