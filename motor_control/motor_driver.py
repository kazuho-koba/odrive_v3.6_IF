import odrive
from odrive.enums import *
import time


class MotorDriver:
    def __init__(self, axis_index=0):
        self.axis_index = axis_index

        # Odriveに接続
        print("Searching for ODrive...")
        self.odrive = odrive.find_any()
        if self.odrive is None:
            raise Exception("ODrive not found!")
        print("ODrive (axis" + str(axis_index) + ") connected!")

        # 制御する軸を選択
        if axis_index == 0:
            self.axis = self.odrive.axis0
        elif axis_index == 1:
            self.axis = self.odrive.axis1
        else:
            raise ValueError("Invalid axis_index! Use 0 or 1.")

        # ボードへの供給電圧を表示する
        print("the supplied voltage is:" +
              str(self.odrive.vbus_voltage) + "[V]")

    def start_closed_loop_control(self):
        # 閉ループ制御を開始する
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("start closed-loop control for axis" +
              str(self.axis_index))

    def set_velocity_control_mode(self):
        # 速度制御モードに設定
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        print("velocity control mode set.")

    def set_ramped_velocity_control_mode(self, ramp_rate=8.0):
        # 速度制御（速度変化平滑化あり）モードに設定
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.vel_ramp_rate = ramp_rate
        self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        print("ramped velocity control mode set.")

    def get_velocity(self):
        # 速度を取得する
        vel = self.axis.encoder.vel_estimate
        print(str(vel*60) + "[rpm]")

    def set_velocity(self, velocity):
        # 速度設定メソッド
        self.axis.controller.input_vel = velocity
        print(f"Velocity set to {velocity} round/sec.")
