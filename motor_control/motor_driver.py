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

        # エラー解除
        self.odrive.clear_errors()

    def start_closed_loop_control(self):
        # 閉ループ制御を開始する
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("start closed-loop control for axis" +
              str(self.axis_index))

    def set_velocity_control_mode(self):
        # 速度制御モードに設定
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.axis.controller.config.pos_gain = 20
        self.axis.controller.config.vel_gain = 0.3
        self.axis.controller.config.vel_integrator_gain = 0.32
        self.axis.controller.config.vel_integrator_limit = 1
        print("velocity control mode set.")

    def set_ramped_velocity_control_mode(self, ramp_rate=8.0):
        # 速度制御（速度変化平滑化あり）モードに設定
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.vel_ramp_rate = ramp_rate
        self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.axis.controller.config.pos_gain = 20
        self.axis.controller.config.vel_gain = 0.3
        self.axis.controller.config.vel_integrator_gain = 0.32
        self.axis.controller.config.vel_integrator_limit = 1
        print("ramped velocity control mode set.")

    def get_velocity(self):
        # 速度を取得する
        vel = self.axis.encoder.vel_estimate
        print(str(vel*60) + "[rpm]")

    def set_velocity(self, velocity):
        # 速度設定メソッド
        self.axis.controller.input_vel = velocity
        print(f"Velocity set to {velocity} round/sec.")

    def set_torque_control_mode(self, kv=130, ramp_rate=10, torque_lim=1.5):
        # トルク制御モードに設定
        self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.axis.motor.config.torque_constant = 8.23 / kv
        self.axis.motor.config.torque_lim = torque_lim
        self.axis.controller.config.input_mode = INPUT_MODE_TORQUE_RAMP
        self.axis.controller.config.torque_ramp_rate = ramp_rate
        self.axis.controller.config.pos_gain = 20
        self.axis.controller.config.vel_gain = 0.3
        self.axis.controller.config.vel_integrator_gain = 0.32
        self.axis.controller.config.vel_integrator_limit = 1

    def set_vel_by_torque_mode(self, kv=130, ramp_rate=10, torque_lim=2):
        # トルクベース速度制御モードに設定（Odriveのモードとしてはトルク制御モード）
        self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.axis.motor.config.torque_constant = 8.23 / kv
        self.axis.motor.config.torque_lim = torque_lim
        self.axis.controller.config.input_mode = INPUT_MODE_TORQUE_RAMP
        self.axis.controller.config.torque_ramp_rate = ramp_rate
        self.axis.controller.config.pos_gain = 20
        self.axis.controller.config.vel_gain = 0.3
        self.axis.controller.config.vel_integrator_gain = 0.32
        self.axis.controller.config.vel_integrator_limit = 1

    def set_vel_by_torque(self, target_vel, last_vel, err_integ, last_torque_cmd, motor_current, dt):
        # トルク制御で目標速度[rps]を出す制御
        CURRENT_LIMIT = 20       # [Amps]
        VEL_ERR_INTEG_MAX = 2  # [N]
        torque_cmd = 0

        # ゲイン設定
        kp = 0.22
        kd = 0.6
        ki = 1.5

        # PID制御
        current_vel = self.axis.encoder.vel_estimate    # 現在速度[rps]
        vel_err = target_vel - current_vel              # 速度誤差
        vel_diff = current_vel - last_vel               # 速度変化量
        err_integ += vel_err*dt                         # 速度誤差蓄積量
        if err_integ > 0:
            err_integ = min(err_integ, VEL_ERR_INTEG_MAX)   # 速度誤差の上限キャップ掛け
        else:
            err_integ = max(err_integ, -1*VEL_ERR_INTEG_MAX)
        print(err_integ)
        torque_cmd = kp*vel_err - kd*vel_diff + ki * err_integ

        # モータ電流が既に高いのに更に高いトルクを出そうとしているなら、それ以上出さない
        if abs(motor_current) > CURRENT_LIMIT:
            print("violation to the additional current limit has been detected!")
            if last_torque_cmd > torque_cmd and torque_cmd < 0 and last_torque_cmd < 0:
                torque_cmd = last_torque_cmd
            elif torque_cmd > last_torque_cmd and torque_cmd > 0 and last_torque_cmd > 0:
                torque_cmd = last_torque_cmd
            self.odrive.clear_errors()

        # トルク入力
        self.set_torque(torque_cmd)

        # 返り値
        return torque_cmd, current_vel, err_integ

    def set_torque(self, torque):
        # トルク制御メソッド
        self.axis.controller.input_torque = torque

    def set_idle(self):
        # モータを脱力状態にする
        self.axis.requested_state = AXIS_STATE_IDLE

    def get_motorcurrent_ave(self, current_ave):
        # モータドライバの電流を取得する（ノイズが多いため指数移動平均を取る
        ALPHA = 0.05
        current = self.axis.motor.current_control.Iq_measured
        current_ave = ALPHA*current + (1-ALPHA)*current_ave
        return current_ave
