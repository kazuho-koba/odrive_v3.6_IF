#!/usr/bin/env python3

from motor_control.motor_driver import MotorDriver
import time


def main():
    # クラスのインスタンス化
    ax0 = MotorDriver(axis_index=0)
    # ax1 = MotorDriver(axis_index=1)

    # 閉ループ制御の開始
    ax0.start_closed_loop_control()

    # トルク制御モードの開始
    ax0.set_vel_by_torque_mode(kv = 130, ramp_rate=10, torque_lim=1.8)

    # 変数定義
    last_vel = 0            # 1ループ前のモータ回転速度[rps]
    motorcurrent_ave = 0    # モータ供給電流（指数移動平均
    err_integ = 0           # 速度制御の累積偏差（I制御用
    last_torque_cmd = 0     # 1ループ前のトルク制御司令値
    dt = 0.001              # 制御ループ周期
    last_time = time.time() # 1ループ前の時刻


    # 動作テスト
    try:
        print("Running motors...")

        start_time = time.tiem()# モータ制御開始時刻
        while time.time() - start_time < 2:
            target_vel = 2              # 目標速度のセット
            dt = time.time()-last_time
            last_time = time.time()
            motorcurrent_ave = ax0.get_motorcurrent_ave(motorcurrent_ave)
            last_torque_cmd, last_vel, err_integ = ax0.set_vel_by_torque(target_vel, last_vel, err_integ, last_torque_cmd, motorcurrent_ave, dt)
            print("motor current:" + str(motorcurrent_ave) + "[rps],  motor velocity:" + str(last_vel) + "[rps],  set torque:" + str(last_torque_cmd) + "[Nm]")

        start_time = time.time()
        while time.time() - start_time < 2:
            target_vel = -2              # 目標速度のセット
            dt = time.time()-last_time
            last_time = time.time()
            last_torque_cmd, last_vel, err_integ = ax0.set_vel_by_torque(target_vel, last_vel, err_integ, last_torque_cmd, motorcurrent_ave, dt)
            print("motor current:" + str(motorcurrent_ave) + "[rps],  motor velocity:" + str(last_vel) + "[rps],  set torque:" + str(last_torque_cmd) + "[Nm]")

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        # 例外の有無に関わらずモータを停止する
        ax0.set_idle()


if __name__ == "__main__":
    main()
