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
    ax0.set_torque_control_mode(kv=130, ramp_rate=10, torque_lim=1.5)

    # 動作テスト
    try:
        print("Running motors...")
        ax0.set_torque(0.05)
        time.sleep(2)
        ax0.get_velocity()
        time.sleep(2)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # 例外の有無に関わらずモータを停止する
        ax0.set_idle()


if __name__ == "__main__":
    main()
