#!/usr/bin/env python3

from motor_control.motor_driver import MotorDriver
import time


def main():
    # クラスのインスタンス化
    ax0 = MotorDriver(axis_index=0)
    # ax1 = MotorDriver(axis_index=1)

    # 閉ループ制御の開始
    ax0.start_closed_loop_control()

    # 速度制御の開始
    # ax0.set_velocity_control_mode()                     # 平滑化なし
    ax0.set_ramped_velocity_control_mode(ramp_rate=10)  # 平滑化あり

    # 動作テスト
    try:
        print("Running motors...")
        ax0.set_velocity(10)  # axis0: 速度2rps
        time.sleep(2)  # 5秒間動作
        ax0.get_velocity()
        time.sleep(3)

        ax0.set_velocity(0)  # 停止

        time.sleep(1)  # 5秒間動作

        ax0.set_velocity(-10)  # axis0: 速度-2rps

        time.sleep(5)  # 5秒間動作

        ax0.set_velocity(0)  # 停止

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # 例外の有無に関わらずモータを停止する
        ax0.set_velocity(0)


if __name__ == "__main__":
    main()
