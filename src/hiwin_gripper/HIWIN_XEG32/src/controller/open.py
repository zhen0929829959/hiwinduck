from ctypes import CDLL
import time


SO_FILE = "./Hiwin_API.so"


modbus = CDLL(SO_FILE)


def wait_until(target_mode, timeout_sec=30.0):
    """
    持續讀取夾爪狀態，
    直到狀態等於 target_mode 才結束等待。

    timeout_sec:
    最長等待時間，避免夾爪異常時程式永遠卡住。
    """
    start_time = time.time()

    while True:
        state = modbus.read_mode()

        if state == target_mode:
            return True

        if time.time() - start_time > timeout_sec:
            print(
                f"Wait timeout: current state={state}, "
                f"target state={target_mode}"
            )
            return False

        time.sleep(0.01)


def move(dis=0, speed=1200, flag=1):
    """
    控制夾爪移動。

    dis:
    目標位置，範圍約為 0～3200

    speed:
    移動速度

    flag:
    控制旗標
    """
    print(
        f"Moving: dis={dis}, "
        f"speed={speed}, flag={flag}"
    )

    result = modbus.move(dis, speed, flag)

    print(f"Move command result: {result}")

    time.sleep(0.01)

    # 等待夾爪移動完成
    success = wait_until(
        target_mode=2,
        timeout_sec=30.0
    )

    if not success:
        print("Move failed or timed out")
        return False

    print("Move done")
    return True


def reset_gripper():
    print("Resetting gripper...")

    result = modbus.reset()

    print(f"Reset command result: {result}")

    # 原本設定等待 25 秒
    time.sleep(25)

    print("Reset done")


def open_gripper():
    print("Opening gripper...")

    success = move(
        dis=1600,
        speed=4800,
        flag=1
    )

    if success:
        print("Open done")
    else:
        print("Open failed")


def close_gripper():
    print("Closing gripper...")

    success = move(
        dis=100,
        speed=4800,
        flag=1
    )

    if success:
        print("Close done")
    else:
        print("Close failed")


def main():
    print("Connecting Modbus...")

    connect_result = modbus.libModbus_Connect(1)

    print(f"Connect result: {connect_result}")

    if connect_result < 0:
        print("Modbus connection failed")
        return

    print("Modbus connected")

    try:
        while True:
            print()
            print("========== Gripper Control ==========")
            print("1 = Reset")
            print("2 = Open")
            print("3 = Close")
            print("q = Quit")
            print("=====================================")

            mode = input(
                "Please enter command: "
            ).strip()

            if mode == "1":
                reset_gripper()

            elif mode == "2":
                open_gripper()

            elif mode == "3":
                close_gripper()

            elif mode.lower() == "q":
                print("Closing program...")
                break

            else:
                print(f"Unknown command: {mode}")

    except KeyboardInterrupt:
        print("\nProgram stopped by user")

    finally:
        modbus.Modbus_Close()
        print("Modbus closed")


if __name__ == "__main__":
    main()