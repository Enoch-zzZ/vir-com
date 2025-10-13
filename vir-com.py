import time
import threading
import serial
import struct
import random
import sys

# --- 配置 ---
# 这些端口是Python脚本（模拟器）连接的端口
# 你的Qt应用应该连接到与之配对的端口 (com14, com15, com16, com17)
BAUD_RATE = 115200
ACTUATOR_PORT = 'COM27'  # 模拟Arduino驱动器
PRESSURE_PORTS_CONFIG = {
    'COM24': {'channel_id': 1, 'base_kpa': 1.5},
    'COM25': {'channel_id': 2, 'base_kpa': 2.6},
    'COM26': {'channel_id': 3, 'base_kpa': 3.4},
}

# --- 共享状态 ---
# 这个字典模拟了驱动器的物理状态，所有线程都可以访问它
actuator_state = {
    # 存储每个通道的PWM值 (0-255)，由'W'指令设置
    'pwm': [0, 0, 0],
    # 存储每个通道的模拟负压 (kPa)，由物理模型线程更新
    'pressure_kpa': [0.0, 0.0, 0.0],
    # 线程停止标志
    'stop_event': threading.Event()
}


def pressure_sensor_simulator(port_name, channel_id, base_kpa):
    """
    模拟一个眶压传感器，定期发送数据。
    它会模拟Qt程序 onPressureDataReceived() 函数期望的格式。
    """
    print(f"[眶压传感器 {channel_id} -> {port_name}] 线程启动。")
    ser = None
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        while not actuator_state['stop_event'].is_set():
            # 模拟一个轻微波动的压力值
            current_pressure = base_kpa + random.uniform(-0.1, 0.1)
            
            # 增加一个逻辑：如果驱动器在工作，眶压会轻微上升
            actuator_pwm_sum = sum(actuator_state['pwm'])
            if actuator_pwm_sum > 10:
                current_pressure += actuator_pwm_sum / 255.0 * 0.5

            # 格式化为 "ID,压力值\r\n"
            data_to_send = f"{channel_id},{current_pressure:.2f}\r\n"
            ser.write(data_to_send.encode('ascii'))
            
            time.sleep(0.1) # 每秒发送10次数据
            
    except serial.SerialException as e:
        print(f"错误: 无法打开或写入串口 {port_name}: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        print(f"[眶压传感器 {channel_id} -> {port_name}] 线程停止。")


def update_physics():
    """
    一个简单的物理模型。根据当前的PWM值，随时间更新驱动器负压。
    这个线程确保了即使没有新的'W'指令，压力也会根据当前PWM值动态变化。
    """
    print("[物理模型] 线程启动。")
    MIN_PRESSURE_KPA = -89.0  # 来自 v3.1_ThreeChannel.ino.txt [cite: 7]
    
    while not actuator_state['stop_event'].is_set():
        for i in range(3):
            pwm = actuator_state['pwm'][i]
            current_p = actuator_state['pressure_kpa'][i]

            # 目标压力与PWM值成正比
            target_p = -(pwm / 255.0) * abs(MIN_PRESSURE_KPA)
            
            # 模拟压力向目标值缓慢变化的过程
            # 如果PWM大于0，压力会向更负的方向变化（抽气）
            # 如果PWM等于0，压力会向0的方向变化（泄漏/恢复）
            new_p = current_p + (target_p - current_p) * 0.05 # 0.05是变化速率系数

            actuator_state['pressure_kpa'][i] = max(MIN_PRESSURE_KPA, new_p)

        time.sleep(0.02) # 以50Hz的频率更新物理状态，与Qt控制循环匹配
    print("[物理模型] 线程停止。")


def actuator_simulator(port_name):
    """
    模拟Arduino驱动器控制器。
    监听来自Qt应用的 'R', 'W', 'S' 指令并作出响应。
    """
    print(f"[驱动器 -> {port_name}] 线程启动，等待指令...")
    ser = None
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        while not actuator_state['stop_event'].is_set():
            if ser.in_waiting > 0:
                command = ser.read(1).decode('ascii')
                
                if command == 'R':  # 请求压力 [cite: 10]
                    if ser.in_waiting > 0:
                        channel_char = ser.read(1).decode('ascii')
                        channel_idx = int(channel_char) - 1
                        if 0 <= channel_idx < 3:
                            pressure = actuator_state['pressure_kpa'][channel_idx]
                            # [cite_start]将浮点数打包为4字节二进制数据发送，模拟Arduino行为 [cite: 25]
                            ser.write(struct.pack('<f', pressure))
                            print(f"  -> 收到 'R{channel_idx+1}', 发送压力: {pressure:.2f} kPa")

                elif command == 'W': # 设定PWM [cite: 11]
                    if ser.in_waiting >= 2:
                        channel_char = ser.read(1).decode('ascii')
                        pwm_byte = ser.read(1)
                        pwm_value = int.from_bytes(pwm_byte, 'big')
                        channel_idx = int(channel_char) - 1
                        if 0 <= channel_idx < 3:
                            actuator_state['pwm'][channel_idx] = pwm_value
                            print(f"  -> 收到 'W{channel_idx+1}', 设置PWM为: {pwm_value}")
                
                elif command == 'S': # 紧急停止 [cite: 12]
                    actuator_state['pwm'] = [0, 0, 0]
                    print("  -> 收到 'S' 指令, 所有PWM已归零。")

    except serial.SerialException as e:
        print(f"错误: 无法打开或操作串口 {port_name}: {e}")
    except (ValueError, IndexError, TypeError) as e:
        print(f"错误: 处理指令时发生数据错误: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        print(f"[驱动器 -> {port_name}] 线程停止。")


if __name__ == "__main__":
    threads = []

    print("--- 启动硬件模拟器 ---")
    print("按下 Ctrl+C 停止程序。")

    try:
        # 启动物理模型线程
        physics_thread = threading.Thread(target=update_physics)
        threads.append(physics_thread)
        physics_thread.start()

        # 启动驱动器模拟线程
        actuator_thread = threading.Thread(target=actuator_simulator, args=(ACTUATOR_PORT,))
        threads.append(actuator_thread)
        actuator_thread.start()

        # 启动所有眶压传感器模拟线程
        for port, config in PRESSURE_PORTS_CONFIG.items():
            p_thread = threading.Thread(
                target=pressure_sensor_simulator,
                args=(port, config['channel_id'], config['base_kpa'])
            )
            threads.append(p_thread)
            p_thread.start()
        
        # 主线程等待所有子线程结束
        for t in threads:
            t.join()

    except KeyboardInterrupt:
        print("\n收到退出指令 (Ctrl+C)...")
        actuator_state['stop_event'].set()
        # 等待所有线程干净地退出
        for t in threads:
            if t.is_alive():
                t.join()
        print("--- 模拟器已关闭 ---")
    except Exception as e:
        print(f"发生未知错误: {e}")
        actuator_state['stop_event'].set()
        sys.exit(1)

        # 你是？