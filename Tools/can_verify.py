#!/usr/bin/env python3
"""
CAN通信验证工具 - 对比cansend和python-can的发送结果
"""
import can
import struct
import subprocess
import time

def send_with_python(node_id, cmd_id, data_bytes=None):
    """使用python-can发送消息"""
    msg_id = (node_id << 5) | cmd_id
    
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    msg = can.Message(
        arbitration_id=msg_id,
        data=data_bytes or [],
        is_extended_id=False
    )
    bus.send(msg)
    bus.shutdown()
    
    data_str = ' '.join([f'{b:02X}' for b in (data_bytes or [])])
    print(f"Python发送: ID=0x{msg_id:03X} 数据=[{data_str}]")
    return msg_id, data_bytes

def send_with_cansend(node_id, cmd_id, data_bytes=None):
    """使用cansend发送消息"""
    msg_id = (node_id << 5) | cmd_id
    
    if data_bytes:
        data_hex = ''.join([f'{b:02X}' for b in data_bytes])
        cmd = f"cansend can0 {msg_id:03X}#{data_hex}"
    else:
        cmd = f"cansend can0 {msg_id:03X}#"
    
    subprocess.run(cmd, shell=True, check=True)
    data_str = ' '.join([f'{b:02X}' for b in (data_bytes or [])]) if data_bytes else ""
    print(f"cansend发送: {cmd}")
    print(f"  解析: ID=0x{msg_id:03X} 数据=[{data_str}]")

def main():
    print("CAN消息发送对比测试")
    print("=" * 60)
    
    node_id = 1
    sleep = 1.0
    
    # 测试1: 清除错误
    print("\n测试1: 清除错误 (MSG_CLEAR_ERRORS = 0x16)")
    print("-" * 60)
    cmd_id = 0x16
    print("方法1 - cansend:")
    send_with_cansend(node_id, cmd_id)
    time.sleep(sleep)
    
    print("\n方法2 - python-can:")
    send_with_python(node_id, cmd_id)
    time.sleep(sleep)
    
    # 测试2: 设置IDLE状态
    print("\n\n测试2: 设置IDLE状态 (MSG_SET_AXIS_REQUESTED_STATE = 0x06)")
    print("-" * 60)
    cmd_id = 0x06
    state_data = struct.pack('<I', 1)  # IDLE = 1
    
    print("方法1 - cansend:")
    send_with_cansend(node_id, cmd_id, state_data)
    time.sleep(sleep)
    
    print("\n方法2 - python-can:")
    send_with_python(node_id, cmd_id, state_data)
    time.sleep(sleep)
    
    # 测试3: 设置闭环控制
    print("\n\n测试3: 设置闭环控制 (MSG_SET_AXIS_REQUESTED_STATE = 0x06)")
    print("-" * 60)
    cmd_id = 0x06
    state_data = struct.pack('<I', 8)  # CLOSED_LOOP_CONTROL = 8
    
    print("方法1 - cansend:")
    send_with_cansend(node_id, cmd_id, state_data)
    time.sleep(sleep)
    
    print("\n方法2 - python-can:")
    send_with_python(node_id, cmd_id, state_data)
    time.sleep(sleep)
    
    # 测试4: 设置控制器模式
    print("\n\n测试4: 设置位置控制模式 (MSG_SET_CONTROLLER_MODES = 0x0A)")
    print("-" * 60)
    cmd_id = 0x0A
    # control_mode = 3 (POSITION_CONTROL), input_mode = 1 (PASSTHROUGH)
    modes_data = struct.pack('<II', 3, 1)
    
    print("方法1 - cansend:")
    send_with_cansend(node_id, cmd_id, modes_data)
    time.sleep(sleep)
    
    print("\n方法2 - python-can:")
    send_with_python(node_id, cmd_id, modes_data)
    time.sleep(sleep)
    
    # 测试5: RTR消息
    print("\n\n测试5: 请求编码器数据 (MSG_GET_ENCODER_ESTIMATES = 0x08)")
    print("-" * 60)
    cmd_id = 0x08
    msg_id = (node_id << 5) | cmd_id
    
    print("方法1 - cansend RTR:")
    subprocess.run(f"cansend can0 {msg_id:03X}#R", shell=True, check=True)
    time.sleep(sleep)
    
    print("\n方法2 - python-can RTR:")
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    msg = can.Message(
        arbitration_id=msg_id,
        data=[],
        is_extended_id=False,
        is_remote_frame=True
    )
    bus.send(msg)
    bus.shutdown()
    print(f"Python发送RTR: ID=0x{msg_id:03X}")
    time.sleep(sleep)

if __name__ == "__main__":
    main()
