#!/usr/bin/env python3
"""
CAN消息调试工具 - 用于验证发送的CAN消息格式
"""
import can
import struct
import time

def test_can_send():
    """测试基本的CAN发送"""
    print("CAN发送测试")
    print("=" * 50)
    
    try:
        # 初始化CAN总线（socketcan不需要bitrate参数）
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        print("✓ CAN总线初始化成功")
    except Exception as e:
        print(f"✗ CAN总线初始化失败: {e}")
        return
    
    # 测试1: 发送简单的测试消息
    print("\n测试1: 发送简单测试消息")
    msg = can.Message(
        arbitration_id=0x100,
        data=[0x01, 0x02, 0x03, 0x04],
        is_extended_id=False
    )
    try:
        bus.send(msg)
        print(f"✓ 发送成功: ID=0x{msg.arbitration_id:03X}, 数据={msg.data.hex()}")
    except Exception as e:
        print(f"✗ 发送失败: {e}")
    
    time.sleep(0.1)
    
    # 测试2: 发送ZFOC格式的消息
    print("\n测试2: 发送ZFOC格式消息")
    node_id = 0x01
    cmd_id = 0x16  # MSG_CLEAR_ERRORS
    msg_id = (node_id << 5) | cmd_id
    
    msg = can.Message(
        arbitration_id=msg_id,
        data=[],
        is_extended_id=False
    )
    try:
        bus.send(msg)
        print(f"✓ 发送成功: ID=0x{msg.arbitration_id:03X} (节点={node_id}, 命令=0x{cmd_id:02X})")
    except Exception as e:
        print(f"✗ 发送失败: {e}")
    
    time.sleep(0.1)
    
    # 测试3: 发送设置轴状态消息
    print("\n测试3: 发送设置轴状态消息 (IDLE)")
    cmd_id = 0x06  # MSG_SET_AXIS_REQUESTED_STATE
    msg_id = (node_id << 5) | cmd_id
    state = 1  # IDLE
    data = struct.pack('<I', state)
    
    msg = can.Message(
        arbitration_id=msg_id,
        data=data,
        is_extended_id=False
    )
    try:
        bus.send(msg)
        data_str = ' '.join([f'{b:02X}' for b in data])
        print(f"✓ 发送成功: ID=0x{msg.arbitration_id:03X}, 数据=[{data_str}]")
    except Exception as e:
        print(f"✗ 发送失败: {e}")
    
    time.sleep(0.1)
    
    # 测试4: 发送RTR消息（请求数据）
    print("\n测试4: 发送RTR消息（请求编码器数据）")
    cmd_id = 0x08  # MSG_GET_ENCODER_ESTIMATES
    msg_id = (node_id << 5) | cmd_id
    
    msg = can.Message(
        arbitration_id=msg_id,
        data=[],
        is_extended_id=False,
        is_remote_frame=True
    )
    try:
        bus.send(msg)
        print(f"✓ 发送成功: ID=0x{msg.arbitration_id:03X}, RTR=True")
    except Exception as e:
        print(f"✗ 发送失败: {e}")
    
    bus.shutdown()
    print("\n测试完成")

def monitor_can():
    """监听CAN总线，显示接收到的消息"""
    print("\nCAN总线监听（按Ctrl+C停止）")
    print("=" * 50)
    
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except Exception as e:
        print(f"✗ CAN总线初始化失败: {e}")
        return
    
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg:
                node_id = (msg.arbitration_id >> 5) & 0x3F
                cmd_id = msg.arbitration_id & 0x1F
                data_str = ' '.join([f'{b:02X}' for b in msg.data])
                rtr_str = " [RTR]" if msg.is_remote_frame else ""
                print(f"[接收] ID=0x{msg.arbitration_id:03X} (节点={node_id}, 命令=0x{cmd_id:02X}), 数据=[{data_str}]{rtr_str}")
    except KeyboardInterrupt:
        print("\n停止监听")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "monitor":
        monitor_can()
    else:
        test_can_send()
        print("\n提示: 使用 'python can_debug.py monitor' 可以监听CAN消息")
