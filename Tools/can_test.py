import can
import time
import struct
import threading
import cmd
import sys
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Callable

class AxisState(Enum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13

class ControlMode(Enum):
    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3

class InputMode(Enum):
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8

class CANSimpleProtocol:
    # Command IDs (lower 5 bits of CAN ID)
    MSG_CO_NMT_CTRL = 0x000
    MSG_ZFOC_HEARTBEAT = 0x001
    MSG_ZFOC_ESTOP = 0x002
    MSG_GET_MOTOR_ERROR = 0x003
    MSG_GET_ENCODER_ERROR = 0x004
    MSG_SET_AXIS_NODE_ID = 0x005
    MSG_SET_AXIS_REQUESTED_STATE = 0x006
    MSG_SET_AXIS_STARTUP_CONFIG = 0x007
    MSG_GET_ENCODER_ESTIMATES = 0x008
    MSG_GET_ENCODER_COUNT = 0x009
    MSG_SET_CONTROLLER_MODES = 0x00A
    MSG_SET_INPUT_POS = 0x00B
    MSG_SET_INPUT_VEL = 0x00C
    MSG_SET_INPUT_TORQUE = 0x00D
    MSG_SET_LIMITS = 0x00E
    MSG_START_ANTICOGGING = 0x00F
    MSG_SET_TRAJ_VEL_LIMIT = 0x010
    MSG_SET_TRAJ_ACCEL_LIMITS = 0x011
    MSG_SET_TRAJ_INERTIA = 0x012
    MSG_GET_IQ = 0x013
    MSG_RESET_ZFOC = 0x014
    MSG_GET_BUS_VOLTAGE_CURRENT = 0x015
    MSG_CLEAR_ERRORS = 0x016
    MSG_SET_LINEAR_COUNT = 0x017
    MSG_SET_POS_GAIN = 0x018
    MSG_SET_VEL_GAINS = 0x019
    MSG_GET_ADC_VOLTAGE = 0x01A
    MSG_GET_CONTROLLER_ERROR = 0x01B
    MSG_CO_HEARTBEAT_CMD = 0x700

    NUM_CMD_ID_BITS = 5
    NUM_NODE_ID_BITS = 6

    def __init__(self, node_id: int, is_extended: bool = False):
        self.node_id = node_id
        self.is_extended = is_extended
        self.base_id = node_id << self.NUM_CMD_ID_BITS
        
    def _build_id(self, command_id: int) -> int:
        return self.base_id | command_id

@dataclass
class HeartbeatData:
    error: int
    current_state: AxisState
    motor_flags: int
    encoder_flags: int
    controller_flags: int
    trajectory_done: bool

@dataclass
class EncoderEstimates:
    pos_estimate: float
    vel_estimate: float

@dataclass
class IQData:
    iq_setpoint: float
    iq_measured: float

@dataclass
class BusData:
    voltage: float
    current: float

class ZFOCDriver:
    def __init__(self, channel: str, node_id: int, bustype: str = 'socketcan', 
                 bitrate: int = 500000, is_extended: bool = False):
        self.protocol = CANSimpleProtocol(node_id, is_extended)
        try:
            self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
            self.connected = True
        except Exception as e:
            print(f"无法连接到CAN总线: {e}")
            self.connected = False
            return
            
        self.running = False
        self.listener_thread = None
        self.callbacks = {}
        self.last_heartbeat = None
        self.last_encoder = None
        self.last_iq = None
        self.last_bus = None
        
    def start(self):
        """启动CAN总线监听"""
        if not self.connected:
            print("未连接到CAN总线")
            return False
            
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_loop)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        print(f"ZFOC驱动器节点 {self.protocol.node_id} 已启动")
        return True
    
    def stop(self):
        """停止CAN总线监听"""
        self.running = False
        if self.listener_thread:
            self.listener_thread.join()
        if self.connected:
            self.bus.shutdown()
        print("ZFOC驱动器已停止")
    
    def is_connected(self):
        """检查是否连接"""
        return self.connected and self.running
    
    def _listen_loop(self):
        """监听CAN消息的循环"""
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    self._handle_message(msg)
            except can.CanError as e:
                print(f"CAN错误: {e}")
    
    def _handle_message(self, msg):
        """处理接收到的CAN消息"""
        cmd_id = msg.arbitration_id & 0x1F  # 提取命令ID
        
        # 心跳消息处理
        if cmd_id == CANSimpleProtocol.MSG_ZFOC_HEARTBEAT:
            if len(msg.data) >= 8:
                error = struct.unpack('<I', msg.data[0:4])[0]
                current_state = AxisState(msg.data[4])
                motor_flags = msg.data[5]
                encoder_flags = msg.data[6]
                controller_flags = msg.data[7]
                trajectory_done = bool(controller_flags & 0x80)
                
                self.last_heartbeat = HeartbeatData(
                    error, current_state, motor_flags, 
                    encoder_flags, controller_flags, trajectory_done
                )
                
                if 'heartbeat' in self.callbacks:
                    self.callbacks['heartbeat'](self.last_heartbeat)
        
        # 编码器估计值
        elif cmd_id == CANSimpleProtocol.MSG_GET_ENCODER_ESTIMATES:
            if len(msg.data) >= 8:
                pos = struct.unpack('<f', msg.data[0:4])[0]
                vel = struct.unpack('<f', msg.data[4:8])[0]
                self.last_encoder = EncoderEstimates(pos, vel)
                if 'encoder_estimates' in self.callbacks:
                    self.callbacks['encoder_estimates'](self.last_encoder)
        
        # IQ数据
        elif cmd_id == CANSimpleProtocol.MSG_GET_IQ:
            if len(msg.data) >= 8:
                iq_setpoint = struct.unpack('<f', msg.data[0:4])[0]
                iq_measured = struct.unpack('<f', msg.data[4:8])[0]
                self.last_iq = IQData(iq_setpoint, iq_measured)
                if 'iq_data' in self.callbacks:
                    self.callbacks['iq_data'](self.last_iq)
        
        # 总线电压电流
        elif cmd_id == CANSimpleProtocol.MSG_GET_BUS_VOLTAGE_CURRENT:
            if len(msg.data) >= 8:
                voltage = struct.unpack('<f', msg.data[0:4])[0]
                current = struct.unpack('<f', msg.data[4:8])[0]
                self.last_bus = BusData(voltage, current)
                if 'bus_data' in self.callbacks:
                    self.callbacks['bus_data'](self.last_bus)
    
    def register_callback(self, message_type: str, callback: Callable):
        """注册消息回调函数"""
        self.callbacks[message_type] = callback
    
    def _send_message(self, command_id: int, data: bytes = None, rtr: bool = False):
        """发送CAN消息"""
        if not self.connected:
            print("未连接到CAN总线")
            return False
            
        msg_id = self.protocol._build_id(command_id)
        msg = can.Message(
            arbitration_id=msg_id,
            data=data or [],
            is_extended_id=self.protocol.is_extended,
            is_remote_frame=rtr
        )
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"发送消息失败: {e}")
            return False
    
    # 控制命令方法
    def set_axis_state(self, state: AxisState):
        """设置轴状态"""
        data = struct.pack('<I', state.value)
        return self._send_message(CANSimpleProtocol.MSG_SET_AXIS_REQUESTED_STATE, data)
    
    def set_input_pos(self, position: float, vel_ff: float = 0, torque_ff: float = 0):
        """设置输入位置"""
        vel_ff_int = int(vel_ff * 1000)  # 缩放因子0.001
        torque_ff_int = int(torque_ff * 1000)  # 缩放因子0.001
        data = struct.pack('<fhh', position, vel_ff_int, torque_ff_int)
        return self._send_message(CANSimpleProtocol.MSG_SET_INPUT_POS, data)
    
    def set_input_vel(self, velocity: float, torque_ff: float = 0):
        """设置输入速度"""
        data = struct.pack('<ff', velocity, torque_ff)
        return self._send_message(CANSimpleProtocol.MSG_SET_INPUT_VEL, data)
    
    def set_input_torque(self, torque: float):
        """设置输入扭矩"""
        data = struct.pack('<f', torque)
        return self._send_message(CANSimpleProtocol.MSG_SET_INPUT_TORQUE, data)
    
    def set_controller_modes(self, control_mode: ControlMode, input_mode: InputMode):
        """设置控制器模式"""
        data = struct.pack('<II', control_mode.value, input_mode.value)
        return self._send_message(CANSimpleProtocol.MSG_SET_CONTROLLER_MODES, data)
    
    def set_limits(self, vel_limit: float, current_lim: float):
        """设置限制"""
        data = struct.pack('<ff', vel_limit, current_lim)
        return self._send_message(CANSimpleProtocol.MSG_SET_LIMITS, data)
    
    # 读取命令方法
    def get_encoder_estimates(self):
        """获取编码器估计值"""
        return self._send_message(CANSimpleProtocol.MSG_GET_ENCODER_ESTIMATES, rtr=True)
    
    def get_motor_error(self):
        """获取电机错误"""
        return self._send_message(CANSimpleProtocol.MSG_GET_MOTOR_ERROR, rtr=True)
    
    def get_encoder_error(self):
        """获取编码器错误"""
        return self._send_message(CANSimpleProtocol.MSG_GET_ENCODER_ERROR, rtr=True)
    
    def get_iq(self):
        """获取IQ数据"""
        return self._send_message(CANSimpleProtocol.MSG_GET_IQ, rtr=True)
    
    def get_bus_voltage_current(self):
        """获取总线电压电流"""
        return self._send_message(CANSimpleProtocol.MSG_GET_BUS_VOLTAGE_CURRENT, rtr=True)
    
    # 特殊命令
    def estop(self):
        """紧急停止"""
        return self._send_message(CANSimpleProtocol.MSG_ZFOC_ESTOP)
    
    def clear_errors(self):
        """清除错误"""
        return self._send_message(CANSimpleProtocol.MSG_CLEAR_ERRORS)
    
    def start_anticogging(self):
        """启动抗齿槽转矩校准"""
        return self._send_message(CANSimpleProtocol.MSG_START_ANTICOGGING)
    
    def get_status(self):
        """获取状态信息"""
        status = {
            "connected": self.is_connected(),
            "last_heartbeat": self.last_heartbeat,
            "last_encoder": self.last_encoder,
            "last_iq": self.last_iq,
            "last_bus": self.last_bus
        }
        return status

class ZFOCShell(cmd.Cmd):
    """ZFOC驱动器交互式命令行界面"""
    
    intro = "\n欢迎使用ZFOC驱动器测试工具! 输入 'help' 或 '?' 查看可用命令。\n"
    prompt = 'zfoc> '
    
    def __init__(self, driver):
        super().__init__()
        self.driver = driver
        self.setup_callbacks()
    
    def setup_callbacks(self):
        """设置回调函数"""
        def heartbeat_callback(data: HeartbeatData):
            print(f"\n[心跳] 状态: {data.current_state.name}, 错误: 0x{data.error:08X}")
        
        def encoder_callback(data: EncoderEstimates):
            print(f"\n[编码器] 位置: {data.pos_estimate:.3f}, 速度: {data.vel_estimate:.3f}")
        
        def iq_callback(data: IQData):
            print(f"\n[IQ数据] 设定值: {data.iq_setpoint:.3f}, 测量值: {data.iq_measured:.3f}")
        
        def bus_callback(data: BusData):
            print(f"\n[总线数据] 电压: {data.voltage:.2f}V, 电流: {data.current:.2f}A")
        
        self.driver.register_callback('heartbeat', heartbeat_callback)
        self.driver.register_callback('encoder_estimates', encoder_callback)
        self.driver.register_callback('iq_data', iq_callback)
        self.driver.register_callback('bus_data', bus_callback)
    
    def preloop(self):
        """命令循环前执行"""
        if not self.driver.start():
            print("无法启动驱动器，请检查连接")
            return False
        return True
    
    def postloop(self):
        """退出时执行"""
        self.driver.stop()
    
    def do_status(self, arg):
        """显示驱动器状态"""
        status = self.driver.get_status()
        print(f"\n驱动器状态:")
        print(f"  连接状态: {'已连接' if status['connected'] else '未连接'}")
        
        if status['last_heartbeat']:
            hb = status['last_heartbeat']
            print(f"  最后心跳: 状态={hb.current_state.name}, 错误=0x{hb.error:08X}")
        
        if status['last_encoder']:
            enc = status['last_encoder']
            print(f"  编码器: 位置={enc.pos_estimate:.3f}, 速度={enc.vel_estimate:.3f}")
        
        if status['last_iq']:
            iq = status['last_iq']
            print(f"  IQ数据: 设定值={iq.iq_setpoint:.3f}, 测量值={iq.iq_measured:.3f}")
        
        if status['last_bus']:
            bus = status['last_bus']
            print(f"  总线数据: 电压={bus.voltage:.2f}V, 电流={bus.current:.2f}A")
    
    def do_clear(self, arg):
        """清除错误"""
        if self.driver.clear_errors():
            print("错误清除命令已发送")
        time.sleep(0.1)
    
    def do_idle(self, arg):
        """设置轴为空闲状态"""
        if self.driver.set_axis_state(AxisState.IDLE):
            print("轴已设置为空闲状态")
        time.sleep(0.1)
    
    def do_closed_loop(self, arg):
        """设置轴为闭环控制状态"""
        if self.driver.set_axis_state(AxisState.CLOSED_LOOP_CONTROL):
            print("轴已设置为闭环控制状态")
        time.sleep(0.1)
    
    def do_position_mode(self, arg):
        """设置位置控制模式"""
        if self.driver.set_controller_modes(ControlMode.POSITION_CONTROL, InputMode.PASSTHROUGH):
            print("已设置为位置控制模式")
        time.sleep(0.1)
    
    def do_velocity_mode(self, arg):
        """设置速度控制模式"""
        if self.driver.set_controller_modes(ControlMode.VELOCITY_CONTROL, InputMode.PASSTHROUGH):
            print("已设置为速度控制模式")
        time.sleep(0.1)
    
    def do_torque_mode(self, arg):
        """设置扭矩控制模式"""
        if self.driver.set_controller_modes(ControlMode.TORQUE_CONTROL, InputMode.PASSTHROUGH):
            print("已设置为扭矩控制模式")
        time.sleep(0.1)
    
    def do_move(self, arg):
        """移动到指定位置: move <位置> [速度前馈] [扭矩前馈]"""
        try:
            args = arg.split()
            if len(args) < 1:
                print("用法: move <位置> [速度前馈] [扭矩前馈]")
                return
            
            pos = float(args[0])
            vel_ff = float(args[1]) if len(args) > 1 else 0
            torque_ff = float(args[2]) if len(args) > 2 else 0
            
            if self.driver.set_input_pos(pos, vel_ff, torque_ff):
                print(f"移动到位置 {pos} (速度前馈: {vel_ff}, 扭矩前馈: {torque_ff})")
        except ValueError:
            print("错误: 参数必须是数字")
        time.sleep(0.1)
    
    def do_velocity(self, arg):
        """设置速度: velocity <速度> [扭矩前馈]"""
        try:
            args = arg.split()
            if len(args) < 1:
                print("用法: velocity <速度> [扭矩前馈]")
                return
            
            vel = float(args[0])
            torque_ff = float(args[1]) if len(args) > 1 else 0
            
            if self.driver.set_input_vel(vel, torque_ff):
                print(f"设置速度 {vel} (扭矩前馈: {torque_ff})")
        except ValueError:
            print("错误: 参数必须是数字")
        time.sleep(0.1)
    
    def do_torque(self, arg):
        """设置扭矩: torque <扭矩>"""
        try:
            if not arg:
                print("用法: torque <扭矩>")
                return
            
            torque = float(arg)
            if self.driver.set_input_torque(torque):
                print(f"设置扭矩 {torque}")
        except ValueError:
            print("错误: 参数必须是数字")
        time.sleep(0.1)
    
    def do_get_encoder(self, arg):
        """获取编码器数据"""
        if self.driver.get_encoder_estimates():
            print("编码器数据请求已发送")
        time.sleep(0.1)
    
    def do_get_iq(self, arg):
        """获取IQ数据"""
        if self.driver.get_iq():
            print("IQ数据请求已发送")
        time.sleep(0.1)
    
    def do_get_bus(self, arg):
        """获取总线数据"""
        if self.driver.get_bus_voltage_current():
            print("总线数据请求已发送")
        time.sleep(0.1)
    
    def do_estop(self, arg):
        """紧急停止"""
        if self.driver.estop():
            print("紧急停止命令已发送")
        time.sleep(0.1)
    
    def do_limits(self, arg):
        """设置限制: limits <速度限制> <电流限制>"""
        try:
            args = arg.split()
            if len(args) < 2:
                print("用法: limits <速度限制> <电流限制>")
                return
            
            vel_limit = float(args[0])
            current_lim = float(args[1])
            
            if self.driver.set_limits(vel_limit, current_lim):
                print(f"设置限制: 速度={vel_limit}, 电流={current_lim}")
        except ValueError:
            print("错误: 参数必须是数字")
        time.sleep(0.1)
    
    def do_test(self, arg):
        """运行自动化测试序列"""
        print("开始自动化测试...")
        
        try:
            # 1. 清除错误
            print("1. 清除错误")
            self.driver.clear_errors()
            time.sleep(0.5)
            
            # 2. 设置空闲状态
            print("2. 设置空闲状态")
            self.driver.set_axis_state(AxisState.IDLE)
            time.sleep(1)
            
            # 3. 设置闭环控制
            print("3. 设置闭环控制")
            self.driver.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
            time.sleep(1)
            
            # 4. 设置位置控制模式
            print("4. 设置位置控制模式")
            self.driver.set_controller_modes(ControlMode.POSITION_CONTROL, InputMode.PASSTHROUGH)
            time.sleep(0.5)
            
            # 5. 获取编码器数据
            print("5. 获取编码器数据")
            self.driver.get_encoder_estimates()
            time.sleep(0.5)
            
            # 6. 移动到位置10
            print("6. 移动到位置10")
            self.driver.set_input_pos(10.0)
            time.sleep(2)
            
            # 7. 获取IQ数据
            print("7. 获取IQ数据")
            self.driver.get_iq()
            time.sleep(0.5)
            
            # 8. 获取总线数据
            print("8. 获取总线数据")
            self.driver.get_bus_voltage_current()
            time.sleep(0.5)
            
            # 9. 返回位置0
            print("9. 返回位置0")
            self.driver.set_input_pos(0.0)
            time.sleep(2)
            
            print("测试完成!")
            
        except KeyboardInterrupt:
            print("测试被用户中断")
    
    def do_quit(self, arg):
        """退出程序"""
        print("再见!")
        return True
    
    def do_exit(self, arg):
        """退出程序"""
        return self.do_quit(arg)
    
    def default(self, line):
        """处理未知命令"""
        print(f"未知命令: {line}")
        print("输入 'help' 查看可用命令")

def main():
    """主函数"""
    print("ZFOC CAN协议测试工具")
    print("=" * 40)
    
    # 配置参数
    channel = 'can0'
    node_id = 0x01
    bitrate = 500000
    
    print(f"配置: 接口={channel}, 节点ID={node_id}, 比特率={bitrate}")
    print("正在初始化驱动器...")
    
    # 创建驱动器实例
    driver = ZFOCDriver(channel=channel, node_id=node_id, bitrate=bitrate)
    
    if not driver.connected:
        print("无法连接到CAN总线，请检查:")
        print("1. CAN接口是否正确配置")
        print("2. 设备是否已连接")
        print("3. 权限设置是否正确")
        sys.exit(1)
    
    # 启动交互式shell
    shell = ZFOCShell(driver)
    
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        driver.stop()

if __name__ == "__main__":
    main()