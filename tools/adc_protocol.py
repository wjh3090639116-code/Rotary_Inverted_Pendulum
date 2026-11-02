"""串口二进制协议：提供组帧、流式解析、量纲换算和状态解码。"""

import struct
import math

class FrameDecoder:
    """从任意分段的字节流中提取 A5 5A TYPE LEN PAYLOAD XOR 帧。"""

    def __init__(self, max_payload=64):
        self.buffer = bytearray()
        self.max_payload = max_payload

    def feed(self, data):
        """输入新字节，返回本次已经收齐且校验正确的 (类型, 载荷) 列表。"""
        self.buffer.extend(data)
        result = []
        while True:
            start = self.buffer.find(b"\xA5\x5A")
            if start < 0:
                
                self.buffer[:] = b"\xA5" if self.buffer[-1:] == b"\xA5" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break

            frame_type = self.buffer[2]
            length = self.buffer[3]
            if length > self.max_payload:
                del self.buffer[0]
                continue
            total_length = 2 + 1 + 1 + length + 1
            if len(self.buffer) < total_length:
                break

            payload = self.buffer[4 : 4 + length]
            checksum = frame_type ^ length
            for value in payload:
                checksum ^= value
            if checksum == self.buffer[4 + length]:
                result.append((frame_type, bytes(payload)))
                del self.buffer[:total_length]
            else:
                
                del self.buffer[0]
        return result

def decode_adc(payload):
    """TYPE=34 的三个载荷字节 -> (12 位原生 ADC 码值, ADC 故障标志)。"""
    if len(payload) != 3:
        raise ValueError("ADC 帧载荷应为 3 字节")
    code = payload[0] | (payload[1] << 8)
    if code>4095:raise ValueError("ADS8688 高12位 单端码超出 0–4095")
    return code, payload[2] & 1

def make_frame(frame_type, payload):
    if len(payload) > 255:
        raise ValueError("payload too long")
    checksum = frame_type ^ len(payload)
    for value in payload:
        checksum ^= value
    return bytes((0xA5, 0x5A, frame_type, len(payload))) + payload + bytes((checksum,))

def make_config(zero, angle_sign, encoder_sign, capture, fall, travel,
                kp, ki, kd, arm_kp, arm_kd):
    """构建整体配置帧；比例、微分及外环增益按各自定点精度编码，整体配置会解除运行状态。"""
    if not (16 <= zero <= 4079 and angle_sign in (-1, 1)
            and encoder_sign in (-1, 1) and 1 <= capture <= 1024
            and capture < fall <= 2048 and 1 <= travel <= 32767):
        raise ValueError("零点、方向或安全门限超出 FPGA 允许范围")
    if any(not isinstance(v,int) or isinstance(v,bool) for v in (zero,capture,fall,travel)):raise ValueError("门限必须为整数")
    gains = [kp, ki, kd, arm_kp, arm_kd]
    if not all(math.isfinite(v) for v in gains):raise ValueError("PID 增益必须为有限数字")
    limits = [131072, 1048576, 2097152, 1048576, 4194304]
    q8 = [round(value * scale) for value, scale in zip(gains, (256, 4096, 256, 65536, 65536))]
    if any(abs(value) > limit for value, limit in zip(q8, limits)):
        raise ValueError("PID 增益超出 FPGA 允许范围")
    payload = struct.pack("<HbbHHHiiiii", zero, angle_sign, encoder_sign,
                          capture, fall, travel, *q8)
    return make_frame(0x32, payload)

def make_swing_config(pump_pwm,pulse_ms,bottom_code,target_code,upper_code,bottom_window,rearm_distance,travel,stage2_pwm=None,stage2_ms=None):
    values=(pump_pwm,pulse_ms,bottom_code,target_code,upper_code,bottom_window,rearm_distance,travel)
    if any(not isinstance(v,int) or isinstance(v,bool) for v in values):
        raise ValueError("脉冲起摆参数必须为整数")
    if not (1<=abs(pump_pwm)<=900 and 1<=pulse_ms<=1000 and
            all(0<=code<=4095 for code in (bottom_code,target_code,upper_code)) and
            5<=bottom_window<=64 and
            80<=rearm_distance<=480 and rearm_distance>bottom_window and 5<=travel<=32767):
        raise ValueError("PWM ±1–900，脉冲 1–1000 ms；最低点、目标、最高点 ADC 各为 0–4095，不限制大小顺序或间距；窗口5–64，离开距离80–480，软行程5–32767")
    if stage2_pwm is None and stage2_ms is None:
        return make_frame(0x26,struct.pack("<h7H",*values))
    if any(not isinstance(v,int) or isinstance(v,bool) for v in (stage2_pwm,stage2_ms)) or not (-900<=stage2_pwm<=900 and 1<=stage2_ms<=1000):
        raise ValueError("第二阶段 PWM 为 −900～+900‰（0 为滑行），每次脉冲时长为 1–1000 ms")
    return make_frame(0x28,struct.pack("<h7HhH",*values,stage2_pwm,stage2_ms))

def decode_swing_echo(payload):
    if len(payload) not in (16,20):raise ValueError("起摆参数回显应为 16 或 20 字节")
    return struct.unpack("<h7HhH" if len(payload)==20 else "<h7H",payload)

def make_live_pid(kp,ki,kd):
    """构建内环增益在线更新帧；由板端运行状态决定是否接受，不触发整体配置复位。"""
    values=(kp,ki,kd)
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
        raise ValueError("PID 必须为有限数字")
    if any(abs(v)>limit for v,limit in zip(values,(512,256,8192))):raise ValueError("内环 P/I/D 超出允许范围")
    codes=tuple(round(v*q) for v,q in zip(values,(256,4096,256)))
    if not any(codes):raise ValueError("内环 P/I/D 不能同时为零")
    return make_frame(0x2C,struct.pack("<iii",*codes))

def decode_live_pid(payload):
    if len(payload)!=13:raise ValueError("实时 PID 回显长度错误")
    status=payload[0]
    if status>2:raise ValueError("实时 PID 回显状态错误")
    codes=struct.unpack("<iii",payload[1:])
    if any(abs(v)>limit for v,limit in zip(codes,(131072,1048576,2097152))):raise ValueError("实时 PID 回显超出范围")
    return status,tuple(v/q for v,q in zip(codes,(256,4096,256)))

def make_live_pd(kp,kd):
    """构建外环比例和微分增益帧，使用带符号１６位小数格式。"""
    values=(kp,kd)
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
        raise ValueError("外环 P/D 必须为有限数字")
    if any(abs(v)>limit for v,limit in zip(values,(16,64))):raise ValueError("外环 P/D 超出允许范围")
    return make_frame(0x2E,struct.pack("<ii",*(round(v*65536) for v in values)))

def decode_live_pd(payload):
    if len(payload)!=9 or payload[0]>2:raise ValueError("实时外环 PD 回显无效")
    codes=struct.unpack("<ii",payload[1:])
    if any(abs(v)>limit for v,limit in zip(codes,(1048576,4194304))):raise ValueError("外环 PD 回显超出范围")
    return payload[0],tuple(v/65536 for v in codes)

def make_query_command():
    """构建当前配置回读命令。"""
    return make_frame(0x17, b"")

def make_run_command(start, manual=False):
    """构建幂等启动或停机命令；重复启动请求同时续订远程运行租约。"""
    return make_frame(0x12, bytes(((2 if manual else 1) if start else 0,)))

def make_control_period(period_ms):
    if not math.isfinite(period_ms) or period_ms!=1:raise ValueError("内环固定 1 ms（1000 Hz）")
    return make_frame(0x13,struct.pack("<H",1000))

def make_outer_period(period_ms):
    if not math.isfinite(period_ms) or not 1<=period_ms<=50:raise ValueError("外环周期应为 1–50 ms")
    tenths=round(period_ms*10)
    if abs(period_ms*10-tenths)>1e-6:raise ValueError("外环周期步长为 0.1 ms")
    return make_frame(0x1F,struct.pack("<H",tenths*100))

def make_tilt_limit(codes):
    """将摆杆目标倾角上限编码为高１２位采集码单位。"""
    if not isinstance(codes, int) or isinstance(codes,bool) or not 1 <= codes <= 102:
        raise ValueError("目标倾角上限应为 1–102 个 ADC 码")
    return make_frame(0x16, struct.pack("<H", codes))

def make_pwm_deadzone(per_mille):
    """构建静摩擦补偿设置；最终非零控制量按方向叠加死区补偿。"""
    if not isinstance(per_mille,int) or isinstance(per_mille,bool) or not 0<=per_mille<=300:
        raise ValueError("PWM 死区补偿应为 0–300‰（0–30%），必须为整数")
    return make_frame(0x2A,struct.pack("<H",per_mille))

def make_wrap_config(low, high):
    """配置电位器整圈边界两侧的角度码，用于环形角度展开。"""
    if any(not isinstance(v,int) or isinstance(v,bool) for v in (low,high)) or not (0 <= low <= 499 and low + 1000 <= high <= 4095):
        raise ValueError("换圈端点应满足低端 0–499，高端≤4095，高低码差≥1000")
    return make_frame(0x14, struct.pack("<HH", low, high))

def decode_motor(payload):
    """解析驱动遥测；摆杆速度与目标倾角使用采集码单位，旋臂位置使用编码器计数。"""
    if len(payload) not in (11, 12, 14, 18):
        raise ValueError("电机帧载荷应为 11、12、14 或 18 字节")
    position, velocity, pwm, invalid, flags = struct.unpack("<ihhHB", payload[:11])
    
    reason = payload[11] if len(payload) >= 12 else 255
    return position, velocity, pwm, invalid, flags, reason

def decode_config_echo(payload):
    if len(payload)!=30:raise ValueError("原生配置回显应为 30 字节")
    return struct.unpack("<HbbHHHiiiii",payload)

def make_arm_target(counts,tilt_limit=None):
    """构建相对启动中心的绝对旋臂目标，使用带符号３２位计数。"""
    if not isinstance(counts, int) or not -32767 <= counts <= 32767:
        raise ValueError("转臂目标超出 ±32767 编码器计数")
    if tilt_limit is None:return make_frame(0x1B, struct.pack("<i", counts))
    if not isinstance(tilt_limit,int) or isinstance(tilt_limit,bool) or not 1<=tilt_limit<=64:
        raise ValueError("随目标发送的倾角限幅应为 1–64 控制码")
    return make_frame(0x1B,struct.pack('<iH',counts,tilt_limit))

def shifted_arm_target(current, degrees, counts_per_rev, travel):
    """按指定方向累加角度，不执行整圈取模或最短路径折返。"""
    if not math.isfinite(degrees) or not 0 < abs(degrees) <= 360:
        raise ValueError("每次增减角度应大于 0° 且不超过 360°")
    if not 1 <= counts_per_rev <= 1000000 or not 1 <= travel <= 32767:
        raise ValueError("编码器每圈计数或行程门限无效")
    delta = round(degrees * counts_per_rev / 360)
    if delta == 0:
        raise ValueError(f"角度小于编码器分辨率（每计数 {360/counts_per_rev:.4f}°）")
    target = current + delta
    if abs(target) > travel:
        raise ValueError(f"目标 {target*360/counts_per_rev:+.2f}° 超出启动中心两侧 ±{travel*360/counts_per_rev:.2f}° 行程")
    return target

def decode_arm_target_echo(payload):
    if len(payload) not in (5,7):
        raise ValueError("转臂目标回显应为 5 或 7 字节")
    return struct.unpack("<iB", payload[:5])

def decode_touch_state(payload):
    """解析操作序号、触屏动作、执行结果、运行控制权、角度零点和旋臂目标。
    动作包括起摆、停机、校零、步进、回中心、整圈及往复控制；回显反映板端实际执行结果。"""
    if len(payload) != 11:
        raise ValueError("触摸状态回显应为 11 字节")
    return struct.unpack("<HBBBHi", payload)


def step_tilt_limit(degrees,small_limit=20,large_limit=40):
    if not math.isfinite(degrees) or not 0<abs(degrees)<=360:
        raise ValueError("每次增减角度应大于 0° 且不超过 360°")
    if any(not isinstance(v,int) or isinstance(v,bool) or not 1<=v<=64 for v in (small_limit,large_limit)):
        raise ValueError("两档倾角限幅均应为1–64整数控制码")
    return small_limit if abs(degrees)<180 else large_limit

def turn_tilt_limit(degrees,normal,forward,reverse):
    if any(not isinstance(v,int) or isinstance(v,bool) or not 1<=v<=64 for v in (normal,forward,reverse)):
        raise ValueError("三种倾角上限均应为1–64整数ADC码")
    if not math.isfinite(degrees) or abs(degrees)>360:
        raise ValueError("单次目标增量应在±360°以内")
    return forward if abs(degrees-360)<1e-8 else reverse if abs(degrees+360)<1e-8 else normal

def make_board_config(counts_per_rev,amplitude_degrees,interval_seconds,small_limit=None,large_limit=None,reverse_limit=None):
    if not isinstance(counts_per_rev,int) or isinstance(counts_per_rev,bool) or not 1<=counts_per_rev<=32767:
        raise ValueError("板载控制每圈计数应为1–32767整数")
    if not math.isfinite(amplitude_degrees) or amplitude_degrees<=0:
        raise ValueError("往复单边幅度必须为正数")
    amplitude=round(amplitude_degrees*counts_per_rev/360)
    if not 1<=amplitude<=32767:raise ValueError("往复幅度超出编码器分辨率或32767计数范围")
    if not math.isfinite(interval_seconds) or not .1<=interval_seconds<=600:
        raise ValueError("往复切换间隔应为0.1–600秒")
    interval=round(interval_seconds*1000)
    extended=small_limit is not None or large_limit is not None
    if not extended:small_limit,large_limit=20,40
    
    if reverse_limit is not None:
        turn_tilt_limit(0,small_limit,large_limit,reverse_limit)
    else:step_tilt_limit(180,small_limit,large_limit)
    limit=small_limit if reverse_limit is not None or amplitude_degrees*2<180 else large_limit
    payload=struct.pack('<HHIH',counts_per_rev,amplitude,interval,limit)
    if extended:payload+=struct.pack('<HH',small_limit,large_limit)
    if reverse_limit is not None:payload+=struct.pack('<H',reverse_limit)
    return make_frame(0x40,payload)

def decode_board_echo(payload):
    if len(payload) not in (12,16,18):raise ValueError("板载控制回显应为12、16或18字节")
    counts,amplitude,interval,limit,status,flags=struct.unpack('<HHIHBB',payload[:12])
    if not 1<=counts<=32767 or not 1<=amplitude<=32767 or not 100<=interval<=600000 or not 1<=limit<=64 or status>2:
        raise ValueError("板载控制回显无效")
    if len(payload)>=16:
        small,large=struct.unpack('<HH',payload[12:16])
        step_tilt_limit(180,small,large)
    if len(payload)==18:
        reverse=struct.unpack('<H',payload[16:])[0]
        turn_tilt_limit(0,small,large,reverse)
        
    return counts,amplitude,interval,limit,status,flags

def make_target_trim(counts_per_rev,tolerance_degrees,enabled=True,max_trim_degrees=60):
    if not isinstance(counts_per_rev,int) or not 1<=counts_per_rev<=32767:
        raise ValueError("每圈计数应为1–32767整数")
    if not math.isfinite(tolerance_degrees) or not 0<=tolerance_degrees<=180:
        raise ValueError("目标容差应为0–180度")
    tolerance_q8=round(tolerance_degrees*counts_per_rev*256/360)
    if not math.isfinite(max_trim_degrees) or not 0<max_trim_degrees<=180:
        raise ValueError("最大补偿角度应大于0且不超过180°")
    maximum=max(1,round(max_trim_degrees*counts_per_rev/360))
    return make_frame(0x37,struct.pack('<IHH',tolerance_q8,maximum,int(bool(enabled))))

def decode_target_trim(payload):
    if len(payload)!=9:raise ValueError("目标补偿回显长度应为9字节")
    tolerance,maximum,flags,status=struct.unpack('<IHHB',payload)
    if tolerance>8388352 or not 1<=maximum<=32767 or flags>1 or status>2:
        raise ValueError("目标补偿回显无效")
    return tolerance,maximum,bool(flags),status

def make_monitor_subscription(enabled=True):
    return make_frame(0x3A,bytes((int(bool(enabled)),)))

def trim_runtime_text(reading):
    state=reading['trim_state'];bias=reading['internal_degrees']-reading['user_degrees']
    suffix=f" · 内部偏置 {bias:+.2f}°"
    if not state&128:return "补偿：旧版逻辑，请下载修正版固件"
    if not state&64:return "补偿：未启用，请检查开关并下发全部参数"
    if not reading['flags']&2:return "补偿：等待直立运行"
    if state&16:return "补偿：参考稳定等待，保持已有偏置"+suffix
    if state&32:return "补偿：正在建立误差均值"+suffix
    if state&8:return "补偿：已达补偿/行程上限"+suffix
    if state&2:return "补偿：正在修正平均偏差"+suffix
    if state&4:return "补偿：保持已有偏置"+suffix
    return "补偿：转臂移动较快，暂缓修正"+suffix

def decode_monitor(payload):
    if len(payload) not in (24,29):raise ValueError("监测帧应为24或29字节")
    user,internal,actual,velocity,pwm,adc,flags,trim_state,counts=struct.unpack('<4ihHBBH',payload[:24])
    if not 1<=counts<=32767 or adc>4095 or abs(pwm)>900:
        raise ValueError("监测帧数值无效")
    result={'user_counts':user,'internal_counts':internal,'actual_counts':actual,
            'velocity_q8':velocity,'pwm_per_mille':pwm,'adc_code':adc,'flags':flags,'trim_state':trim_state,
            'counts_per_rev':counts,'user_degrees':user*360/counts,'internal_degrees':internal*360/counts,
            'actual_degrees':actual*360/counts,'speed_degrees_s':velocity*36000/(counts*256)}
    if len(payload)==29:
        center,board_flags=struct.unpack('<iB',payload[24:])
        result.update(reciprocal_center_counts=center,reciprocal_center_degrees=center*360/counts,
                      board_flags=board_flags,reciprocal_active=bool(board_flags&4),reciprocal_enabled=bool(board_flags&2))
    return result

def make_reciprocal_command(enabled):
    return make_frame(0x42,bytes((int(bool(enabled)),)))

def make_board_handoff():
    return make_frame(0x12,b'\x03')

def make_response_speed(fast):
    if not isinstance(fast,bool):raise ValueError("响应档位必须为原速或快速")
    return make_frame(0x44,bytes((int(fast),)))

def decode_response_speed(payload):
    if len(payload)!=2 or payload[0]>2 or payload[1]>1:raise ValueError("响应速度回显无效")
    return payload[0],bool(payload[1])

def make_correction_tuning(strength,max_degrees_s,counts_per_rev=1040):
    if isinstance(strength,bool) or not isinstance(strength,int) or not 0<=strength<=2:raise ValueError("补偿强度无效")
    if isinstance(max_degrees_s,bool) or not isinstance(max_degrees_s,(int,float)) or not math.isfinite(max_degrees_s) or not .5<=max_degrees_s<=10:raise ValueError("最大修正速度应为0.5–10°/秒")
    if counts_per_rev!=1040:raise ValueError("本工程使用1040计数/圈")
    limit=math.floor(max_degrees_s*counts_per_rev*256/18000+1e-9)
    return make_frame(0x46,struct.pack('<BH',strength,limit))

def decode_correction_tuning(payload):
    if len(payload)!=4:raise ValueError("强度/限速确认长度无效")
    status,strength,limit=struct.unpack('<BBH',payload)
    if status>2 or strength>2 or not 7<=limit<=147:raise ValueError("强度/限速确认无效")
    return status,strength,limit

def make_pwm_output_limit(percent):
    """构建最终对称占空比上限；范围为０至９０％，分辨率为０．１％。"""
    if isinstance(percent,bool) or not isinstance(percent,(int,float)) or not math.isfinite(percent) or not 0<=percent<=90:
        raise ValueError("PWM总输出上限应为0–90%")
    codes=round(percent*10)
    if abs(percent*10-codes)>1e-6:
        raise ValueError("PWM总输出上限步长为0.1%")
    return make_frame(0x48,struct.pack('<H',codes))

def decode_pwm_output_limit(payload):
    if len(payload)!=3:raise ValueError("PWM总限幅回显长度无效")
    status,codes=struct.unpack('<BH',payload)
    if status>1 or codes>900:raise ValueError("PWM总限幅回显无效")
    return status,codes
