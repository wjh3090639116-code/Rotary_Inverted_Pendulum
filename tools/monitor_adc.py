"""命令行串口监视工具：接收并解码采集、驱动及配置回显数据。"""

import argparse
import serial_dependency
import serial
from adc_protocol import FrameDecoder, decode_adc, decode_motor, decode_config_echo

def frames(port):
    """按 A5 5A TYPE LEN PAYLOAD XOR 格式同步并校验。"""
    decoder = FrameDecoder()
    while True:
        chunk = port.read(min(max(port.in_waiting, 1), 4096))
        yield from decoder.feed(chunk)

def main():
    parser = argparse.ArgumentParser(description="Gowin ADC UART monitor")
    parser.add_argument("port", help="串口名，例如 COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--counts-per-rev", type=int, default=1040,
                        help="编码器输出轴每转 x4 计数，默认 1040")
    args = parser.parse_args()
    if args.counts_per_rev <= 0:
        parser.error("--counts-per-rev 必须为正整数")

    with serial.Serial(args.port, args.baud, timeout=1) as port:
        for frame_type, payload in frames(port):
            if frame_type == 0x34 and len(payload) == 3:
                code, fault = decode_adc(payload)
                if fault:
                    print("ADS8688 高12位 无有效读数（SPI 或采样超时）")
                else:
                    print(f"ADC={code:5d} (0x{code:04X})  {code / 4096 * 5.12:.3f} V")
            elif frame_type == 0x35 and len(payload) in (11, 12, 14, 18):
                position, pendulum_speed, pwm, invalid, flags, reason = decode_motor(payload)
                if not flags & 16:
                    pwm = 0
                angle = position * 360 / args.counts_per_rev
                state = ("故障" if flags & 4 else "PID运行" if flags & 2
                         else "自动起摆" if flags & 64 else "捕获中" if flags & 1 else "停机")
                print(f"电机 {state}  角度={angle:+.2f}°  摆速={pendulum_speed:+d}码/采样  "
                      f"PWM={pwm/10:+.1f}%  编码器={position:+d}  异常跳变={invalid}")
                if flags & 4:
                    print(f"  故障代码={reason}")
            elif frame_type == 0x33 and len(payload) == 30:
                print(f"配置已接受: {decode_config_echo(payload)}")
            else:
                print(f"TYPE=0x{frame_type:02X} DATA={payload.hex(' ')}")

if __name__ == "__main__":
    main()
