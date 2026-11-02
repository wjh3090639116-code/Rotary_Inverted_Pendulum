"""串口采集辅助工具：记录遥测数据，并在明确指定运行参数时执行有边界的采集任务。"""

import argparse
import csv
import json
from pathlib import Path
import time

import serial_dependency
import serial

from adc_protocol import (FrameDecoder, decode_adc, decode_motor, make_config,
                          make_swing_config, make_control_period, make_outer_period,
                          make_wrap_config, make_tilt_limit, make_pwm_deadzone, make_run_command)

from native_profile import migrate_profile

def profile_frames(path):
    data = migrate_profile(json.loads(Path(path).read_text(encoding="utf-8")))
    b, s = data["balance"], data["swing"]
    config = make_config(
        int(b["零点 ADC"]), int(b["角度方向"]), int(b["编码器方向"]),
        int(b["捕获窗口"]), int(b["跌倒门限"]), int(b["行程门限"]),
        *(float(b[name]) for name in ("Kp", "Ki", "Kd", "转臂 Kp", "转臂 Kd")))
    swing = make_swing_config(*(int(s[name]) for name in
        ('pump_pwm','pulse_ms','bottom_code','target_code','upper_code','bottom_window','rearm_distance','travel','stage2_pwm','stage2_ms')))
    period = make_control_period(float(data["period_ms"]))
    wrap = make_wrap_config(int(data["wrap_low"]), int(data["wrap_high"]))
    tilt = make_tilt_limit(int(data.get("tilt_limit", 2)))
    return ((config, 0x33), (wrap, 6), (period, 5), (make_outer_period(float(data.get("outer_period_ms",10))),0x11), (swing, 0x29), (tilt, 8), (make_pwm_deadzone(int(data.get("pwm_deadzone",0))),0x2B))

def record(port_name, seconds, output, run=False, position_limit=300, profile=None):
    decoder = FrameDecoder()
    counts = {"adc": 0, "motor": 0, "invalid": 0, "fault": 0}
    start = time.monotonic()
    last_renewal = start
    reference_position = None
    stop_reason = "time limit"
    with serial.Serial(port_name, 115200, timeout=0.05, write_timeout=0.1) as port, \
            Path(output).open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("seconds", "type", "adc_code", "adc_invalid", "encoder_position",
                         "pendulum_speed", "pwm_per_mille", "encoder_invalid_count",
                         "flags", "fault_reason"))
        try:
            if profile is not None:
                frames = profile_frames(profile)
                port.write(make_run_command(False))
                time.sleep(0.05)
                for frame, _ in frames:
                    port.write(frame)
                expected = {kind: frame[4:-1] for frame, kind in frames}
                deadline = time.monotonic() + 5
                while expected and time.monotonic() < deadline:
                    chunk = port.read(min(max(port.in_waiting, 1), 4096))
                    for frame_type, payload in decoder.feed(chunk):
                        if frame_type in expected and payload == expected[frame_type]:
                            del expected[frame_type]
                if expected:
                    raise RuntimeError(f"FPGA did not echo configuration types {list(expected)}")
                start = time.monotonic()
                last_renewal = start
            if run:
                port.write(make_run_command(True))
            while time.monotonic() - start < seconds:
                now = time.monotonic()
                if run and now-last_renewal >= 0.2:
                    port.write(make_run_command(True))
                    last_renewal = now
                chunk = port.read(min(max(port.in_waiting, 1), 4096))
                stamp = time.monotonic()-start
                for frame_type, payload in decoder.feed(chunk):
                    if frame_type == 0x34 and len(payload) == 3:
                        code, invalid = decode_adc(payload)
                        counts["adc"] += 1
                        counts["invalid"] += invalid
                        writer.writerow((f"{stamp:.6f}", "adc", code, invalid,
                                         "", "", "", "", "", ""))
                        if run and invalid:
                            stop_reason = "ADC invalid"
                            return counts, stop_reason
                    elif frame_type == 0x35 and len(payload) == 18:
                        pos, speed, pwm, invalid, flags, reason = decode_motor(payload)
                        counts["motor"] += 1
                        writer.writerow((f"{stamp:.6f}", "motor", "", "", pos,
                                         speed, pwm, invalid, flags, reason))

                        
                        if run and not (flags & 1 and flags & (2 | 64)):
                            reference_position = None
                        elif reference_position is None:
                            reference_position = pos
                        if run and flags & 4:
                            counts["fault"] += 1
                            stop_reason = f"FPGA fault {reason}"
                            return counts, stop_reason
                        if run and reference_position is not None and \
                                abs(pos-reference_position)>position_limit:
                            stop_reason = "bench position limit"
                            return counts, stop_reason
        finally:
            if run:
                for _ in range(3):
                    port.write(make_run_command(False))
                port.flush()
    return counts, stop_reason

def summarize(path, profile=None):
    with Path(path).open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    adc = [(float(row["seconds"]), int(row["adc_code"])) for row in rows
           if row["type"] == "adc" and row["adc_invalid"] == "0"]
    motor = [row for row in rows if row["type"] == "motor"]
    if not adc or not motor:
        return "insufficient telemetry"
    position = [int(row["encoder_position"]) for row in motor]
    pwm = [int(row["pwm_per_mille"]) for row in motor]
    faults = [int(row["fault_reason"]) for row in motor
              if int(row["flags"]) & 4]
    capture = next((float(row["seconds"]) for row in motor
                    if int(row["flags"]) & 2), None)
    if profile is None:
        zero, span = 1910, 2640
    else:
        data = migrate_profile(json.loads(Path(profile).read_text(encoding="utf-8")))
        zero = int(data["balance"]["零点 ADC"])
        span = int(data["wrap_high"]) - int(data["wrap_low"])
    closest = min(abs((code-zero+span//2) % span-span//2) for _, code in adc)
    peak_from_down = 180 - closest * 360 / span
    return (f"ADC {min(code for _, code in adc)}..{max(code for _, code in adc)}, "
            f"highest {peak_from_down:.1f}° from down, "
            f"arm {min(position)}..{max(position)} counts, "
            f"max PWM {max(abs(value) for value in pwm)}‰, "
            f"PID capture {capture if capture is not None else 'none'} s, "
            f"fault {faults[-1] if faults else 'none'}")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM12")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--output", default="fpga_telemetry.csv")
    parser.add_argument("--run", action="store_true", help="start motor, then stop on timeout or limit")
    parser.add_argument("--position-limit", type=int, default=300)
    parser.add_argument("--profile", help="JSON profile to send and verify before recording")
    args = parser.parse_args()
    if not 0 < args.seconds <= 30 or not 1 <= args.position_limit <= 32767:
        parser.error("seconds must be 0..30 and position limit 1..32767")
    counts, reason = record(args.port, args.seconds, args.output,
                            args.run, args.position_limit, args.profile)
    print(f"{counts} stop={reason} saved={Path(args.output).resolve()}")
    print(summarize(args.output, args.profile))

if __name__ == "__main__":
    main()
