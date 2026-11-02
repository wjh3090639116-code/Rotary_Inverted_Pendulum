"""串口上位机主程序：集中处理设备连接、参数配置、真实数据展示和运行日志。"""

from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from adc_protocol import (FrameDecoder, decode_adc, decode_motor, make_config,
                          make_swing_config, make_run_command, make_control_period, make_outer_period,
                          make_wrap_config, make_query_command,
                          make_tilt_limit, make_live_pid, decode_live_pid, make_live_pd, decode_live_pd, make_pwm_deadzone, decode_swing_echo, make_arm_target,
                          shifted_arm_target, decode_arm_target_echo)

from native_profile import migrate_profile, clean_history
from scroll_panel import ScrollPanel
from angle_fullscreen import AngleFullscreen, clipped_trace_runs, user_degrees, reference_segments
from adc_protocol import turn_tilt_limit

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dependencies" / "pyserial.zip"))
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        serial = None
        list_ports = None

class TracePlot:
    """带时间轴与物理单位的曲线控件，密集样本按像素列归并以保留极值。"""

    def __init__(self, parent, title, color, unit, fixed_range=None):
        self.frame = ttk.LabelFrame(parent, text=title, padding=5)
        self.canvas = tk.Canvas(self.frame, height=205, bg="white",
                                highlightthickness=1, highlightbackground="#d7e0ea")
        self.canvas.pack(fill="both", expand=True)
        self.color = color
        self.unit = unit
        self.fixed_range = fixed_range
        self.clip_values = False
        self.grid_divisions = 4
        self.tick_font = ("Segoe UI", 10)
        self.minimum_span = 0
        self.show_reference_label = True

    def redraw(self, samples, span, now, fixed_range=None, *, reference=None, show_zero=True, reference_history=None, reference_label="用户目标"):
        canvas = self.canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 180 or height < 100:
            return
        canvas.delete("all")
        x0, x1, y0, y1 = 64, width - 14, 12, height - 27
        start = now - span
        visible = [(t, value) for t, value in samples if start <= t <= now]
        references = reference_segments(reference_history, start, now) if reference_history is not None else []
        bounds = fixed_range or self.fixed_range
        if bounds is None:
            values = [value for _, value in visible]
            values.extend(value for _, _, value, _ in references)
            if reference is not None:values.append(reference)
            if values:
                low, high = (min(values),max(values)) if reference is not None or references else (min(min(values),0),max(max(values),0))
                pad = max((high - low) * 0.12, 0.5)
                low, high = low - pad, high + pad
            else:
                low, high = -1.0, 1.0
        else:
            low, high = bounds
        if bounds is None and high-low < self.minimum_span:
            middle = (low+high)/2
            low, high = middle-self.minimum_span/2, middle+self.minimum_span/2
        if high <= low:
            high = low + 1

        def yy(value):
            return y1 - (value - low) / (high - low) * (y1 - y0)

        for step in range(self.grid_divisions + 1):
            fraction = step / self.grid_divisions
            y = y1 - fraction * (y1 - y0)
            canvas.create_line(x0, y, x1, y, fill="#e8edf3")
            canvas.create_text(x0 - 6, y, anchor="e", fill="#536578",
                               text=f"{low + fraction * (high-low):.1f}", font=self.tick_font)
        for step in range(5):
            fraction = step / 4
            x = x0 + fraction * (x1 - x0)
            canvas.create_line(x, y0, x, y1, fill="#eef2f6")
            canvas.create_text(x, y1 + 15, fill="#536578",
                               text=f"{-span + fraction * span:g}s", font=self.tick_font)
        if show_zero and reference is None and low < 0 < high:
            canvas.create_line(x0, yy(0), x1, yy(0), fill="#9ba9ba", dash=(3, 3))
        if reference_history is None and reference is not None:
            references = [(start, now, reference, reference_label)]
        previous = None
        for a, b, value, label in references:
            ax = x0 + (a-start)*(x1-x0)/span
            bx = x0 + (b-start)*(x1-x0)/span
            if low <= value <= high:
                canvas.create_line(ax, yy(value), bx, yy(value), fill="#2f6fa5", dash=(7,4), width=2, tags="user_target")
            
            if previous is not None and previous[1] == a and previous[3] == label and previous[2] != value:
                for run in clipped_trace_runs([ax, yy(previous[2]), ax, yy(value)], y0, y1):
                    canvas.create_line(*run, fill="#2f6fa5", dash=(7,4), width=2, tags="user_target")
            previous = (a, b, value, label)
        if reference is not None and low<=reference<=high and self.show_reference_label:
            canvas.create_text(x1-5,max(y0+10,min(y1-8,yy(reference)-9)),anchor="e",fill="#2f6fa5",
                               text=f"{reference_label} {reference:+.0f}°",tags="user_target_label")
        canvas.create_text(x0 + 4, y0 + 3, anchor="nw", fill="#536578", text=self.unit)
        canvas.create_rectangle(x0, y0, x1, y1, outline="#adb9c6")
        if not visible:
            canvas.create_text((x0+x1)/2, (y0+y1)/2, text="等待数据…", fill="#8191a2")
            return

        buckets = {}
        for timestamp, value in visible:
            x = round(x0 + (timestamp - start) * (x1 - x0) / span)
            if x in buckets:
                minimum, maximum, _ = buckets[x]
                buckets[x] = min(minimum, value), max(maximum, value), value
            else:
                buckets[x] = value, value, value
        points = []
        for x in sorted(buckets):
            minimum, maximum, last = buckets[x]
            if maximum != minimum:
                if not self.clip_values or minimum <= high and maximum >= low:
                    bottom_value = max(low, minimum) if self.clip_values else minimum
                    top_value = min(high, maximum) if self.clip_values else maximum
                    canvas.create_line(x, yy(bottom_value), x, yy(top_value), fill=self.color, width=1, tags="trace")
            points.extend((x, yy(last)))
        if len(points) >= 4:
            for run in clipped_trace_runs(points, y0, y1) if self.clip_values else [points]:
                canvas.create_line(*run, fill=self.color, width=2, tags="trace")
        elif points and (not self.clip_values or y0 <= points[1] <= y1):
            canvas.create_oval(points[0]-2, points[1]-2, points[0]+2, points[1]+2,
                               fill=self.color, outline=self.color, tags="trace")

from adc_protocol import decode_touch_state,make_target_trim,decode_target_trim,make_board_config,decode_board_echo,step_tilt_limit,decode_monitor,trim_runtime_text,make_monitor_subscription,make_reciprocal_command,make_board_handoff,make_response_speed,decode_response_speed,make_correction_tuning,decode_correction_tuning,make_pwm_output_limit,decode_pwm_output_limit

class PendulumWindow:
    ADC_REPORT_HZ = 50
    MOTOR_REPORT_HZ = 200
    FAULT_REASONS = {
        1: "ADC 无有效读数（检查 SPI、供电和采样超时）",
        2: "旧版固件的 ADC 超量程故障，请烧录新版",
        3: "旧版固件将换圈端点误判为故障，请烧录新版",
        4: "电机转臂超过行程门限",
        5: "第一阶段 20 秒未达到目标",
        6: "PID 运行时摆杆超过跌倒门限",
        7: "控制状态异常",
        255: "旧版码流未提供故障原因，请重新生成并烧录",
    }

    def __init__(self, root):
        self.root = root
        root.title("旋转倒立摆控制系统 · 逻辑派 G1 / GW2A-LV18")
        root.geometry("1600x900")
        root.minsize(1200, 700)
        try:
            root.state("zoomed")  
        except tk.TclError:
            pass
        self.port_name = tk.StringVar()
        self.baud_text = tk.StringVar(value="115200")
        self.full_scale_text = tk.StringVar(value="5.12")
        self.counts_text = tk.StringVar(value="1040")
        self.window_text = tk.StringVar(value="10 秒")
        self.status_text = tk.StringVar(value="未连接")
        self.rate_text = tk.StringVar(value="ADC --/50 Hz · 电机 --/200 Hz")
        self.state_text = tk.StringVar(value="等待电机状态")
        self.fault_text = tk.StringVar(value="")
        self.config_text = tk.StringVar(value="尚未收到 FPGA 参数回显")
        self.zero_text = tk.StringVar(value="FPGA 电位器起点：未设置")
        self.adc_value = tk.StringVar(value="--.--- V")
        self.adc_code_value = tk.StringVar(value="ADC ----")
        self.motor_angle_value = tk.StringVar(value="--.--°")
        self.pwm_value = tk.StringVar(value="--.- %")
        self.target_tilt_value = tk.StringVar(value="目标摆杆倾角：--")
        self.arm_step_text = tk.StringVar(value="10")
        self.arm_target_text = tk.StringVar(value="转臂目标：等待 FPGA 回显")
        self.arm_target_status = tk.StringVar(value="直立运行后可调整目标角")
        self.arm_target_count = 0
        self.arm_target_confirmed = False
        self.wide_arm_firmware = False
        self.pending_arm_is_turn=False
        self.pending_arm_target = None
        self.arm_target_deadline = 0.0
        self.motor_count_value = tk.StringVar(value="编码器 -- 计数")
        self.parameter_defaults = {
            "零点 ADC": "1910", "角度方向": "1", "编码器方向": "-1",
            "捕获窗口": "61", "跌倒门限": "301", "行程门限": "2000",
            "Kp": "0", "Ki": "0", "Kd": "0", "转臂 Kp": "0", "转臂 Kd": "0",
        }
        self.parameters = {name: tk.StringVar(value=value)
                           for name, value in self.parameter_defaults.items()}
        self.gain_step = tk.StringVar(value="0.10")
        self.live_pid_enabled=tk.BooleanVar(value=False)
        self.live_pid_status=tk.StringVar(value="实时 PID：先连接并确认新版固件")
        self.live_pid_supported=False
        self.live_pid_pending=None
        self.live_pid_desired=None
        self.live_pid_sent_at=0.0
        self.live_pid_next_at=0.0
        self.live_pid_syncing=False
        self.live_pid_sliders={}
        self.live_pid_bounds={}
        self.live_pid_ranges={}
        self.live_pid_confirmed=None
        self.live_pd_enabled=tk.BooleanVar(value=False)
        self.live_pd_status=tk.StringVar(value="实时外环 PD：先连接并确认新版固件")
        self.live_pd_supported=False
        self.live_pd_pending=None
        self.live_pd_desired=None
        self.live_pd_sent_at=0.0
        self.live_pd_next_at=0.0
        self.live_pd_syncing=False
        self.live_pd_sliders={}
        self.live_pd_bounds={}
        self.live_pd_ranges={}
        self.live_pd_confirmed=None
        self.full_scale = 5.12
        self.counts_per_rev = 1040
        self.latest_code = None
        self.latest_adc_at = 0.0
        self.latest_motor = None
        self.configured_zero = None
        self.configured_travel = None
        self.configured_wrap_low = 0
        self.configured_wrap_high = 2640
        self.pending_config = None
        self.pending_pwm_limit=None;self.pwm_limit_deadline=0.0
        self.pending_tuning=None;self.tuning_deadline=0.0
        self.pending_response=None;self.response_deadline=0.0
        self.pending_tolerance=None
        self.tolerance_deadline=0.0
        self.pending_board=None
        self.board_deadline=0.0
        self.pending_arm_tilt=None
        self.latest_monitor=None
        self.latest_monitor_at=0.0
        self.monitor_decode_error=""
        self.monitor_request_at=0.0
        self.pending_swing = None
        self.pending_period = None
        self.pending_outer_period = None
        self.pending_wrap = None
        self.pending_tilt = None
        self.pending_deadzone = None
        self.remote_run_requested = False
        self.touch_local_run = False
        self.auto_start_pending = False
        self.auto_start_deadline = 0.0
        self.manual_start = False
        self.log_file = None
        self.log_writer = None
        self.log_metadata_path = None
        self.log_started = 0.0
        self.log_text = tk.StringVar(value="运行记录：未开始")
        self.tolerance_text=tk.StringVar(value="2.0")
        self.tolerance_status=tk.StringVar(value="等待FPGA补偿配置回显")
        self.response_mode_text=tk.StringVar(value="连续")
        self.response_status=tk.StringVar(value="连续档：强度和限速需单独确认")
        self.correction_strength_text=tk.StringVar(value="柔和")
        self.correction_rate_text=tk.StringVar(value="3.0")
        self.pwm_limit_text=tk.StringVar(value="90.0")
        self.pwm_limit_status=tk.StringVar(value="总输出上限：等待FPGA确认")
        self.correction_tuning_status=tk.StringVar(value="默认柔和，约3°/秒；仅连续档生效")
        self.target_trim_enabled=tk.BooleanVar(value=False)
        self.target_trim_max_text=tk.StringVar(value="60")
        self.board_amplitude_text=tk.StringVar(value="60")
        self.board_interval_text=tk.StringVar(value="2")
        self.small_step_limit_text=tk.StringVar(value="20")
        self.large_step_limit_text=tk.StringVar(value="40")
        self.reverse_turn_limit_text=tk.StringVar(value="40")
        self.triplet_settings_confirmed=False;self.confirmed_tilt_profiles=None;self.last_tilt_phase_logged=None
        self.tilt_profile_status=tk.StringVar(value="三种倾角限幅：等待FPGA确认")
        self.board_status_text=tk.StringVar(value="往复由屏幕或本页按钮开关")
        self.trim_runtime_text=tk.StringVar(value="补偿状态：等待监测")
        self.three_angles_text=tk.StringVar(value="用户 / 内部 / 实际：等待FPGA")
        self.user_angle_value=tk.StringVar(value="—")
        self.actual_angle_value=tk.StringVar(value="—")
        self.internal_angle_value=tk.StringVar(value="—")
        self.position_error_value=tk.StringVar(value="—")
        self.monitor_age_text=tk.StringVar(value="等待内部目标回传")
        self.disconnecting=False;self.close_pending=False;self.handoff_result=None
        self.profile_loaded = False
        self.swing_pump_text = tk.StringVar(value="250")
        self.swing_duration_text = tk.StringVar(value="20")
        self.swing_bottom_text = tk.StringVar(value="2560")
        self.swing_target_code_text = tk.StringVar(value="1808")
        self.swing_upper_text = tk.StringVar(value="1168")
        self.swing_window_text = tk.StringVar(value="32")
        self.swing_rearm_text = tk.StringVar(value="160")
        self.swing_target_text = tk.StringVar(value="160")
        self.swing_stage2_pwm_text = tk.StringVar(value="200")
        self.swing_stage2_ms_text = tk.StringVar(value="20")
        self.swing_text = tk.StringVar(value="两阶段起摆：等待 FPGA 参数回显")
        self.pendulum_speed = 0
        self.period_ms_text = tk.StringVar(value="1.0")
        self.outer_period_ms_text = tk.StringVar(value="10.0")
        self.outer_period_text = tk.StringVar(value="外环复位默认 10 ms（100 Hz）")
        self.confirmed_outer_period_us = 10000
        self.period_text = tk.StringVar(value="内环固定 1 ms（1000 Hz）")
        self.wrap_low_text = tk.StringVar(value="0")
        self.wrap_high_text = tk.StringVar(value="2640")
        self.wrap_text = tk.StringVar(value="换圈端点：FPGA 默认低 0 / 高 2640，建议实测后发送")
        self.deadzone_text = tk.StringVar(value="0")
        self.deadzone_status = tk.StringVar(value="PWM 死区补偿：等待 FPGA 回显")
        self.tilt_limit_text = tk.StringVar(value="2")
        self.tilt_text = tk.StringVar(value="目标倾角上限：等待 FPGA 回显")
        self.adc_samples = deque(maxlen=10000)  
        self.angle_reference_history = deque(maxlen=10000)
        self.reference_reciprocal = False
        self.reference_center_degrees = None
        self.motor_samples = deque(maxlen=10000)  
        self._angle_fullscreen = None
        self.events = queue.Queue()
        self.write_queue = queue.Queue()
        self.stop_event = None
        self.session = 0
        self.connected = False
        self.rate_started = time.monotonic()
        self.adc_since_rate = 0
        self.motor_since_rate = 0
        removed,errors=0,[] 
        self.log_text.set("历史运行记录已保留")
        self._load_profile()
        self._make_widgets()
        for gain in ("Kp","Ki","Kd","转臂 Kp","转臂 Kd"):
            self.parameters[gain].trace_add("write",lambda *args,key=gain:self._live_gain_edited(key))
        self.refresh_ports()
        root.after(50, self.poll_events)
        root.after(100, self.redraw_plots)
        root.after(1000, self.update_rates)
        root.after(250, self.renew_remote_run)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def _make_widgets(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        connection = ttk.Frame(outer)
        connection.pack(fill="x", pady=(0, 8))
        
        ttk.Button(connection, text="保存当前参数", command=self.save_profile)\
            .pack(side="right", padx=(10, 0))
        ttk.Label(connection, text="串口").pack(side="left")
        self.port_box = ttk.Combobox(connection, textvariable=self.port_name, width=12)
        self.port_box.pack(side="left", padx=5)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).pack(side="left")
        ttk.Label(connection, text="波特率").pack(side="left", padx=(18, 5))
        ttk.Entry(connection, textvariable=self.baud_text, width=9).pack(side="left")
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=10)
        ttk.Label(connection, textvariable=self.status_text).pack(side="left", padx=10)

        status = ttk.Frame(outer)
        status.pack(fill="x", pady=(0, 8))
        ttk.Label(status, textvariable=self.state_text, font=("Segoe UI", 11, "bold"))\
            .pack(side="left")
        ttk.Label(status, textvariable=self.rate_text).pack(side="right")
        ttk.Label(outer, textvariable=self.fault_text, foreground="#b42318",
                  font=("Segoe UI", 11, "bold"), wraplength=1400)\
            .pack(fill="x", pady=(0, 8))

        content = ttk.Frame(outer)
        content.pack(fill="both", expand=True)
        content.columnconfigure(2, weight=1)
        content.rowconfigure(0, weight=1)

        monitor_panel = ScrollPanel(content, width=340)
        monitor_panel.grid(row=0, column=0, sticky="ns")
        monitor = monitor_panel.body
        settings = ttk.Frame(content, width=330, padding=(0, 0, 8, 0))
        settings.grid(row=0, column=1, sticky="ns")
        settings.pack_propagate(False)
        settings_tabs = ttk.Notebook(settings)
        settings_tabs.pack(fill="both", expand=True)
        pid_panel,period_panel,swing_panel,arm_panel = (ScrollPanel(settings_tabs) for _ in range(4))
        settings_tabs.add(pid_panel,text="直立 PID")
        settings_tabs.add(period_panel,text="周期与限幅")
        settings_tabs.add(swing_panel,text="自动起摆")
        settings_tabs.add(arm_panel,text="转臂目标")
        pid_settings,timing_settings,swing_settings,arm_settings = (x.body for x in (pid_panel,period_panel,swing_panel,arm_panel))
        arm_box = ttk.LabelFrame(arm_settings, text="转臂目标角增减", padding=8)
        arm_box.pack(fill="x")
        ttk.Label(arm_box, textvariable=self.arm_target_text, wraplength=285,
                  font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(0, 8))
        ttk.Label(arm_box, text="每次增减角度（0–360°，不含 0）").pack(anchor="w")
        ttk.Entry(arm_box, textvariable=self.arm_step_text, width=12).pack(anchor="w", pady=5)
        arm_buttons = ttk.Frame(arm_box)
        arm_buttons.pack(fill="x", pady=5)
        self.arm_minus_button = ttk.Button(arm_buttons, text="− 目标角",
                                           command=lambda: self.shift_arm_target(-1))
        self.arm_plus_button = ttk.Button(arm_buttons, text="＋ 目标角",
                                          command=lambda: self.shift_arm_target(1))
        self.arm_minus_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.arm_plus_button.pack(side="left", fill="x", expand=True)
        self.arm_home_button = ttk.Button(arm_box, text="目标回到启动中心（0°）",
                                         command=lambda: self.send_arm_target(0))
        self.arm_home_button.pack(fill="x", pady=5)
        self.arm_target_buttons = (self.arm_minus_button, self.arm_plus_button, self.arm_home_button)
        for button in self.arm_target_buttons:
            button.configure(state="disabled")
        ttk.Label(arm_box, text="按加减方向累计：＋270° 就正向转 270°；\n"
                               "＋360° 就正向一整圈。启动位置为 0°。\n"
                               "量化到编码器计数；仍受硬行程限制。",
                  wraplength=285, foreground="#596b7d").pack(anchor="w", pady=8)
        ttk.Label(arm_box, textvariable=self.arm_target_status, wraplength=285).pack(anchor="w")
        readings=ttk.LabelFrame(arm_settings,text="转臂位置 · 编码器反馈",padding=8)
        readings.pack(fill="x",pady=8)
        readings.columnconfigure(1,weight=1)
        for row,(name,var) in enumerate((("用户设定目标",self.user_angle_value),("编码器实测",self.actual_angle_value),("目标−实测误差",self.position_error_value))):
            ttk.Label(readings,text=name).grid(row=row,column=0,sticky="w",pady=4)
            ttk.Label(readings,textvariable=var,font=("Segoe UI",14,"bold"),anchor="e").grid(row=row,column=1,sticky="e",padx=(6,0))
        ttk.Separator(readings).grid(row=3,column=0,columnspan=2,sticky="ew",pady=7)
        ttk.Label(readings,text="内部控制目标").grid(row=4,column=0,sticky="w")
        ttk.Label(readings,textvariable=self.internal_angle_value,font=("Segoe UI",11)).grid(row=4,column=1,sticky="e")
        ttk.Label(readings,text="内部目标仅供控制计算",foreground="#596b7d").grid(row=5,column=0,columnspan=2,sticky="w",pady=3)
        ttk.Label(readings,textvariable=self.monitor_age_text,wraplength=270,foreground="#596b7d").grid(row=6,column=0,columnspan=2,sticky="w")
        ttk.Label(readings,textvariable=self.trim_runtime_text,wraplength=270,justify="left").grid(row=7,column=0,columnspan=2,sticky="w",pady=5)
        tol=ttk.LabelFrame(arm_settings,text="稳态目标误差补偿",padding=8)
        tol.pack(fill="x",pady=8)
        ttk.Label(tol,text="补偿响应速度").pack(anchor="w")
        ttk.Combobox(tol,textvariable=self.response_mode_text,values=("原速","连续"),state="readonly",width=12).pack(anchor="w",pady=3)
        ttk.Button(tol,text="发送响应速度",command=self.send_response_speed).pack(fill="x",pady=3)
        ttk.Label(tol,textvariable=self.response_status,wraplength=270,justify="left").pack(anchor="w",pady=4)
        ttk.Label(tol,text="连续档补偿强度").pack(anchor="w")
        ttk.Combobox(tol,textvariable=self.correction_strength_text,values=("柔和","适中","较强"),state="readonly",width=12).pack(anchor="w",pady=3)
        ttk.Label(tol,text="内部目标修正上限（0.5–10°/秒）").pack(anchor="w")
        ttk.Entry(tol,textvariable=self.correction_rate_text,width=12).pack(anchor="w",pady=3)
        ttk.Button(tol,text="发送强度与限速",command=self.send_correction_tuning).pack(fill="x",pady=3)
        ttk.Label(tol,textvariable=self.correction_tuning_status,wraplength=270,justify="left").pack(anchor="w",pady=4)
        ttk.Label(tol,text="只限制补偿偏置变化，不限制目标步进本身。先保持补偿关闭；原速仅对照，不受此处限速。",wraplength=270,foreground="#8c4d1d").pack(anchor="w",pady=3)
        ttk.Checkbutton(tol,text="自动修正内部目标",variable=self.target_trim_enabled).pack(anchor="w")
        for label,var in (("允许用户目标误差（±度，0–180）",self.tolerance_text),("最大内部补偿（±度，停机设置）",self.target_trim_max_text)):
            ttk.Label(tol,text=label).pack(anchor="w")
            ttk.Entry(tol,textvariable=var,width=12).pack(anchor="w",pady=3)
        ttk.Button(tol,text="发送容差设置",command=self.send_position_tolerance).pack(fill="x",pady=4)
        ttk.Label(tol,textvariable=self.tolerance_status,wraplength=275).pack(anchor="w")
        box=ttk.LabelFrame(arm_settings,text="倾角限幅 / 对称往复",padding=8)
        box.pack(fill="x",pady=8)
        for label,var in (("平时倾角上限（稳定保持值，1–64码）",self.small_step_limit_text),("正转一圈倾角上限（1–64码）",self.large_step_limit_text),("反转一圈倾角上限（1–64码）",self.reverse_turn_limit_text),("往复单边幅度（度）",self.board_amplitude_text),("目标切换间隔（秒，0.1–600）",self.board_interval_text)):
            ttk.Label(box,text=label,wraplength=275).pack(anchor="w")
            ttk.Entry(box,textvariable=var,width=12).pack(anchor="w",pady=3)
        ttk.Button(box,text="发送三种倾角及往复设置",command=self.send_board_config).pack(fill="x",pady=4)
        row=ttk.Frame(box);row.pack(fill="x")
        ttk.Button(row,text="开启往复",command=lambda:self.set_reciprocal(True)).pack(side="left",expand=True,fill="x")
        ttk.Button(row,text="关闭往复",command=lambda:self.set_reciprocal(False)).pack(side="left",expand=True,fill="x")
        ttk.Label(box,textvariable=self.board_status_text,wraplength=275).pack(anchor="w",pady=4)
        ttk.Label(box,textvariable=self.tilt_profile_status,wraplength=275).pack(anchor="w",pady=4)
        ttk.Label(box,text="起摆接管或摆杆受扰后自动使用至少40码的恢复档，稳定后回平时值。",wraplength=275,foreground="#596b7d").pack(anchor="w",pady=3)
        ttk.Label(box,text="以开启时用户目标为中心；首次先等待一个间隔。关闭保持末次目标。新人工目标重设中心和计时。往复按2×单边幅度选择限幅。",wraplength=275).pack(anchor="w")
        plots = ttk.Frame(content)
        plots.grid(row=0, column=2, sticky="nsew")

        self._card(monitor, "摆杆电位器", self.adc_value, self.adc_code_value)
        self._card(monitor, "电机转臂角度", self.motor_angle_value, self.motor_count_value)
        self._card(monitor, "电机 PWM 指令", self.pwm_value, None)
        ttk.Label(monitor, textvariable=self.target_tilt_value).pack(anchor="w")

        quick = ttk.LabelFrame(monitor, text="快速操作", padding=8)
        quick.pack(fill="x", pady=(8, 0))
        controls = ttk.Frame(quick)
        controls.pack(fill="x", pady=(0, 8))
        ttk.Button(controls, text="自动起摆", command=self.start_system)\
            .pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(controls, text="停机", command=self.stop_system)\
            .pack(side="left", fill="x", expand=True)
        ttk.Button(quick,text="手动起摆（扶起后 PID 接管）",command=lambda:self.start_system(manual=True)).pack(fill="x",pady=(0,5))
        ttk.Button(quick, text="当前电位器位置设为起点并发送", command=self.set_pot_zero)\
            .pack(fill="x")
        ttk.Label(quick, textvariable=self.zero_text, wraplength=250).pack(anchor="w", pady=(4, 8))
        ttk.Label(quick, text="编码器每转 x4 计数").pack(anchor="w")
        encoder_row = ttk.Frame(quick)
        encoder_row.pack(fill="x", pady=(2, 8))
        ttk.Entry(encoder_row, textvariable=self.counts_text, width=10).pack(side="left")
        ttk.Button(encoder_row, text="应用角度换算", command=self.apply_counts)\
            .pack(side="left", padx=5)
        ttk.Label(quick, text="ADC 显示满量程 (V)").pack(anchor="w")
        display_row = ttk.Frame(quick)
        display_row.pack(fill="x", pady=(2, 8))
        ttk.Entry(display_row, textvariable=self.full_scale_text, width=10).pack(side="left")
        ttk.Button(display_row, text="应用显示换算",
                   command=lambda: self.apply_full_scale(show_error=True))\
            .pack(side="left", padx=5)
        ttk.Label(quick, text="波形时间范围").pack(anchor="w")
        plot_row = ttk.Frame(quick)
        plot_row.pack(fill="x", pady=(2, 0))
        ttk.Combobox(plot_row, textvariable=self.window_text, state="readonly",
                     values=("2 秒", "5 秒", "10 秒", "30 秒"), width=8)\
            .pack(side="left")
        ttk.Button(plot_row, text="清空波形", command=self.clear_samples)\
            .pack(side="left", padx=5)
        ttk.Label(quick, textvariable=self.log_text, wraplength=310)\
            .pack(anchor="w", pady=(6, 0))

        safety = ttk.LabelFrame(pid_settings, text="零点、方向与保护", padding=8)
        safety.pack(fill="x")
        rows = (
            ("零点 ADC", "12 位码"),
            ("角度方向", "±1"),
            ("编码器方向", "±1"),
            ("捕获窗口", "ADC 码差"),
            ("跌倒门限", "ADC 码差"),
            ("行程门限", "x4 计数"),
        )
        for index, (label, unit) in enumerate(rows):
            ttk.Label(safety, text=label, width=11).grid(row=index, column=0, sticky="w", pady=3)
            ttk.Entry(safety, textvariable=self.parameters[label], width=12)\
                .grid(row=index, column=1, sticky="w", padx=4, pady=3)
            ttk.Label(safety, text=unit, foreground="#596b7d")\
                .grid(row=index, column=2, sticky="w", pady=3)

        wrap = ttk.LabelFrame(monitor, text="360° 电位器换圈端点", padding=8)
        wrap.pack(fill="x", pady=(10, 0))
        for row, (label, variable, side) in enumerate((("跳变后低码", self.wrap_low_text, "low"),
                                                      ("跳变前高码", self.wrap_high_text, "high"))):
            ttk.Label(wrap, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(wrap, textvariable=variable, width=10).grid(row=row, column=1, padx=4)
            ttk.Button(wrap, text="取当前 ADC", command=lambda key=side: self.capture_wrap_endpoint(key))\
                .grid(row=row, column=2, pady=2)
        ttk.Button(wrap, text="发送换圈端点（停机时）", command=self.send_wrap_config)\
            .grid(row=2, column=0, columnspan=3, sticky="ew", pady=4)
        ttk.Label(wrap, textvariable=self.wrap_text, wraplength=300)\
            .grid(row=3, column=0, columnspan=3, sticky="w")

        gains = ttk.LabelFrame(pid_settings, text="直立串级控制增益", padding=8)
        gains.pack(fill="x", pady=(10, 0))
        ttk.Label(gains, text="▲/▼ 步长").grid(row=0, column=0, sticky="w", pady=5)
        ttk.Combobox(gains, textvariable=self.gain_step, state="readonly", width=7,
                     values=("0.000244140625", "0.001", "0.01", "0.05", "0.10", "0.50", "1.00"))\
            .grid(row=0, column=1, columnspan=3, sticky="w", pady=5)
        for index, label in enumerate(("Kp", "Ki", "Kd", "转臂 Kp", "转臂 Kd"), start=1):
            ttk.Label(gains, text=label, width=11).grid(row=index, column=0, sticky="w", pady=4)
            ttk.Entry(gains, textvariable=self.parameters[label], width=11)\
                .grid(row=index, column=1, sticky="w", pady=4)
            ttk.Button(gains, text="▲", width=2,
                       command=lambda key=label: self.adjust_gain(key, 1))\
                .grid(row=index, column=2, padx=(4, 0), pady=4)
            ttk.Button(gains, text="▼", width=2,
                       command=lambda key=label: self.adjust_gain(key, -1))\
                .grid(row=index, column=3, pady=4)
        ttk.Label(gains, text="转臂 P/D 输出目标摆杆 ADC 码；可用负值改方向。\n"
                          "内环 Kd 范围 ±8192；ADC 参数全部为原生码。",
                  foreground="#596b7d",wraplength=265).grid(row=6, column=0, columnspan=4,
                                               sticky="w", pady=(5, 0))
        live=ttk.LabelFrame(pid_settings,text="运动中调节内环 PID",padding=8)
        live.pack(fill="x",pady=(10,0));live.columnconfigure(1,weight=1)
        ttk.Checkbutton(live,text="开启运行中实时调节",variable=self.live_pid_enabled,
                        command=self._toggle_live_pid).grid(row=0,column=0,columnspan=4,sticky="w")
        for index,key in enumerate(("Kp","Ki","Kd")):
            row=1+index*3
            center=float(self.parameters[key].get());limit={"Kp":512,"Ki":256,"Kd":8192}[key]
            span={"Kp":5,"Ki":0.25,"Kd":128}[key]
            low=max(-limit,center-span);high=min(limit,center+span)
            if center>=0:low=max(0,low)
            elif center<0:high=min(0,high)
            if low>=high:low,high=-limit,limit
            self.live_pid_bounds[key]=(low,high)
            ttk.Label(live,text=key).grid(row=row,column=0,sticky="w",pady=(7,0))
            ttk.Entry(live,textvariable=self.parameters[key],width=12).grid(row=row,column=1,columnspan=3,sticky="w",pady=(7,0))
            slider=ttk.Scale(live,from_=low,to=high,value=center,
                             command=lambda value,k=key:self._slide_live_gain(k,value))
            slider.grid(row=row+1,column=0,columnspan=4,sticky="ew",pady=3)
            self.live_pid_sliders[key]=slider
            lo=tk.StringVar(value=f"{low:g}");hi=tk.StringVar(value=f"{high:g}");self.live_pid_ranges[key]=(lo,hi)
            ttk.Label(live,text="范围").grid(row=row+2,column=0,sticky="w")
            ttk.Entry(live,textvariable=lo,width=7).grid(row=row+2,column=1,sticky="w")
            ttk.Entry(live,textvariable=hi,width=7).grid(row=row+2,column=2,sticky="w",padx=3)
            ttk.Button(live,text="应用",width=5,command=lambda k=key:self._apply_live_range(k)).grid(row=row+2,column=3,sticky="e")
        ttk.Label(live,text="PID 接管后开启；滑动条、输入框与上下按钮同步。\n开启后只发送内环增益，不停机；范围可自行缩小微调。",
                  wraplength=270,foreground="#596b7d").grid(row=10,column=0,columnspan=4,sticky="w",pady=(8,3))
        ttk.Label(live,textvariable=self.live_pid_status,wraplength=270).grid(row=11,column=0,columnspan=4,sticky="w")
        self._make_live_outer_widgets(pid_settings)
        cap=ttk.LabelFrame(timing_settings,text="PWM 总输出限幅",padding=8)
        cap.pack(fill="x",pady=(10,0))
        ttk.Label(cap,text="最大绝对输出（0–90%，步长0.1%）").pack(anchor="w")
        ttk.Entry(cap,textvariable=self.pwm_limit_text,width=10).pack(anchor="w",pady=4)
        ttk.Button(cap,text="发送PWM总限幅（可运行中调整）",command=self.send_pwm_limit).pack(fill="x",pady=4)
        ttk.Label(cap,text="正反共同限幅，包含死区补偿，作用于两段起摆和直立控制。0%输出为零。",wraplength=270,foreground="#596b7d").pack(anchor="w")
        ttk.Label(cap,textvariable=self.pwm_limit_status,wraplength=270).pack(anchor="w",pady=4)
        dead=ttk.LabelFrame(timing_settings,text="最终 PWM 死区补偿",padding=8)
        dead.pack(fill="x",pady=(10,0))
        ttk.Label(dead,text="补偿值 (0–300‰)").grid(row=0,column=0,sticky="w")
        ttk.Entry(dead,textvariable=self.deadzone_text,width=9).grid(row=0,column=1,padx=5)
        ttk.Label(dead,text="100‰ = 10%；非零指令按方向叠加，0 保持 0，最终服从PWM总输出上限。用于两段起摆及直立 PID。",wraplength=270,foreground="#596b7d").grid(row=1,column=0,columnspan=2,sticky="w",pady=4)
        ttk.Button(dead,text="发送死区补偿（停机时）",command=self.send_deadzone).grid(row=2,column=0,columnspan=2,sticky="ew",pady=4)
        ttk.Label(dead,textvariable=self.deadzone_status,wraplength=270).grid(row=3,column=0,columnspan=2,sticky="w")
        period=ttk.LabelFrame(timing_settings,text="内环与 ADC 采样",padding=8)
        period.pack(fill="x",pady=(10,0))
        ttk.Label(period,text="内环固定 1 ms（1000 Hz）\nFPGA 读取固定 1000 Hz\nADS8688 SPI 转换 1000 SPS\n仅使用 raw16[15:4]",wraplength=270).pack(anchor="w")
        ttk.Label(period,textvariable=self.period_text,wraplength=270).pack(anchor="w",pady=4)
        outer=ttk.LabelFrame(timing_settings,text="位置外环计算周期",padding=8)
        outer.pack(fill="x",pady=(10,0))
        ttk.Label(outer,text="周期 1–50 ms；步长 0.1").grid(row=0,column=0,sticky="w")
        ttk.Entry(outer,textvariable=self.outer_period_ms_text,width=8).grid(row=0,column=1,padx=3)
        ttk.Button(outer,text="发送外环周期（停机时）",command=self.send_outer_period).grid(row=1,column=0,columnspan=2,sticky="ew",pady=5)
        ttk.Label(outer,textvariable=self.outer_period_text,wraplength=270).grid(row=2,column=0,columnspan=2,sticky="w")
        swing = ttk.LabelFrame(swing_settings, text="两阶段：最低点反向脉冲 → 直立 PID", padding=8)
        swing.pack(fill="x", pady=(10, 0))
        swing_fields = (
            ("第一段 PWM (±1–900‰)", self.swing_pump_text),
            ("第一段每次时长 (1–1000 ms)", self.swing_duration_text),
            ("最低点 ADC", self.swing_bottom_text),
            ("第一阶段目标 ADC", self.swing_target_code_text),
            ("最高点 ADC（判定侧边界）", self.swing_upper_text),
            ("最低点判定窗口 (5–40 码)", self.swing_window_text),
            ("再次触发前离开 (80–300 码)", self.swing_rearm_text),
            ("转臂软行程 (5–32767 计数)", self.swing_target_text),
            ("第二段 PWM (−900～+900‰)", self.swing_stage2_pwm_text),
            ("第二段每次时长 (1–1000 ms)", self.swing_stage2_ms_text),
        )
        for row, (label, variable) in enumerate(swing_fields):
            ttk.Label(swing, text=label,wraplength=185).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(swing, textvariable=variable, width=9).grid(row=row, column=1,
                                                                     padx=5, pady=3)
        ttk.Button(swing, text="发送起摆参数（停机时）", command=self.send_swing_config)\
            .grid(row=10, column=0, columnspan=2, sticky="ew", pady=5)
        ttk.Label(swing, textvariable=self.swing_text, wraplength=300)\
            .grid(row=11, column=0, columnspan=2, sticky="w")
        ttk.Label(swing,text="三个 ADC 参数均可独立填写 0–4095，不限制大小顺序或间距；换圈端点应按实际校准。第一段达到目标后进入第二段，两段分别设置 PWM 和时长。最低点过点补判范围为窗口加 128 码；脉冲尚未结束时保留触发，离开此范围后不补发。进入直立 PID 页的捕获窗口后由 PID 接管，当时编码器位置及外环目标归零。第二段 PWM=0 时滑行等待接管。软行程取本值与硬行程较小者。",wraplength=270,foreground="#596b7d").grid(row=12,column=0,columnspan=2,sticky="w",pady=5)
        ttk.Button(pid_settings, text="发送全部参数（会停机）", command=self.send_config)\
            .pack(fill="x", pady=(12, 2))
        ttk.Label(pid_settings, text="发送 PID 参数会停机，收到回显后可点击启动或按实体运行键。",
                  foreground="#995222", wraplength=310).pack(anchor="w")
        ttk.Label(pid_settings, textvariable=self.config_text, wraplength=310)\
            .pack(anchor="w", pady=(8, 0))
        self.adc_plot = TracePlot(plots, "摆杆电位器电压", "#1769aa", "V")
        self.angle_plot = TracePlot(plots, "电机转臂角度（相对捕获位置 · 双击全屏）", "#8556af", "°")
        self.pwm_plot = TracePlot(plots, "电机最终 PWM（含死区补偿）",
                                  "#dc6b1d", "%", (-100, 100))
        self.angle_plot.canvas.bind("<Double-Button-1>", self.open_angle_fullscreen)
        self.angle_plot.minimum_span = 180
        for index, plot in enumerate((self.adc_plot, self.angle_plot, self.pwm_plot)):
            plot.frame.grid(row=index, column=0, sticky="nsew", padx=4, pady=4)
            plots.rowconfigure(index, weight=1)
        plots.columnconfigure(0, weight=1)
        if serial is None:
            self.status_text.set("缺少 pyserial，请安装后重开")

    def _config_frame(self):
        p = self.parameters
        return make_config(
            int(p["零点 ADC"].get()), int(p["角度方向"].get()),
            int(p["编码器方向"].get()), int(p["捕获窗口"].get()),
            int(p["跌倒门限"].get()), int(p["行程门限"].get()),
            float(p["Kp"].get()), float(p["Ki"].get()), float(p["Kd"].get()),
            float(p["转臂 Kp"].get()), float(p["转臂 Kd"].get()))

    def _swing_frame(self):
        return make_swing_config(*(int(v.get()) for v in self._swing_variables().values()))

    def _load_profile(self):
        profile = Path(__file__).resolve().parent / "tuned_profile.json"
        if not profile.exists():
            return
        try:
            data = migrate_profile(json.loads(profile.read_text(encoding="utf-8")))
            for name, value in data["balance"].items():
                if name in self.parameters:
                    self.parameters[name].set(str(value))
            variables = self._swing_variables()
            if "pump_pwm" not in data["swing"]:
                raise ValueError("旧起摆参数不能用于第一阶段，请重新设置脉冲参数")
            for name, value in data["swing"].items():
                if name in variables:
                    variables[name].set(str(value))
            self.period_ms_text.set("1.0")
            self.outer_period_ms_text.set(str(data.get("outer_period_ms",10)))
            self.wrap_low_text.set(str(data["wrap_low"]))
            self.wrap_high_text.set(str(data["wrap_high"]))
            self.tilt_limit_text.set(str(data.get("tilt_limit", "2")))
            self.tolerance_text.set(str(data.get("target_tolerance_deg",data.get("position_tolerance_deg","2.0"))))
            self.target_trim_enabled.set(bool(data.get("target_trim_enabled",False)) if data.get("compensation_tuning_version")==1 else False)
            self.correction_strength_text.set(str(data.get("correction_strength","柔和")))
            self.correction_rate_text.set(str(data.get("correction_rate_deg_s","3.0")))
            self.pwm_limit_text.set(str(data.get("pwm_output_limit_percent","90.0")))
            self.response_mode_text.set("连续" if data.get("target_response_fast",True) else "原速")
            for var,key,default in ((self.target_trim_max_text,"target_trim_max_deg","60"),(self.board_amplitude_text,"board_amplitude_deg","60"),(self.board_interval_text,"board_interval_seconds","2"),(self.small_step_limit_text,"small_step_tilt_limit","20"),(self.large_step_limit_text,"large_step_tilt_limit","40")):
                var.set(str(data.get(key,default)))
            self.small_step_limit_text.set(str(data.get("normal_tilt_limit",data.get("small_step_tilt_limit","20"))))
            self.large_step_limit_text.set(str(data.get("forward_turn_tilt_limit",data.get("large_step_tilt_limit","40"))))
            self.reverse_turn_limit_text.set(str(data.get("reverse_turn_tilt_limit",data.get("large_step_tilt_limit","40"))))
            self.tilt_limit_text.set(self.small_step_limit_text.get())
            self.deadzone_text.set(str(data.get("pwm_deadzone", "0")))
            self.counts_text.set(str(data.get("counts_per_rev", self.counts_per_rev)))
            self.arm_step_text.set(str(data.get("arm_step_deg", "10")))
            if not self.apply_counts(show_error=False):
                raise ValueError("编码器每圈计数无效")
            self._config_frame()
            self._swing_frame()
            make_outer_period(float(self.outer_period_ms_text.get()))
            make_control_period(1.0)
            make_wrap_config(int(self.wrap_low_text.get()),
                             int(self.wrap_high_text.get()))
            make_tilt_limit(int(self.tilt_limit_text.get()))
            self._trim_frame()
            self._pwm_limit_frame()
            self._tuning_frame()
            self._board_frame()
            make_pwm_deadzone(int(self.deadzone_text.get()))
            self.config_text.set("已载入参数；旧档案的自动补偿已关闭，需明确启用并发送" if data.get("compensation_tuning_version")!=1 else "已载入上次保存的参数；连接后可一键启动")
            self.profile_loaded = True
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.config_text.set(f"参数档案无效：{exc}")

    def _swing_variables(self):
        return dict(pump_pwm=self.swing_pump_text,pulse_ms=self.swing_duration_text,
                    bottom_code=self.swing_bottom_text,target_code=self.swing_target_code_text,
                    upper_code=self.swing_upper_text,bottom_window=self.swing_window_text,
                    rearm_distance=self.swing_rearm_text,travel=self.swing_target_text,
                    stage2_pwm=self.swing_stage2_pwm_text,stage2_ms=self.swing_stage2_ms_text)

    def save_profile(self):
        try:
            if not self.apply_counts(show_error=False):
                raise ValueError("编码器每圈计数无效")
            self._config_frame()
            self._swing_frame()
            make_outer_period(float(self.outer_period_ms_text.get()))
            make_control_period(1.0)
            make_wrap_config(int(self.wrap_low_text.get()),
                             int(self.wrap_high_text.get()))
            make_tilt_limit(int(self.tilt_limit_text.get()))
            self._trim_frame()
            self._pwm_limit_frame()
            self._tuning_frame()
            self._board_frame()
            make_pwm_deadzone(int(self.deadzone_text.get()))
            data = {"balance": {key: value.get() for key, value in self.parameters.items()},
                    "swing": {key: value.get() for key, value in self._swing_variables().items()},
                    "format_version": 10, "swing_mode": "two_stage_bottom_pulse", "adc_units": "ads8688_high12", "control_mode": "position_pd_native",
                    "counts_per_rev": self.counts_text.get(),
                    "arm_step_deg": self.arm_step_text.get(),
                    "period_ms": "1.0",
                    "outer_period_ms": self.outer_period_ms_text.get(),
                    "tilt_limit": self.tilt_limit_text.get(),
                    "target_tolerance_deg": self.tolerance_text.get(),
                    "target_trim_enabled":self.target_trim_enabled.get(),
                    "compensation_tuning_version":1,
                    "correction_strength":self.correction_strength_text.get(),
                    "correction_rate_deg_s":self.correction_rate_text.get(),
                    "pwm_output_limit_percent":self.pwm_limit_text.get(),
                    "target_response_fast":self.response_mode_text.get()=="连续",
                    "target_trim_max_deg":self.target_trim_max_text.get(),
                    "board_amplitude_deg":self.board_amplitude_text.get(),
                    "board_interval_seconds":self.board_interval_text.get(),
                    "small_step_tilt_limit":self.small_step_limit_text.get(),
                    "large_step_tilt_limit":self.large_step_limit_text.get(),
                    "normal_tilt_limit":self.small_step_limit_text.get(),
                    "forward_turn_tilt_limit":self.large_step_limit_text.get(),
                    "reverse_turn_tilt_limit":self.reverse_turn_limit_text.get(),
                    "pwm_deadzone": self.deadzone_text.get(),
                    "wrap_low": self.wrap_low_text.get(),
                    "wrap_high": self.wrap_high_text.get()}
            path = Path(__file__).resolve().parent / "tuned_profile.json"
            path.write_text(json.dumps(data, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        except (OSError, ValueError, OverflowError) as exc:
            messagebox.showerror("保存参数", str(exc))
            return
        self.config_text.set("参数已保存；下次打开上位机自动载入")
        self.profile_loaded = True

    def _start_log(self):
        log_dir = Path(__file__).resolve().parent.parent / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"
        metadata = {
            "started_at": datetime.now().astimezone().isoformat(),
            "port": self.port_name.get(),
            "counts_per_rev": self.counts_per_rev,
            "balance": {name: value.get() for name, value in self.parameters.items()},
            "tilt_limit_adc": self.tilt_limit_text.get(),
            "target_tolerance_deg": self.tolerance_text.get(),
                    "target_trim_enabled":self.target_trim_enabled.get(),
                    "compensation_tuning_version":1,
                    "correction_strength":self.correction_strength_text.get(),
                    "correction_rate_deg_s":self.correction_rate_text.get(),
                    "pwm_output_limit_percent":self.pwm_limit_text.get(),
                    "target_response_fast":self.response_mode_text.get()=="连续",
                    "target_trim_max_deg":self.target_trim_max_text.get(),
                    "board_amplitude_deg":self.board_amplitude_text.get(),
                    "board_interval_seconds":self.board_interval_text.get(),
                    "small_step_tilt_limit":self.small_step_limit_text.get(),
                    "large_step_tilt_limit":self.large_step_limit_text.get(),
                    "normal_tilt_limit":self.small_step_limit_text.get(),
                    "forward_turn_tilt_limit":self.large_step_limit_text.get(),
                    "reverse_turn_tilt_limit":self.reverse_turn_limit_text.get(),
            "pwm_deadzone_per_mille": self.deadzone_text.get(),
            "period_ms": "1.0",
                    "outer_period_ms": self.outer_period_ms_text.get(),
                    "adc_units": "ads8688_high12",
            "wrap_low": self.wrap_low_text.get(),
            "wrap_high": self.wrap_high_text.get(),
            "swing": {name: value.get() for name, value in self._swing_variables().items()},
            "config_echo_confirmed": False,
            "tilt_echo_confirmed": False,
        }
        self.log_metadata_path = path.with_suffix(".json")
        self.log_metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        self.log_file = path.open("w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(("seconds", "type", "adc_code", "adc_invalid",
                                  "arm_position", "pendulum_speed", "pwm_per_mille",
                                  "encoder_invalid_count", "flags", "fault_reason",
                                  "target_tilt_adc", "arm_target_count", "kp", "ki", "kd", "arm_kp", "arm_kd", "internal_target_count", "trim_status", "arm_velocity_q8", "monitor_counts_per_rev", "reciprocal_center"))
        self.log_started = time.monotonic()
        self.log_rows = 0
        self.log_text.set(f"运行记录：{path.name}")

    def _log_row(self, row):
        if self.log_writer is None:
            return
        self.log_writer.writerow(row)
        self.log_rows += 1
        if self.log_rows % 100 == 0:
            self.log_file.flush()

    def _finish_log(self):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
            self.log_writer = None
        self.log_metadata_path = None

    def _confirm_log_parameter(self, key, value=None):
        if self.log_metadata_path is None:
            return
        try:
            metadata = json.loads(self.log_metadata_path.read_text(encoding="utf-8"))
            history=metadata.setdefault("parameter_history",[])
            history.append({"seconds":round(time.monotonic()-self.log_started,6),"key":key,"value":value})
            metadata[key] = True
            if value is not None:
                metadata[key.removesuffix("_confirmed")] = value
            self.log_metadata_path.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        except (OSError, ValueError):
            pass

    @staticmethod
    def _card(parent, title, value, subtitle):
        box = ttk.LabelFrame(parent, text=title, padding=(10, 6))
        box.pack(fill="x", pady=(0, 5))
        ttk.Label(box, textvariable=value, font=("Segoe UI", 18, "bold"),
                  anchor="center").pack(fill="x")
        if subtitle is not None:
            subtitle_area = ttk.Frame(box, height=38)
            subtitle_area.pack(fill="x")
            subtitle_area.pack_propagate(False)
            ttk.Label(subtitle_area, textvariable=subtitle, anchor="center",
                      justify="center", wraplength=310).pack(fill="both", expand=True)

    def refresh_ports(self):
        if list_ports is None:
            return
        ports = [item.device for item in list_ports.comports()]
        self.port_box["values"] = ports
        if not self.port_name.get() and ports:
            self.port_name.set(ports[0])

    def toggle_connection(self):
        if self.stop_event is not None:
            self._begin_disconnect()
            return
        if serial is None:
            messagebox.showerror("串口", "缺少 pyserial。")
            return
        name = self.port_name.get().strip()
        try:
            baud = int(self.baud_text.get())
            if not name or baud <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("串口", "请选择串口并填写有效波特率。")
            return
        if not self.apply_full_scale(show_error=True) or not self.apply_counts(show_error=True):
            return
        self.clear_samples()
        self.reference_reciprocal = False
        self.reference_center_degrees = None
        self.latest_code = None
        self.latest_motor = None
        self.configured_zero = None
        self.configured_travel = None
        self.configured_wrap_low = 0
        self.configured_wrap_high = 2640
        self.pending_config = None
        self.pending_pwm_limit=None;self.pwm_limit_deadline=0.0
        self.pending_tuning=None;self.tuning_deadline=0.0
        self.pending_response=None;self.response_deadline=0.0
        self.pending_tolerance=None
        self.tolerance_deadline=0.0
        self.pending_board=None
        self.board_deadline=0.0
        self.pending_arm_tilt=None
        self.latest_monitor=None
        self.latest_monitor_at=0.0
        self.monitor_decode_error=""
        self.monitor_request_at=0.0
        self.pending_swing = None
        self.pending_period = None
        self.pending_outer_period = None
        self.pending_wrap = None
        self.pending_tilt = None
        self.pending_deadzone = None
        self._reset_live_pid(supported=False)
        self._reset_live_pd(supported=False)
        self.pending_arm_is_turn=False
        self.pending_arm_target = None
        self.arm_target_confirmed = False
        self.wide_arm_firmware = False
        self.triplet_settings_confirmed=False;self.confirmed_tilt_profiles=None;self.last_tilt_phase_logged=None
        self.tilt_profile_status.set("三种倾角限幅：等待FPGA确认")
        self.arm_target_count = 0
        self.arm_target_text.set("转臂目标：等待 FPGA 回显")
        self.arm_target_status.set("直立运行后可调整目标角")
        self.remote_run_requested = False
        self.adc_value.set("--.--- V")
        self.adc_code_value.set("ADC ----")
        self.motor_angle_value.set("--.--°")
        self.pwm_value.set("--.- %")
        self.target_tilt_value.set("目标摆杆倾角：--")
        self.motor_count_value.set("编码器 -- 计数")
        self.state_text.set("等待电机状态")
        self.fault_text.set("")
        self.config_text.set("尚未收到 FPGA 参数回显")
        self.zero_text.set("FPGA 电位器起点：未设置")
        self.period_text.set("PID 计算周期：请发送设置以确认 FPGA 当前值")
        self.wrap_text.set("换圈端点：FPGA 默认低 0 / 高 2640，建议实测后发送")
        self.tilt_text.set("目标倾角上限：等待 FPGA 回显")
        self.deadzone_status.set("PWM 死区补偿：等待 FPGA 回显")
        self.pwm_limit_status.set("总输出上限：等待FPGA确认")
        self.swing_text.set("两阶段起摆：等待 FPGA 参数回显")
        self.pendulum_speed = 0
        self.rate_started = time.monotonic()
        self.adc_since_rate = 0
        self.motor_since_rate = 0
        self.session += 1
        session = self.session
        self.stop_event = threading.Event()
        self.write_queue = queue.Queue()
        self.connect_button.configure(text="取消连接")
        self.status_text.set(f"正在连接 {name}…")
        threading.Thread(target=self.serial_worker,
                         args=(session, name, baud, self.stop_event, self.write_queue),
                         daemon=True).start()

    def serial_worker(self, session, name, baud, stop_event, write_queue):
        decoder = FrameDecoder()
        try:
            with serial.Serial(name, baud, timeout=0.05, write_timeout=0.3) as port:
                self.events.put((session, "connected", name))
                while not stop_event.is_set():
                    try:
                        while True:
                            port.write(write_queue.get_nowait())
                    except queue.Empty:
                        pass
                    chunk = port.read(min(max(port.in_waiting, 1), 4096))
                    if not chunk:
                        continue
                    received_at = time.monotonic()
                    adc_batch = []
                    for frame_type, payload in decoder.feed(chunk):
                        if frame_type == 0x34 and len(payload) == 3:
                            adc_batch.append((received_at, *decode_adc(payload)))
                        elif frame_type == 0x35 and len(payload) == 18:
                            target = (int.from_bytes(payload[12:14], "little", signed=True)
                                      if len(payload) >= 14 else None)
                            arm_target = (int.from_bytes(payload[14:18], "little", signed=True)
                                          if len(payload) == 18 else None)
                            self.events.put((session, "motor", (received_at, *decode_motor(payload), target, arm_target)))
                        elif frame_type == 0x33 and len(payload) == 30:
                            self.events.put((session, "config", payload))
                        elif frame_type == 0x0D and len(payload) in (5,7):
                            self.events.put((session, "arm_target", payload))
                        elif ((frame_type == 0x29 and len(payload) == 20) or (frame_type == 0x27 and len(payload) == 16)):
                            self.events.put((session, "swing", payload))
                        elif frame_type == 0x11 and len(payload) == 2:
                            self.events.put((session,"outer_period",payload))
                        elif frame_type == 0x05 and len(payload) == 2:
                            self.events.put((session, "period", payload))
                        elif frame_type == 0x06 and len(payload) == 4:
                            self.events.put((session, "wrap", payload))
                        elif frame_type == 0x2F and len(payload) == 9:
                            self.events.put((session,"live_pd",payload))
                        elif frame_type == 0x36 and len(payload) == 11:
                            self.events.put((session, "touch_state", payload))
                        elif frame_type == 0x2D and len(payload) == 13:
                            self.events.put((session,"live_pid",payload))
                        elif frame_type == 0x2B and len(payload) == 2:
                            self.events.put((session, "deadzone", payload))
                        elif frame_type == 0x49 and len(payload) == 3:
                            self.events.put((session,"pwm_limit",payload))
                        elif frame_type == 0x47 and len(payload) == 4:
                            self.events.put((session,"correction_tuning",payload))
                        elif frame_type == 0x45 and len(payload) == 2:
                            self.events.put((session,"response_speed",payload))
                        elif frame_type == 0x38 and len(payload) == 9:
                            self.events.put((session,"target_trim",payload))
                        elif frame_type == 0x41 and len(payload) in (12,16,18):
                            self.events.put((session,"board_config",payload))
                        elif frame_type == 0x39 and len(payload) in (24,29):
                            self.events.put((session,"dual_monitor",(received_at,payload)))
                        elif frame_type == 0x08 and len(payload) == 2:
                            self.events.put((session, "tilt", payload))
                    if adc_batch:
                        self.events.put((session, "adc", adc_batch))

                
                while True:
                    try:remaining=write_queue.get_nowait()
                    except queue.Empty:break
                    if remaining==make_run_command(False):port.write(remaining)
                port.write(make_board_handoff())
                port.flush()
                deadline=time.monotonic()+0.6
                released=None
                while time.monotonic()<deadline and released is None:
                    chunk=port.read(min(max(port.in_waiting,1),4096))
                    for kind,payload in decoder.feed(chunk):
                        if kind==0x3c and len(payload)==1 and payload[0]<=2:
                            released=payload[0];break
                self.events.put((session,"handoff",released))
                port.write(make_monitor_subscription(False));port.flush()
        except Exception as exc:
            if not stop_event.is_set():
                self.events.put((session, "error", str(exc)))
            else:self.events.put((session,"handoff_error",str(exc)))
        finally:
            self.events.put((session, "disconnected", None))

    def poll_events(self):
        for _ in range(400):
            try:
                session, kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if session != self.session:
                continue
            if kind == "connected":
                self.connected = True
                self.connect_button.configure(text="断开")
                self.status_text.set(f"已连接 {value}")
                self.write_queue.put(make_query_command())
                self.write_queue.put(make_monitor_subscription(True))
                self.monitor_request_at=time.monotonic()
            elif kind == "adc":
                self.adc_since_rate += len(value)
                for timestamp, code, fault in value:
                    self._log_row((f"{timestamp-self.log_started:.6f}", "adc",
                                   code, fault, "", "", "", "", "", "", "", ""))
                    if not fault:
                        self.adc_samples.append((timestamp, code))
                timestamp, code, fault = value[-1]
                if fault:
                    self.latest_code = None
                    self.adc_value.set("ADC 故障")
                    self.adc_code_value.set("检查传感器/SPI")
                else:
                    self.latest_code = code
                    self.latest_adc_at = timestamp
                    self.adc_value.set(f"{code / 4096 * self.full_scale:.3f} V")
                    delta = ""
                    if self.configured_zero is not None:
                        offset = code - self.configured_zero
                        span = self.configured_wrap_high - self.configured_wrap_low
                        if offset > span // 2:
                            offset -= span
                        elif offset < -(span // 2):
                            offset += span
                        delta = f" · 环形起点偏差 {offset:+d}"
                    self.adc_code_value.set(f"ADS8688 高12位 12 位码 {code}{delta} · 摆速 {self.pendulum_speed:+d} 码/采样")
            elif kind == "motor":
                timestamp, position, pendulum_speed, pwm, invalid, flags, reason, target, arm_target = value
                self._log_row((f"{timestamp-self.log_started:.6f}", "motor", "", "",
                               position, pendulum_speed, pwm, invalid, flags, reason,
                               "" if target is None else target, "" if arm_target is None else arm_target))
                if arm_target is not None:
                    self.arm_target_count = arm_target
                    self.arm_target_confirmed = True
                    self.wide_arm_firmware = True
                    self.update_arm_target_display()
                self.pendulum_speed = pendulum_speed
                if not flags & 16:
                    pwm = 0
                self.latest_motor = (timestamp, position, pendulum_speed,
                                     pwm, invalid, flags, reason)
                if target is not None:
                    span = max(1, self.configured_wrap_high - self.configured_wrap_low)
                    self.target_tilt_value.set(
                        f"目标摆杆倾角：{target:+d} ADC 码（约 {target * 360 / span:+.2f}°）")
                self.motor_since_rate += 1
                self.motor_samples.append((timestamp, position, pwm))
                self.record_angle_reference(timestamp)
                self.update_motor_readings()
                state = ("故障锁定" if flags & 4 else "直立 PID 运行（接管位置为编码器起点）" if flags & 2
                         else ("第二阶段脉冲起摆" if flags & 128 else "第一阶段脉冲起摆") if flags & 64
                         else "第一阶段完成，已停机" if flags & 128 else "手动起摆：等待直立 PID 接管" if flags & 1 else "停机")
                self.state_text.set(
                    f"{state}  ·  ADC {'有效' if flags & 8 else '故障'}"
                    f"  ·  电机 {'使能' if flags & 16 else '关闭'}"
                    f"  ·  参数 {'已加载' if flags & 32 else '未加载'}"
                    f"  ·  编码器异常跳变 {invalid}")
                if flags & 4:
                    detail = self.FAULT_REASONS.get(reason, f"未知故障代码 {reason}")
                    if reason == 4 and self.configured_travel is not None:
                        detail += f"（门限 ±{self.configured_travel} x4 计数）"
                    self.fault_text.set(f"故障停机原因：{detail}")
                    if self.remote_run_requested:
                        self.remote_run_requested = False
                        self.write_queue.put(make_run_command(False))
                    self.auto_start_pending = False
                    self._finish_log()
                elif flags & 128 and not flags & 64:
                    self.remote_run_requested = False
                    self.auto_start_pending = False
                    self.fault_text.set("第一阶段目标已达到，PWM 已停止")
                    self._finish_log()
                elif flags & 1:
                    self.fault_text.set("")
            elif kind == "config":
                self.wide_arm_firmware = len(value) == 30
                if not self.wide_arm_firmware:
                    self.arm_target_status.set("需要烧录支持目标角及大 D 的新版固件")
                zero = int.from_bytes(value[:2], "little")
                if zero < 16:
                    self.config_text.set("FPGA 尚未加载直立 PID 参数")
                    continue
                self.configured_zero = zero
                self.configured_travel = int.from_bytes(value[8:10], "little")
                self.zero_text.set(f"FPGA 电位器起点：ADC {zero}")
                arm_gains = ""
                if len(value) >= 20:
                    arm_kp = int.from_bytes(value[22:26], "little", signed=True) / 65536
                    arm_kd = int.from_bytes(value[26:30], "little", signed=True) / 65536
                    actual_ki = int.from_bytes(value[14:18], "little", signed=True) / 4096
                    actual_kd = int.from_bytes(value[18:22], "little", signed=True) / 256
                    arm_gains = f"；Ki={actual_ki:g}、Kd={actual_kd:g}；转臂 Kp={arm_kp:g}、Kd={arm_kd:g}"
                    if arm_kp == 0 and arm_kd == 0:
                        arm_gains += "（外环未启用，目标倾角恒为 0）"
                if value == self.pending_config:
                    self._confirm_log_parameter("config_echo_confirmed", value.hex())
                    self.config_text.set(
                        f"FPGA 已确认：摆杆零点 ADC={zero}{arm_gains}；可选择自动或手动起摆")
                    self.pending_config = None
                else:
                    self.config_text.set(f"FPGA 回显配置：摆杆零点 ADC={zero}{arm_gains}")
            elif kind == "arm_target":
                count, result = decode_arm_target_echo(value)
                if len(value)==7:self.tilt_text.set(f"FPGA已确认：外环倾角限制 ±{int.from_bytes(value[5:7],'little')} ADC码")
                self.arm_target_count = count
                self.arm_target_confirmed = True
                self.update_arm_target_display()
                if result:
                    self.pending_arm_target = None
                    self.arm_target_status.set("FPGA 拒绝：当前未直立运行" if result == 1 else
                                               "FPGA 拒绝：目标超出硬行程")
                elif self.pending_arm_target == count and len(value)==7 and (int.from_bytes(value[5:7],"little")==self.pending_arm_tilt or
                        (self.triplet_settings_confirmed and self.pending_arm_is_turn and self.confirmed_tilt_profiles is not None and
                         int.from_bytes(value[5:7],"little")==self.confirmed_tilt_profiles[0])):
                    self.pending_arm_target = None
                    self.arm_target_status.set("FPGA 已确认目标，正在跟踪")
                elif self.pending_arm_target is None:
                    self.arm_target_status.set("FPGA 目标已同步")
            elif kind == "swing":
                numbers=decode_swing_echo(value)
                pump, duration, bottom, target, upper, window, rearm, travel=numbers[:8]
                stage2_info=(f"；第二段 {numbers[8]}‰ / {numbers[9]} ms；进入捕获窗口后 PID 接管并设置编码器起点"
                             if len(numbers)==10 else "；旧单段模式，到目标停机；两段功能需要新版固件")
                if not self.profile_loaded or value == self.pending_swing:
                    for variable,number in zip(self._swing_variables().values(),numbers):variable.set(str(number))
                hard=self.configured_travel
                effective=min(travel,hard) if hard is not None else None
                limit_info=f"；生效软行程 ±{effective} 计数" if effective is not None else "；生效软行程待硬行程回显"
                self.swing_text.set(f"FPGA 已确认：首次 {pump}‰、正反向每次 {duration} ms；最低 {bottom}、目标 {target}、最高 {upper}"+limit_info+stage2_info)
                if value == self.pending_swing:
                    self.pending_swing = None
            elif kind == "period":
                period_us = int.from_bytes(value, "little")
                self.period_text.set(f"FPGA 已确认：PID 每 {period_us / 1000:.1f} ms 计算一次")
                if self.pending_period is None and not self.profile_loaded:
                    self.period_ms_text.set(f"{period_us / 1000:.1f}")
                if value == self.pending_period:
                    self.pending_period = None
            elif kind == "outer_period":
                period_us=int.from_bytes(value,"little")
                if not 1000<=period_us<=50000:continue
                self.confirmed_outer_period_us=period_us
                self.outer_period_text.set(f"FPGA 已确认：外环每 {period_us/1000:.1f} ms（{1000000/period_us:g} Hz）")
                if not self.profile_loaded and self.pending_outer_period is None:self.outer_period_ms_text.set(f"{period_us/1000:.1f}")
                if value==self.pending_outer_period:
                    self.pending_outer_period=None
                    self._confirm_log_parameter("outer_period_confirmed_us",period_us)
            elif kind == "wrap":
                low = int.from_bytes(value[:2], "little")
                high = int.from_bytes(value[2:], "little")
                self.configured_wrap_low = low
                self.configured_wrap_high = high
                self.wrap_text.set(f"FPGA 已确认：低码 {low} / 高码 {high}")
                if self.pending_wrap is None and not self.profile_loaded:
                    self.wrap_low_text.set(str(low))
                    self.wrap_high_text.set(str(high))
                if value == self.pending_wrap:
                    self.pending_wrap = None
            elif kind == "live_pd":
                self._live_pd_echo(value)
            elif kind == "touch_state":
                self._sync_touch_state(value)
            elif kind == "live_pid":
                self._live_pid_echo(value)
            elif kind == "deadzone":
                codes=int.from_bytes(value,"little")
                if not 0<=codes<=300:continue
                self.deadzone_status.set(f"FPGA 已确认：补偿 {codes}‰（{codes/10:g}%），最终PWM服从总限幅设置")
                if self.pending_deadzone is None and not self.profile_loaded:self.deadzone_text.set(str(codes))
                if value==self.pending_deadzone:
                    self.pending_deadzone=None
                    self._confirm_log_parameter("pwm_deadzone_confirmed_per_mille",codes)
            elif kind == "handoff":
                self.handoff_result=value
            elif kind == "handoff_error":
                self.handoff_result=None
                self.status_text.set(f"板端接管失败：{value}")
            elif kind == "pwm_limit":
                self._pwm_limit_echo(value)
            elif kind == "correction_tuning":
                self._correction_tuning_echo(value)
            elif kind == "response_speed":
                self._response_speed_echo(value)
            elif kind == "target_trim":
                self._position_tolerance_echo(value)
            elif kind == "board_config":
                self._board_echo(value)
            elif kind == "dual_monitor":
                self._monitor_echo(*value)
            elif kind == "tilt":
                codes = int.from_bytes(value, "little")
                self.tilt_text.set(f"FPGA 已确认：目标摆杆倾角限制 ±{codes} ADC 码")
                if self.pending_tilt is None and not self.profile_loaded:
                    self.tilt_limit_text.set(str(codes))
                if value == self.pending_tilt:
                    self._confirm_log_parameter("tilt_echo_confirmed", codes)
                    self.pending_tilt = None
            elif kind in ("error", "disconnected"):
                self.connected = False
                self.triplet_settings_confirmed=False;self.confirmed_tilt_profiles=None;self.last_tilt_phase_logged=None
                self.tilt_profile_status.set("串口已释放；当前倾角档位需重连确认")
                self.pwm_limit_status.set("串口已释放；当前总限幅需重连查询确认")
                self._reset_live_pid(supported=False)
                self._reset_live_pd(supported=False)
                self.remote_run_requested = False
                self.auto_start_pending = False
                self._finish_log()
                self.stop_event = None
                was_disconnecting=self.disconnecting
                self.disconnecting=False
                self.connect_button.configure(text="连接",state="normal")
                if kind == "error":
                    self.status_text.set(f"串口错误：{value}")
                elif was_disconnecting:
                    self.status_text.set("已交给板端继续运行，串口已释放" if self.handoff_result==0 else "已断开，板端未运行" if self.handoff_result==1 else "已断开，板端处于故障" if self.handoff_result==2 else "串口已释放，但板端接管未确认")
                elif not self.status_text.get().startswith("串口错误"):
                    self.status_text.set("已断开")
                if self.close_pending:
                    self.root.destroy();return
        if self.pending_pwm_limit is not None and time.monotonic()>self.pwm_limit_deadline:
            self.pending_pwm_limit=None;self.auto_start_pending=False
            self.pwm_limit_status.set("总限幅确认超时，自动启动已取消；请检查配套固件")
        if self.pending_tuning is not None and time.monotonic()>self.tuning_deadline:
            self.pending_tuning=None;self.auto_start_pending=False
            self.correction_tuning_status.set("强度与限速确认超时，自动启动已取消")
        if self.pending_response is not None and time.monotonic()>self.response_deadline:
            self.pending_response=None;self.auto_start_pending=False
            self.response_status.set("响应速度确认超时，自动启动已取消")
        if self.pending_board is not None and time.monotonic()>self.board_deadline:
            self.pending_board=None
            self.auto_start_pending=False
            self.board_status_text.set("三种倾角及往复设置确认超时；启动已取消")
        if self.pending_tolerance is not None and time.monotonic()>self.tolerance_deadline:
            self.pending_tolerance=None
            self.auto_start_pending=False
            self.tolerance_status.set("双目标补偿确认超时；启动已取消，请使用本目录配套固件")
        if self.auto_start_pending:
            if time.monotonic() > self.auto_start_deadline:
                self.auto_start_pending = False
                self._finish_log()
                self.status_text.set("启动取消：5 秒内未收到全部 FPGA 参数回显")
            elif (self.latest_motor is not None and self.latest_motor[-2] & 32 and
                  all(value is None for value in
                      (self.pending_config, self.pending_swing,
                       self.pending_period, self.pending_outer_period, self.pending_wrap, self.pending_tilt, self.pending_deadzone, self.pending_tolerance, self.pending_board, self.pending_response, self.pending_tuning, self.pending_pwm_limit))):
                self.auto_start_pending = False
                self.remote_run_requested = True
                self.write_queue.put(make_run_command(True,manual=self.manual_start))
                self.status_text.set("参数均已回显，手动起摆等待直立接管" if self.manual_start else "参数均已回显，自动起摆请求已发送")
        if self.pending_arm_target is not None and time.monotonic() >= self.arm_target_deadline:
            self.pending_arm_target = None
            self.arm_target_confirmed = False
            self.arm_target_status.set("目标确认超时，正在重新查询 FPGA")
            self.write_queue.put(make_query_command())
        self._service_live_pid()
        self._service_live_pd()
        self.update_arm_target_buttons()
        self._refresh_target_readout()
        self.root.after(50, self.poll_events)

    def update_rates(self):
        now = time.monotonic()
        elapsed = max(now - self.rate_started, 0.001)
        adc_rate = self.adc_since_rate / elapsed
        motor_rate = self.motor_since_rate / elapsed
        self.rate_text.set(f"实际收到：ADC {adc_rate:.0f}/{self.ADC_REPORT_HZ} Hz  ·  "
                           f"电机 {motor_rate:.0f}/{self.MOTOR_REPORT_HZ} Hz")
        self.rate_started = now
        self.adc_since_rate = 0
        self.motor_since_rate = 0
        self.root.after(1000, self.update_rates)

    def apply_full_scale(self, show_error=False):
        try:
            value = float(self.full_scale_text.get())
            if not math.isfinite(value) or value <= 0:
                raise ValueError
        except ValueError:
            if show_error:
                messagebox.showerror("ADC 满量程", "请输入正的显示满量程电压。")
            return False
        self.full_scale = value
        if self.latest_code is not None:
            self.adc_value.set(f"{self.latest_code / 4096 * value:.3f} V")
        return True

    def apply_counts(self, show_error=True):
        try:
            count = int(self.counts_text.get())
            if not 1 <= count <= 1000000:
                raise ValueError
        except ValueError:
            if show_error:
                messagebox.showerror("编码器", "请输入每转的 x4 编码器计数，例如 1040。")
            return False
        self.counts_per_rev = count
        self.update_motor_readings()
        self.update_arm_target_display()
        return True

    def update_arm_target_display(self):
        if not self.arm_target_confirmed:
            return
        degrees = self.user_target_degrees()
        self.arm_target_text.set(f"转臂目标：{degrees:+d}°\n"
                                 f"编码器目标：{self.arm_target_count:+d} 计数")

    def user_target_degrees(self):
        return user_degrees(self.arm_target_count, self.counts_per_rev)

    def arm_target_operable(self):
        return (self.connected and self.wide_arm_firmware and self.arm_target_confirmed
                and self.pending_arm_target is None and self.latest_motor is not None
                and time.monotonic() - self.latest_motor[0] < 0.5
                and self.latest_motor[-2] & 3 == 3 and self.latest_motor[-2] & 16
                and not self.latest_motor[-2] & 4)

    def update_arm_target_buttons(self):
        state = "normal" if self.arm_target_operable() else "disabled"
        for button in self.arm_target_buttons:
            button.configure(state=state)

    # 对累计用户角度统一量化，避免逐次量化步进导致编码器舍入误差累积。
    def shift_arm_target(self, direction):
        if not self.arm_target_operable():
            return
        try:
            if not self.apply_counts():
                return
            step = float(self.arm_step_text.get())
            if step <= 0:
                raise ValueError("请输入大于 0° 的增减角度")
            limit=self._step_limit(direction*step)
            target = shifted_arm_target(self.arm_target_count, direction * step,
                                        self.counts_per_rev, self.configured_travel)
            if step.is_integer():

                
                target = round((self.user_target_degrees()+direction*int(step))*self.counts_per_rev/360)

            limit=self._step_limit((target-self.arm_target_count)*360/self.counts_per_rev)
        except (ValueError, TypeError) as exc:
            messagebox.showerror("转臂目标", str(exc))
            return
        self.send_arm_target(target,limit)

    def send_arm_target(self, target,limit=None):
        if not self.arm_target_operable():
            return
        if self.configured_travel is None or abs(target) > self.configured_travel:
            messagebox.showerror("转臂目标", "目标超出 FPGA 确认的硬行程")
            return
        try:
            if limit is None:
                delta_degrees=(target-self.arm_target_count)*360/self.counts_per_rev
                limit=self._step_limit(delta_degrees) if abs(delta_degrees)<=360 else int(self.small_step_limit_text.get())
            frame=make_arm_target(target,limit)
        except (ValueError,OverflowError) as exc:
            messagebox.showerror("转臂目标",str(exc));return
        self.pending_arm_is_turn=abs(target-self.arm_target_count)==int(self.counts_per_rev)
        self.pending_arm_tilt=limit
        self.pending_arm_target = target
        self.arm_target_deadline = time.monotonic() + 1.0
        self.arm_target_status.set(f"已请求 {user_degrees(target,self.counts_per_rev):+d}°，等待 FPGA 确认")
        self.update_arm_target_buttons()
        self.write_queue.put(frame)

    def update_motor_readings(self):
        if self.latest_motor is None:
            return
        _, position, _velocity, pwm, _invalid, _flags, _reason = self.latest_motor
        self.motor_angle_value.set(f"{position * 360 / self.counts_per_rev:+.2f}°")
        self.pwm_value.set(f"{pwm / 10:+.1f} %")
        self.motor_count_value.set(f"编码器 {position:+d} x4计数")

    def _sync_touch_state(self, payload):
        sequence, action, result, run_flags, zero, target = decode_touch_state(payload)
        free_event = action == 0 and result in (6, 8)
        if free_event:
            self.status_text.set(("往复运动已开启：按板端已确认幅度和间隔切换" if run_flags & 8 else "往复运动已关闭，正在返回本次往复中心") if result == 6 else "往复未切换：需直立保持、位置外环有效且摆幅在行程内")
        running = bool(run_flags & 1)
        local_owner = running and not bool(run_flags & 2)
        self.touch_local_run = local_owner
        if action or local_owner:
            
            self.auto_start_pending = False
            self.remote_run_requested = bool(running and run_flags & 2)
        elif not self.auto_start_pending:
            self.remote_run_requested = bool(running and run_flags & 2)
        if action or local_owner or not self.auto_start_pending:
            self.manual_start = bool(run_flags & 4)
        if zero >= 16 and (action == 4 or self.pending_config is None):
            self.configured_zero = zero
            self.parameters["零点 ADC"].set(str(zero))
            self.zero_text.set(f"FPGA 电位器起点：ADC {zero}")
        if action >= 5 or self.pending_arm_target is None:
            self.arm_target_count = target
            self.arm_target_confirmed = True
            self.update_arm_target_display()
        labels = {1: "自动起摆", 2: "停机", 3: "手动起摆", 4: "设置直立零点",
                  5: "目标减小（屏幕步进）", 6: "目标增大（屏幕步进）", 7: "目标回到启动中心",
                  8: "反转一圈", 9: "正转一圈"}
        if action:
            if result in (1, 3, 5, 6):
                self.status_text.set(f"屏幕操作已同步：{labels.get(action, action)}")
                self.arm_target_status.set("FPGA 目标已同步")
            else:
                reason = {4: "需先直立 PID 接管", 7: "目标超出行程", 8: "当前条件不允许", 9: "无ADC数据"}.get(result, "请求未执行")
                self.status_text.set(f"屏幕操作未执行：{reason}")
            if action == 4 and result == 3:
                self.pending_config = None
                self.config_text.set(f"屏幕设置已同步：摆杆零点 ADC={zero}")
        if not running and not self.auto_start_pending:
            self.state_text.set("停机 · 已与 FPGA 同步")
            self._finish_log()
        elif local_owner:
            self.state_text.set("屏幕手动起摆：等待直立 PID 接管" if self.manual_start else "屏幕自动起摆")

    def set_pot_zero(self):
        if self.latest_code is None or time.monotonic() - self.latest_adc_at > 0.5:
            messagebox.showerror("电位器起点", "没有新鲜的有效 ADC 数据，无法设置起点。")
            return
        if not 16 <= self.latest_code <= 4079:
            messagebox.showerror("电位器起点", "当前 ADC 太接近量程边界，不能作为安全起点。")
            return
        self.parameters["零点 ADC"].set(str(self.latest_code))
        self.send_config()

    def capture_wrap_endpoint(self, side):
        if self.latest_code is None or time.monotonic() - self.latest_adc_at > 0.5:
            messagebox.showerror("换圈端点", "没有新鲜的有效 ADC 数据。")
            return
        (self.wrap_low_text if side == "low" else self.wrap_high_text).set(str(self.latest_code))

    def send_wrap_config(self):
        if not self.connected:
            messagebox.showerror("串口", "请先连接串口。")
            return
        if self.latest_motor is not None and self.latest_motor[-2] & 1:
            messagebox.showerror("换圈端点", "请先停机，再发送换圈端点。")
            return
        try:
            frame = make_wrap_config(int(self.wrap_low_text.get()),
                                     int(self.wrap_high_text.get()))
        except ValueError as exc:
            messagebox.showerror("换圈端点", str(exc))
            return
        self.pending_wrap = frame[4:-1]
        self.wrap_text.set("换圈端点已发送，等待 FPGA 回显")
        self.write_queue.put(frame)

    def adjust_gain(self, name, direction):
        try:
            value = float(self.parameters[name].get())
            step = float(self.gain_step.get())
            if not math.isfinite(value) or not math.isfinite(step) or step <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("PID 增益", "请先输入有效的数字增益。")
            return
        limit = {"Kp":512,"Ki":256,"Kd":8192,"转臂 Kp":16,"转臂 Kd":64}[name]
        value = max(-limit, min(limit, value + direction * step))
        scale = 4096 if name == "Ki" else 65536 if name in ("转臂 Kp","转臂 Kd") else 256
        value = round(value * scale) / scale
        self.parameters[name].set(f"{value:.12f}".rstrip("0").rstrip("."))
        if name in ("Kp","Ki","Kd") and self.live_pid_enabled.get():
            self.config_text.set("内环增益已修改，实时发送后等待确认")
        elif name in ("转臂 Kp","转臂 Kd") and self.live_pd_enabled.get():
            self.config_text.set("外环增益已修改，实时发送后等待确认")
        else:self.config_text.set("参数已修改；发送全部参数会停机")

    def _reset_live_pid(self,supported=None):
        self.live_pid_enabled.set(False)
        self.live_pid_desired=None
        self.live_pid_pending=None
        if supported is not None:self.live_pid_supported=supported

    def _live_running(self):
        return (self.connected and self.latest_motor is not None and
                time.monotonic()-self.latest_motor[0]<=0.5 and bool(self.latest_motor[-2]&2) and
                not self.auto_start_pending and self.pending_config is None)

    def _toggle_live_pid(self):
        if not self.live_pid_enabled.get():
            self.live_pid_desired=None
            self.live_pid_status.set("实时发送已关闭，已生效参数保留")
            return
        if not self.live_pid_supported:
            self.live_pid_enabled.set(False)
            self.live_pid_status.set("尚未确认实时调节功能；请烧录新版并重新连接")
            if self.connected:self.write_queue.put(make_query_command())
            return
        if not self._live_running():
            self.live_pid_enabled.set(False)
            self.live_pid_status.set("请先起摆并进入直立 PID，再开启实时调节")
            return
        self._queue_live_pid()

    def _slide_live_gain(self,key,value):
        if self.live_pid_syncing:return
        q=4096 if key=="Ki" else 256
        low,high=self.live_pid_bounds[key]
        v=round(max(low,min(high,float(value)))*q)/q
        self.parameters[key].set(f"{v:.12f}".rstrip("0").rstrip("."))

    def _live_gain_edited(self,key):
        if key in ("转臂 Kp","转臂 Kd"):
            self._outer_gain_edited(key)
            return
        try:v=float(self.parameters[key].get())
        except ValueError:return
        if not math.isfinite(v):return
        if not self.live_pid_syncing:
            self.live_pid_syncing=True
            self.live_pid_sliders[key].set(v)
            self.live_pid_syncing=False
        if self.live_pid_enabled.get():self._queue_live_pid()

    def _apply_live_range(self,key):
        try:
            low,high=(float(v.get()) for v in self.live_pid_ranges[key])
            limit={"Kp":512,"Ki":256,"Kd":8192}[key]
            if not (math.isfinite(low) and math.isfinite(high) and -limit<=low<high<=limit):raise ValueError
        except ValueError:
            self.live_pid_status.set(f"{key} 范围应在 ±{limit} 内，最小值小于最大值")
            return
        self.live_pid_bounds[key]=(low,high)
        self.live_pid_syncing=True
        self.live_pid_sliders[key].configure(from_=low,to=high)
        self.live_pid_sliders[key].set(float(self.parameters[key].get()))
        self.live_pid_syncing=False

    def _queue_live_pid(self):
        try:frame=make_live_pid(*(float(self.parameters[k].get()) for k in ("Kp","Ki","Kd")))
        except (ValueError,OverflowError) as exc:
            self.live_pid_desired=None
            self.live_pid_status.set(f"未发送：{exc}")
            return
        self.live_pid_desired=frame[4:-1]

    def _service_live_pid(self):
        now=time.monotonic()
        if self.live_pid_enabled.get() and not self._live_running():
            self._reset_live_pid()
            self.live_pid_status.set("实时发送已暂停：当前未处于直立 PID 运行")
            return
        if self.live_pid_pending is not None:
            if now-self.live_pid_sent_at>1.0:
                self._reset_live_pid()
                self.live_pid_status.set("实时 PID 确认超时，发送已关闭；正在查询实际参数")
                if self.connected:self.write_queue.put(make_query_command())
            return
        if not self.live_pid_enabled.get() or self.live_pid_desired is None or now<self.live_pid_next_at:return
        payload=self.live_pid_desired
        self.live_pid_desired=None
        if payload==self.live_pid_confirmed:return
        self.write_queue.put(make_live_pid(*(int.from_bytes(payload[i:i+4],"little",signed=True)/q for i,q in zip((0,4,8),(256,4096,256)))))
        self.live_pid_pending=payload
        self.live_pid_sent_at=now
        self.live_pid_next_at=now+0.1
        self.live_pid_status.set("PID 微调已发送，等待生效确认")

    def _live_pid_echo(self,value):
        try:status,gains=decode_live_pid(value)
        except ValueError:return
        self.live_pid_supported=True
        actual=value[1:]
        matched=self.live_pid_pending==actual
        if self.live_pid_pending is not None and (status!=0 or matched):self.live_pid_pending=None
        self.live_pid_confirmed=actual
        text="、".join(f"{k}={v:g}" for k,v in zip(("P","I","D"),gains))
        if status:
            self._reset_live_pid()
            self.live_pid_status.set(("未更新：当前未处于直立运行" if status==1 else "未更新：增益超出范围")+"；实际 "+text)
        else:
            self.live_pid_status.set("已确认："+text)
            if matched:
                self._confirm_log_parameter("last_live_pid",dict(zip(("Kp","Ki","Kd"),gains)))
                if self.log_writer:
                    self.log_writer.writerow((time.monotonic()-self.log_started,"pid")+("",)*10+gains)
                    self.log_file.flush()

    def _make_live_outer_widgets(self,parent):
        live=ttk.LabelFrame(parent,text="运动中调节外环 PD",padding=8)
        live.pack(fill="x",pady=(10,0));live.columnconfigure(1,weight=1)
        ttk.Checkbutton(live,text="开启外环实时调节",variable=self.live_pd_enabled,
                        command=self._toggle_live_pd).grid(row=0,column=0,columnspan=4,sticky="w")
        for index,key in enumerate(("转臂 Kp","转臂 Kd")):
            row=1+index*3;center=float(self.parameters[key].get())
            limit={"转臂 Kp":16,"转臂 Kd":64}[key];span={"转臂 Kp":0.25,"转臂 Kd":4}[key]
            low=max(-limit,center-span);high=min(limit,center+span)
            if center>=0:low=max(0,low)
            else:high=min(0,high)
            if low>=high:low,high=-limit,limit
            self.live_pd_bounds[key]=(low,high)
            ttk.Label(live,text=key).grid(row=row,column=0,sticky="w",pady=(7,0))
            ttk.Entry(live,textvariable=self.parameters[key],width=12).grid(row=row,column=1,columnspan=3,sticky="w",pady=(7,0))
            slider=ttk.Scale(live,from_=low,to=high,value=center,command=lambda value,k=key:self._slide_outer_gain(k,value))
            slider.grid(row=row+1,column=0,columnspan=4,sticky="ew",pady=3);self.live_pd_sliders[key]=slider
            lo=tk.StringVar(value=f"{low:g}");hi=tk.StringVar(value=f"{high:g}");self.live_pd_ranges[key]=(lo,hi)
            ttk.Label(live,text="范围").grid(row=row+2,column=0,sticky="w")
            ttk.Entry(live,textvariable=lo,width=7).grid(row=row+2,column=1,sticky="w")
            ttk.Entry(live,textvariable=hi,width=7).grid(row=row+2,column=2,sticky="w",padx=3)
            ttk.Button(live,text="应用",width=5,command=lambda k=key:self._apply_outer_range(k)).grid(row=row+2,column=3,sticky="e")
        ttk.Label(live,text="直立运行后开启；只调外环 P/D，不改变目标位置或外环周期。范围可缩小微调，P/D 均为 0 时关闭外环作用。",
                  wraplength=270,foreground="#596b7d").grid(row=7,column=0,columnspan=4,sticky="w",pady=(8,3))
        ttk.Label(live,textvariable=self.live_pd_status,wraplength=270).grid(row=8,column=0,columnspan=4,sticky="w")

    def _reset_live_pd(self,supported=None):
        self.live_pd_enabled.set(False)
        self.live_pd_desired=None
        self.live_pd_pending=None
        if supported is not None:self.live_pd_supported=supported

    def _toggle_live_pd(self):
        if not self.live_pd_enabled.get():
            self.live_pd_desired=None
            self.live_pd_status.set("实时发送已关闭，已生效参数保留")
            return
        if not self.live_pd_supported:
            self.live_pd_enabled.set(False)
            self.live_pd_status.set("尚未确认实时调节功能；请烧录新版并重新连接")
            if self.connected:self.write_queue.put(make_query_command())
            return
        if not self._live_running():
            self.live_pd_enabled.set(False)
            self.live_pd_status.set("请先起摆并进入直立 PID，再开启实时调节")
            return
        self._queue_live_pd()

    def _slide_outer_gain(self,key,value):
        if self.live_pd_syncing:return
        q=65536
        low,high=self.live_pd_bounds[key]
        v=round(max(low,min(high,float(value)))*q)/q
        self.parameters[key].set(f"{v:.12f}".rstrip("0").rstrip("."))

    def _outer_gain_edited(self,key):
        try:v=float(self.parameters[key].get())
        except ValueError:return
        if not math.isfinite(v):return
        if not self.live_pd_syncing:
            self.live_pd_syncing=True
            self.live_pd_sliders[key].set(v)
            self.live_pd_syncing=False
        if self.live_pd_enabled.get():self._queue_live_pd()

    def _apply_outer_range(self,key):
        try:
            low,high=(float(v.get()) for v in self.live_pd_ranges[key])
            limit={"转臂 Kp":16,"转臂 Kd":64}[key]
            if not (math.isfinite(low) and math.isfinite(high) and -limit<=low<high<=limit):raise ValueError
        except ValueError:
            self.live_pd_status.set(f"{key} 范围应在 ±{limit} 内，最小值小于最大值")
            return
        self.live_pd_bounds[key]=(low,high)
        self.live_pd_syncing=True
        self.live_pd_sliders[key].configure(from_=low,to=high)
        self.live_pd_sliders[key].set(float(self.parameters[key].get()))
        self.live_pd_syncing=False

    def _queue_live_pd(self):
        try:frame=make_live_pd(*(float(self.parameters[k].get()) for k in ("转臂 Kp","转臂 Kd")))
        except (ValueError,OverflowError) as exc:
            self.live_pd_desired=None
            self.live_pd_status.set(f"未发送：{exc}")
            return
        self.live_pd_desired=frame[4:-1]

    def _service_live_pd(self):
        now=time.monotonic()
        if self.live_pd_enabled.get() and not self._live_running():
            self._reset_live_pd()
            self.live_pd_status.set("实时发送已暂停：当前未处于直立 PID 运行")
            return
        if self.live_pd_pending is not None:
            if now-self.live_pd_sent_at>1.0:
                self._reset_live_pd()
                self.live_pd_status.set("实时 外环 PD 确认超时，发送已关闭；正在查询实际参数")
                if self.connected:self.write_queue.put(make_query_command())
            return
        if not self.live_pd_enabled.get() or self.live_pd_desired is None or now<self.live_pd_next_at:return
        payload=self.live_pd_desired
        self.live_pd_desired=None
        if payload==self.live_pd_confirmed:return
        self.write_queue.put(make_live_pd(*(int.from_bytes(payload[i:i+4],"little",signed=True)/q for i,q in zip((0,4),(65536,65536)))))
        self.live_pd_pending=payload
        self.live_pd_sent_at=now
        self.live_pd_next_at=now+0.1
        self.live_pd_status.set("外环 PD 微调已发送，等待生效确认")

    def _live_pd_echo(self,value):
        try:status,gains=decode_live_pd(value)
        except ValueError:return
        self.live_pd_supported=True
        actual=value[1:]
        matched=self.live_pd_pending==actual
        if self.live_pd_pending is not None and (status!=0 or matched):self.live_pd_pending=None
        self.live_pd_confirmed=actual
        text="、".join(f"{k}={v:g}" for k,v in zip(("P","D"),gains))
        if status:
            self._reset_live_pd()
            self.live_pd_status.set(("未更新：当前未处于直立运行" if status==1 else "未更新：增益超出范围")+"；实际 "+text)
        else:
            self.live_pd_status.set("已确认："+text)
            if matched:
                self._confirm_log_parameter("last_live_pd",dict(zip(("转臂 Kp","转臂 Kd"),gains)))
                if self.log_writer:
                    self.log_writer.writerow((time.monotonic()-self.log_started,"outer_pd")+("",)*13+gains)
                    self.log_file.flush()

    def send_config(self):
        if not self.connected:
            messagebox.showerror("串口", "请先连接串口。")
            return
        try:
            frame = self._config_frame()
            tilt = make_tilt_limit(int(self.tilt_limit_text.get()))
        except (ValueError, OverflowError) as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self._reset_live_pid()
        self._reset_live_pd()
        self.pending_config = frame[4:-1]
        self.pending_tilt = tilt[4:-1]
        self.remote_run_requested = False
        self.config_text.set("参数已发送，等待 FPGA 回显；电机将停机")
        self.write_queue.put(frame)
        self.write_queue.put(tilt)

    def send_tilt_limit(self):
        if not self.connected:
            messagebox.showerror("串口", "请先连接串口。")
            return
        if self.latest_motor is not None and self.latest_motor[-2] & 1:
            messagebox.showerror("目标倾角", "请先停机，再发送目标倾角限制。")
            return
        try:
            frame = make_tilt_limit(int(self.tilt_limit_text.get()))
        except ValueError as exc:
            messagebox.showerror("目标倾角", str(exc))
            return
        self.pending_tilt = frame[4:-1]
        self.tilt_text.set("目标倾角限制已发送，等待 FPGA 回显")
        self.write_queue.put(frame)

    def send_swing_config(self):
        if not self.connected:
            messagebox.showerror("串口", "请先连接串口。")
            return
        if self.latest_motor is not None and self.latest_motor[-2] & 1:
            messagebox.showerror("起摆参数", "请先停机，再发送起摆参数。")
            return
        try:
            frame = self._swing_frame()
        except (ValueError, OverflowError) as exc:
            messagebox.showerror("起摆参数", str(exc))
            return
        self.pending_swing = frame[4:-1]
        self.swing_text.set("起摆参数已发送，等待 FPGA 回显")
        self.write_queue.put(frame)

    def _pwm_limit_frame(self):
        return make_pwm_output_limit(float(self.pwm_limit_text.get()))

    def send_pwm_limit(self):
        if not self.connected:return
        try:frame=self._pwm_limit_frame()
        except (ValueError,OverflowError) as exc:
            messagebox.showerror("PWM总输出限幅",str(exc));return
        self.pending_pwm_limit=frame[4:-1];self.pwm_limit_deadline=time.monotonic()+3
        self.write_queue.put(frame);self.pwm_limit_status.set("已发送，等待FPGA确认")

    def _pwm_limit_echo(self,payload):
        try:status,codes=decode_pwm_output_limit(payload)
        except ValueError:return
        if status:
            self.pending_pwm_limit=None;self.auto_start_pending=False
            self.pwm_limit_status.set(f"FPGA拒绝设置，实际总限幅±{codes/10:g}%");return
        self.pwm_limit_status.set(f"FPGA确认：最终PWM总限幅±{codes/10:g}%")
        if payload[1:]==self.pending_pwm_limit:
            self.pending_pwm_limit=None
            self._confirm_log_parameter("pwm_output_limit_confirmed_per_mille",codes)

    def _tuning_frame(self):
        strength=("柔和","适中","较强").index(self.correction_strength_text.get())
        return make_correction_tuning(strength,float(self.correction_rate_text.get()),int(self.counts_per_rev))

    def send_correction_tuning(self):
        if not self.connected:return
        try:frame=self._tuning_frame()
        except (ValueError,OverflowError) as exc:
            messagebox.showerror("强度与限速",str(exc));return
        self.pending_tuning=frame[4:-1];self.tuning_deadline=time.monotonic()+3
        self.write_queue.put(frame);self.correction_tuning_status.set("已发送，等待FPGA确认；此按钮不会启用补偿")

    def _correction_tuning_echo(self,payload):
        try:status,strength,limit=decode_correction_tuning(payload)
        except ValueError:return
        if status:
            self.pending_tuning=None;self.auto_start_pending=False
            self.correction_tuning_status.set("FPGA拒绝设置：越界或当前状态不允许");return
        actual=limit*18000/(1040*256)
        self.correction_tuning_status.set(f"FPGA确认：{('柔和','适中','较强')[strength]}，内部修正上限约{actual:.2f}°/秒；仅连续档")
        if payload[1:]==self.pending_tuning:
            self.pending_tuning=None;self._confirm_log_parameter("correction_tuning_confirmed",{"strength":strength,"rate_q8":limit,"degrees_s":actual})

    def send_response_speed(self):
        if not self.connected:return
        frame=make_response_speed(self.response_mode_text.get()=="连续")
        self.pending_response=frame[4];self.response_deadline=time.monotonic()+3
        self.write_queue.put(frame)
        self.response_status.set("已发送响应速度，等待FPGA确认；保留已有补偿")

    def _response_speed_echo(self,payload):
        try:status,fast=decode_response_speed(payload)
        except ValueError:return
        if status:
            self.pending_response=None;self.auto_start_pending=False
            self.response_status.set("FPGA拒绝速度设置：需停机或直立运行" if status==2 else "FPGA拒绝无效速度档位")
            return
        self.response_status.set("FPGA连续档：使用下方已确认的强度和限速" if fast else "FPGA原速档：等待1s、均值1s、每0.2s修正")
        if self.pending_response==int(fast):
            self.pending_response=None;self._confirm_log_parameter("target_response_fast_confirmed",fast)

    def _trim_frame(self):
        return make_target_trim(int(self.counts_per_rev),float(self.tolerance_text.get()),self.target_trim_enabled.get(),float(self.target_trim_max_text.get()))

    def _board_frame(self):
        
        if int(self.counts_per_rev)!=1040:raise ValueError("本硬件工程每圈1040计数；修改硬件参数需重编译")
        self.tilt_limit_text.set(self.small_step_limit_text.get())
        return make_board_config(int(self.counts_per_rev),float(self.board_amplitude_text.get()),float(self.board_interval_text.get()),int(self.small_step_limit_text.get()),int(self.large_step_limit_text.get()),int(self.reverse_turn_limit_text.get()))

    def _step_limit(self,degrees):
        normal,forward,reverse=(int(v.get()) for v in (self.small_step_limit_text,self.large_step_limit_text,self.reverse_turn_limit_text))
        return turn_tilt_limit(degrees,normal,forward,reverse)

    def send_position_tolerance(self):
        if not self.connected:return
        try:frame=self._trim_frame()
        except (ValueError,OverflowError) as exc:
            messagebox.showerror("目标补偿",str(exc));return
        self.pending_tolerance=frame[4:-1]
        self.tolerance_deadline=time.monotonic()+3
        self.write_queue.put(frame)
        self.tolerance_status.set("已发送，等待FPGA确认；运行中只允许改容差")

    def _position_tolerance_echo(self,payload):
        try:tolerance,maximum,enabled,status=decode_target_trim(payload)
        except ValueError:return
        if status:
            self.pending_tolerance=None;self.auto_start_pending=False
            self.tolerance_status.set("FPGA拒绝：范围错误" if status==1 else "需停机设置开关/最大补偿范围，或尚未配置")
            return
        self.tolerance_status.set(f"FPGA已确认：容差±{tolerance*360/(1040*256):g}°，最大补偿±{maximum*360/1040:.2f}°，{'开启' if enabled else '关闭'}")
        if payload[:8]==self.pending_tolerance:
            self.pending_tolerance=None
            self._confirm_log_parameter("target_trim_echo",payload.hex())

    def send_board_config(self):
        if not self.connected:return
        try:frame=self._board_frame()
        except (ValueError,OverflowError) as exc:
            messagebox.showerror("三种倾角及往复设置",str(exc));return
        self.pending_board=frame[4:-1];self.board_deadline=time.monotonic()+3
        self.write_queue.put(frame)
        self.board_status_text.set("已发送，等待FPGA实际设置确认")

    def _board_echo(self,payload):
        try:counts,amplitude,interval,limit,status,flags=decode_board_echo(payload)
        except ValueError:return
        if status == 0:
            enabled = bool(flags & 2)
            if enabled and not self.reference_reciprocal:
                self.reference_center_degrees = None  
            self.reference_reciprocal = enabled
        state="行程阻挡" if flags&8 else "往复运行" if flags&4 else "往复等待" if flags&2 else "往复关闭"
        self.board_status_text.set(f"FPGA：{state}；幅度±{amplitude*360/counts:.2f}°，间隔{interval/1000:g}s" if status==0 else "FPGA拒绝设置：参数或运行状态不允许")
        if status==0 and len(payload)==18:
            self.triplet_settings_confirmed=True
            self.confirmed_tilt_profiles=tuple(int.from_bytes(payload[i:i+2],"little") for i in (12,14,16))
            kind=(flags>>5)&3;normal_recovery=kind==0 and bool(flags&128)
            active=max(self.confirmed_tilt_profiles[0],40) if normal_recovery else self.confirmed_tilt_profiles[0 if kind==3 else kind]
            motion_name=('摆杆恢复档' if normal_recovery else '平时','正转一圈','反转一圈','反转180°' if flags&1 else '正转180°')[kind]
            phase=motion_name+('·限速减速' if flags&128 else '·低速到位确认') if kind else motion_name
            self.tilt_profile_status.set(f"FPGA当前：{phase}；倾角上限±{active} ADC码")
            phase_key=(kind,bool(flags&128) and kind!=0,bool(flags&1),normal_recovery,self.confirmed_tilt_profiles)
            if phase_key!=self.last_tilt_phase_logged:
                self.last_tilt_phase_logged=phase_key
                self._confirm_log_parameter("tilt_motion_phase",{"kind":kind,"forward_ramping":bool(flags&128) and kind in (1,3) and not (kind==3 and flags&1),"motion_ramping":bool(flags&128) and kind!=0,"normal_recovery_active":normal_recovery,"half_turn_negative":bool(kind==3 and flags&1),"limits":self.confirmed_tilt_profiles})
        elif len(payload)!=18:
            self.triplet_settings_confirmed=False;self.confirmed_tilt_profiles=None;self.last_tilt_phase_logged=None
            self.tilt_profile_status.set("当前回显为旧两档格式，三种倾角设置尚未确认")
        if status:
            self.pending_board=None;self.auto_start_pending=False
        elif self.pending_board is not None and len(payload)==18 and payload[:10]+payload[12:18]==self.pending_board:
            self.pending_board=None
            self._confirm_log_parameter("board_config_echo",payload.hex())

    def set_reciprocal(self,enabled):
        if self.connected:
            self.write_queue.put(make_reciprocal_command(enabled))
            self.board_status_text.set("往复开关已请求，等待FPGA状态")

    # 状态数据仅用于界面与日志，实测角度始终来自板端真实编码器数据。
    def _monitor_echo(self,timestamp,payload):
        try:r=decode_monitor(payload)
        except ValueError:return
        self.latest_monitor=r
        if "reciprocal_enabled" in r:
            self.reference_reciprocal = r["reciprocal_enabled"]
            self.reference_center_degrees = r["reciprocal_center_degrees"]
        self.latest_monitor_at=timestamp
        self.three_angles_text.set(f"用户目标：{user_degrees(r['user_counts'],r['counts_per_rev']):+d}°\n内部目标：{r['internal_degrees']:+.2f}°\n实际角度：{r['actual_degrees']:+.2f}°")
        self.trim_runtime_text.set(trim_runtime_text(r))
        self._log_row((f"{timestamp-self.log_started:.6f}","dual_target",r['adc_code'],0,r['actual_counts'],"",r['pwm_per_mille'],"",r['flags'],"","",r['user_counts'],"","","","","",r['internal_counts'],r['trim_state'],r['velocity_q8'],r['counts_per_rev'],r.get('reciprocal_center_counts','')))

    def send_deadzone(self):
        if not self.connected:
            messagebox.showerror("串口","请先连接串口。")
            return
        if self.auto_start_pending or self.remote_run_requested or (self.latest_motor is not None and self.latest_motor[-2]&1):
            messagebox.showerror("PWM 死区","请先停机，再修改死区补偿。")
            return
        try:frame=make_pwm_deadzone(int(self.deadzone_text.get()))
        except (ValueError,OverflowError) as exc:
            messagebox.showerror("PWM 死区",str(exc))
            return
        self.pending_deadzone=frame[4:-1]
        self.deadzone_status.set("已发送 PWM 死区补偿，等待 FPGA 回显")
        self.write_queue.put(frame)

    def send_outer_period(self):
        if not self.connected:
            messagebox.showerror("串口","请先连接串口。")
            return
        if self.remote_run_requested or (self.latest_motor is not None and self.latest_motor[-2]&1):
            messagebox.showerror("外环周期","请先停机，再修改周期。")
            return
        try:frame=make_outer_period(float(self.outer_period_ms_text.get()))
        except ValueError as exc:
            messagebox.showerror("外环周期",str(exc))
            return
        self.pending_outer_period=frame[4:-1]
        self.outer_period_text.set("已发送外环周期，等待 FPGA 回显")
        self.write_queue.put(frame)

    # 启动按参数发送、逐项回显确认、执行运行命令的顺序进行。
    def start_system(self, manual=False):
        if not self.connected:
            messagebox.showerror("串口", "请先连接串口。")
            return
        if self.auto_start_pending or self.remote_run_requested:
            return
        if self.latest_motor is None or not self.latest_motor[-2] & 8:
            messagebox.showerror("启动", "请先确认 ADC 有新鲜有效读数。")
            return
        if self.latest_motor[-2] & 1 or self.latest_motor[-2] & 4:
            messagebox.showerror("启动", "故障已锁定，请先点击停机，再启动。")
            return
        try:
            if all(round(float(self.parameters[name].get()) * scale) == 0
                   for name, scale in (("Kp", 256), ("Ki", 4096), ("Kd", 256))):
                raise ValueError("直立 PID 三项全为零，无法在最高点保持；请先完成直立调参")
            config = self._config_frame()
            swing = self._swing_frame() if not manual else None
            period = make_control_period(1.0)
            outer_period = make_outer_period(float(self.outer_period_ms_text.get()))
            wrap = make_wrap_config(int(self.wrap_low_text.get()),
                                    int(self.wrap_high_text.get()))
            tilt = make_tilt_limit(int(self.tilt_limit_text.get()))
            tolerance=self._trim_frame()
            board=self._board_frame()
            tuning=self._tuning_frame()
            pwm_limit=self._pwm_limit_frame()
            response=make_response_speed(self.response_mode_text.get()=="连续")
            deadzone=make_pwm_deadzone(int(self.deadzone_text.get()))
            self._start_log()
        except (ValueError, OverflowError, OSError) as exc:
            messagebox.showerror("启动", str(exc))
            return
        self._reset_live_pid()
        self._reset_live_pd()
        self.pending_config, self.pending_swing = config[4:-1], swing[4:-1] if swing else None
        self.pending_period, self.pending_wrap = period[4:-1], wrap[4:-1]
        self.pending_outer_period=outer_period[4:-1]
        self.pending_tilt = tilt[4:-1]
        self.pending_pwm_limit=pwm_limit[4:-1];self.pwm_limit_deadline=time.monotonic()+5
        self.pending_tuning=tuning[4:-1];self.tuning_deadline=time.monotonic()+5
        self.pending_response=response[4];self.response_deadline=time.monotonic()+5
        self.pending_tolerance=tolerance[4:-1]
        self.pending_board=board[4:-1]
        self.board_deadline=time.monotonic()+5
        self.tolerance_deadline=time.monotonic()+5
        self.pending_deadzone=deadzone[4:-1]
        self.manual_start=bool(manual)
        self.auto_start_pending = True
        self.auto_start_deadline = time.monotonic() + 5
        for frame in (config, pwm_limit, wrap, period, outer_period, swing, tilt, deadzone, tolerance, board, response, tuning):
            if frame is not None:self.write_queue.put(frame)
        self.status_text.set("已发送全部参数，等待 FPGA 回显后自动启动")

    def stop_system(self):
        self._reset_live_pid()
        self._reset_live_pd()
        self.remote_run_requested = False
        self.auto_start_pending = False
        self._finish_log()
        if self.connected:
            self.write_queue.put(make_run_command(False))
            self.status_text.set("停机请求已发送，等待 FPGA 状态")

    def renew_remote_run(self):
        if self.connected and self.remote_run_requested:
            self.write_queue.put(make_run_command(True,manual=self.manual_start))
        self.root.after(250, self.renew_remote_run)

    def clear_samples(self):
        self.adc_samples.clear()
        self.motor_samples.clear()
        self.angle_reference_history.clear()

    # 参考线按生效时间保留历史；往复开启时使用板端中心，不替代为两端目标。
    def record_angle_reference(self, now):
        label = "往复中心" if self.reference_reciprocal else "用户目标"
        value = None
        if self.connected:
            if self.reference_reciprocal:
                value = self.reference_center_degrees
            elif self.arm_target_confirmed:
                value = self.user_target_degrees()
        history = self.angle_reference_history
        
        if history:
            now = max(now, history[-1][0])
        if not history or (value, label) != history[-1][1:]:
            history.append((now, value, label))
        return value, label

    def redraw_plots(self):
        now = time.monotonic()
        span = int(self.window_text.get().split()[0])
        reference, reference_label = self.record_angle_reference(now)
        self.adc_plot.redraw(((t, code / 4096 * self.full_scale)
                              for t, code in self.adc_samples),
                             span, now, (0, self.full_scale))
        self.angle_plot.redraw(((t, position * 360 / self.counts_per_rev)
                                for t, position, _ in self.motor_samples), span, now,
                               reference=reference, reference_label=reference_label,
                               reference_history=self.angle_reference_history, show_zero=False)
        self.pwm_plot.redraw(((t, pwm / 10)
                              for t, _, pwm in self.motor_samples), span, now)
        if self._angle_fullscreen is not None:
            self._angle_fullscreen.render(now)
        self.root.after(100, self.redraw_plots)

    def open_angle_fullscreen(self, event=None):
        if self._angle_fullscreen is None:
            self._angle_fullscreen = AngleFullscreen(self, TracePlot)
        else:
            self._angle_fullscreen.top.lift()
            self._angle_fullscreen.top.focus_set()
        return "break"

    def _refresh_target_readout(self):
        now=time.monotonic()
        if self.connected and self.arm_target_confirmed:
            self.user_angle_value.set(f"{self.user_target_degrees():+d}°")
        else:self.user_angle_value.set("—")
        if self.connected and self.latest_motor is not None and now-self.latest_motor[0]<1:
            actual=self.latest_motor[1]*360/self.counts_per_rev
            self.actual_angle_value.set(f"{actual:+.2f}°")
            self.position_error_value.set(f"{self.user_target_degrees()-actual:+.2f}°" if self.arm_target_confirmed else "—")
        else:
            self.actual_angle_value.set("—");self.position_error_value.set("—")
        age=now-self.latest_monitor_at if self.latest_monitor_at else float('inf')
        if self.connected and self.latest_monitor is not None and age<1:
            self.internal_angle_value.set(f"{self.latest_monitor['internal_degrees']:+.2f}°")
            self.monitor_age_text.set(f"内部状态回传：{age:.1f}秒前")
            self.trim_runtime_text.set(trim_runtime_text(self.latest_monitor))
        else:
            self.internal_angle_value.set("—")
            self.monitor_age_text.set("内部状态回传已过期" if self.latest_monitor is not None else "等待内部状态回传")
            self.trim_runtime_text.set("内部回传无效："+self.monitor_decode_error if self.monitor_decode_error else "补偿状态未知；不能用旧状态判断是否正在补偿")
            if self.connected and not self.disconnecting and now-self.monitor_request_at>=1:
                self.write_queue.put(make_monitor_subscription(True));self.monitor_request_at=now

    # 正常断开时先移交板端运行控制权，收到确认后才关闭串口。
    def _begin_disconnect(self):
        if self.disconnecting:return
        self._reset_live_pid();self._reset_live_pd()
        self.remote_run_requested=False;self.auto_start_pending=False
        self.disconnecting=True;self.connected=False;self.handoff_result=None
        self.connect_button.configure(text="交接中",state="disabled")
        self.status_text.set("正在请求板端继续运行并释放串口…")
        self.stop_event.set()

    def close(self):
        self.close_pending=True
        if self.stop_event is not None:self._begin_disconnect()
        else:self._finish_log();self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    PendulumWindow(root)
    root.mainloop()


