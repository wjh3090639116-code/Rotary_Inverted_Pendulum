"""旋臂角度全屏显示：复用主窗口数据与连接，保持真实角度和参考线历史。"""
import math
import time
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont

def user_degrees(counts, counts_per_rev):
    value = counts * 360 / counts_per_rev
    return math.floor(value+.5) if value >= 0 else -math.floor(-value+.5)

def initial_view(values):
    """根据可见角度与参考值确定初始坐标范围，最小显示跨度为１８０度。"""
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return 0.0, 90
    low, high = min(values), max(values)
    half = 90
    while True:
        grid = half / 3
        center = round((low + high) / 2 / grid) * grid
        if low >= center-half+half*.1 and high <= center+half-half*.1:
            return center, half
        half *= 2

class AdaptiveAngleRange:
    def __init__(self):
        self.bounds = None
        self.shrink_since = None

    def update(self, values, now):
        values = [v for v in values if math.isfinite(v)]
        center, half = initial_view(values)
        wanted = center-half, center+half
        if self.bounds is None:
            self.bounds = wanted
        low, high = self.bounds
        margin = (high-low)*.04
        if values and (min(values) < low+margin or max(values) > high-margin):
            
            if 2*half < high-low:
                middle = round((min(values)+max(values))/2 / ((high-low)/6))*((high-low)/6)
                wanted = middle-(high-low)/2, middle+(high-low)/2
            self.bounds = wanted
            self.shrink_since = None
        elif 2*half < high-low:
            if self.shrink_since is None:self.shrink_since = now
            if now-self.shrink_since >= 3:
                self.bounds = wanted
                self.shrink_since = None
        else:self.shrink_since = None
        return self.bounds

def clipped_trace_runs(points, top, bottom):
    """在绘图区上下边缘裁剪折线，保留交点并分离越界线段，不伪造边界处的平直采样。"""
    runs = []
    run = []
    for i in range(2, len(points), 2):
        ax, ay, bx, by = points[i-2:i+2]
        if (ay < top and by < top) or (ay > bottom and by > bottom):
            if run:
                runs.append(run)
                run = []
            continue
        if by != ay:
            t0, t1 = sorted(((top-ay)/(by-ay), (bottom-ay)/(by-ay)))
            t0, t1 = max(0, t0), min(1, t1)
            sx, sy = ax+(bx-ax)*t0, ay+(by-ay)*t0
            ex, ey = ax+(bx-ax)*t1, ay+(by-ay)*t1
        else:
            sx, sy, ex, ey = ax, ay, bx, by
        sy, ey = min(bottom, max(top, sy)), min(bottom, max(top, ey))
        if run and abs(run[-2]-sx) < 1e-6 and abs(run[-1]-sy) < 1e-6:
            run.extend((ex, ey))
        else:
            if run:
                runs.append(run)
            run = [sx, sy, ex, ey]
    if run:
        runs.append(run)
    return runs

def reference_segments(history, start, end):
    """按参考值生效时间生成分段保持线，保留窗口左边界之前最后一个有效值。
    空值表示数据缺失，不补画首个有效参考之前的时间段。"""
    result = []
    events = list(history)
    for i, (timestamp, value, label) in enumerate(events):
        finish = events[i+1][0] if i+1 < len(events) else end
        a, b = max(start, timestamp), min(end, finish)
        if value is not None and b > a:
            result.append((a, b, value, label))
    return result

class AngleFullscreen:
    def __init__(self, owner, plot_class, *, fullscreen=True):
        self.owner = owner
        self.top = tk.Toplevel(owner.root)
        self.top.title("旋臂角度 · 全屏监测")
        self.top.configure(bg="white")
        self.top.minsize(900, 650)
        if fullscreen:
            self.top.attributes("-fullscreen", True)
        else:
            self.top.geometry("1280x800+20+20")
        self.top.protocol("WM_DELETE_WINDOW", self.close)
        self.top.bind("<Escape>", self.close)
        self.top.columnconfigure(0, weight=1)
        self.top.rowconfigure(0, weight=1)
        self.axis = AdaptiveAngleRange()
        self.live_text = tk.StringVar(value="等待数据")
        self.plot = plot_class(self.top, "", owner.angle_plot.color, "°")
        style = ttk.Style(self.top)
        style.configure("AngleFullscreen.TLabelframe", background="white")
        style.configure("AngleFullscreen.TLabelframe.Label", background="white", foreground="#627282")
        self.plot.frame.configure(style="AngleFullscreen.TLabelframe")
        self.plot.frame.grid(row=0, column=0, sticky="nsew", padx=24, pady=(20, 0))
        self.plot.clip_values = True
        self.plot.show_reference_label = False
        self.plot.grid_divisions = 6
        self.plot.tick_font = ("Segoe UI", 11)
        self.plot.canvas.bind("<Double-Button-1>", self.close)

        footer = tk.Frame(self.top, bg="white")
        footer.grid(row=1, column=0, sticky="ew", padx=40, pady=(20, 30))
        readouts = tk.Frame(footer, bg="white")
        readouts.pack(fill="x")
        self.readouts = readouts
        self.number_font = tkfont.Font(root=self.top, family="Segoe UI", size=-150, weight="bold")
        self._font_pixels = 150
        self.value_labels = []
        for index, (caption, var, color) in enumerate((
                ("实测角度", owner.actual_angle_value, owner.angle_plot.color),
                ("目标角度", owner.user_angle_value, "#2f6fa5"),
                ("误差角度", owner.position_error_value, "#263746"))):
            group = tk.Frame(readouts, bg="white")
            group.grid(row=0, column=index, sticky="ew", padx=(0, 16))
            readouts.columnconfigure(index, weight=1, uniform="angle")
            tk.Label(group, text=caption, bg="white", fg="#627282",
                     font=("Microsoft YaHei UI", 18)).pack(anchor="w")
            label = tk.Label(group, textvariable=var, bg="white", fg=color,
                             font=self.number_font, anchor="w")
            label.pack(anchor="w")
            self.value_labels.append(label)

        self.top.bind("<Configure>", self.fit_numbers)
        self.render()
        self.top.after_idle(self.top.focus_set)

    def _span(self):
        return int(self.owner.window_text.get().split()[0])

    def _samples(self):
        cpr = self.owner.counts_per_rev
        return [(t, count * 360 / cpr) for t, count, _ in self.owner.motor_samples]

    def fit_numbers(self, event=None):
        width = self.top.winfo_width()
        pixels = max(54, min(180, int((width-128)/3/4.8), int(self.top.winfo_height()*.18)))
        if pixels != self._font_pixels:
            self._font_pixels = pixels
            self.number_font.configure(size=-pixels)

    def render(self, now=None):
        if not self.top.winfo_exists():
            return
        now = time.monotonic() if now is None else now
        owner = self.owner
        samples = self._samples()
        target, reference_label = owner.record_angle_reference(now)
        values = [v for t, v in samples if now-self._span() <= t <= now]
        values.extend(v for _, _, v, _ in reference_segments(owner.angle_reference_history, now-self._span(), now))
        if target is not None:values.append(target)
        self.bounds = self.axis.update(values, now)
        self.plot.redraw(samples, self._span(), now, self.bounds,
                         reference=target, show_zero=False, reference_history=owner.angle_reference_history, reference_label=reference_label)
        fresh = owner.latest_motor is not None and now-owner.latest_motor[0] < 1
        self.live_text.set("" if owner.connected and fresh else
                           "数据已过期" if owner.connected else "未连接")
        if self.live_text.get():
            self.plot.canvas.create_text(self.plot.canvas.winfo_width()-24, 24,
                anchor="ne", text=self.live_text.get(), fill="#8a939c", font=("Microsoft YaHei UI", 11))

    def close(self, event=None):
        self.owner._angle_fullscreen = None
        self.top.destroy()
        return "break"
