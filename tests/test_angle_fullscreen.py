"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
from collections import deque
import importlib.machinery
import importlib.util
import math
import queue
import struct
import sys
import time
import tkinter as tk
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from angle_fullscreen import AngleFullscreen, initial_view, clipped_trace_runs, AdaptiveAngleRange, user_degrees

def load_gui():
    loader=importlib.machinery.SourceFileLoader('fullscreen_gui',str(ROOT/'tools/adc_gui.pyw'))
    mod=importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name,loader))
    loader.exec_module(mod)
    return mod

class AngleViewTests(unittest.TestCase):
    def test_range_selection_and_clipping(self):
        self.assertEqual(initial_view([]),(0,90))
        center,half=initial_view([-240,0,360,float('nan')])
        self.assertLessEqual(center-half,-240)
        self.assertGreaterEqual(center+half,360)
        self.assertEqual(clipped_trace_runs([0,-5,10,-1],0,10),[])
        self.assertEqual(clipped_trace_runs([0,-10,20,10,40,30],0,20),[[10,0,20,10,30,20]])
        self.assertEqual(clipped_trace_runs([0,5,10,-5,20,5],0,10),[[0,5,5,0],[15,0,20,5]])

    def test_axis_floor_hysteresis_and_integer_nominal(self):
        axis=AdaptiveAngleRange();base=axis.update([59.5,60.5,60],0)
        for i in range(100):
            self.assertEqual(axis.update([60+math.sin(i)*3,60],i*.1),base)
        expanded=axis.update([-360,0,240],10)
        self.assertLessEqual(expanded[0],-360);self.assertGreaterEqual(expanded[1],240)
        axis.update([239.5,240.5,240],11)
        shrunk=axis.update([239.5,240.5,240],15)
        self.assertGreaterEqual(shrunk[1]-shrunk[0],180)
        self.assertEqual(user_degrees(173,1040),60)
        self.assertEqual(user_degrees(-173,1040),-60)
        self.assertEqual(user_degrees(693,1040),240)
        self.assertEqual(user_degrees(1040,1040),360)

    def test_clean_fullscreen_tracks_large_motion_without_magnifying_noise(self):
        g=load_gui();root=tk.Tk();root.withdraw()
        root.state=lambda *args:None;root.geometry=lambda *args:None
        app=g.PendulumWindow(root)
        try:
            now=time.monotonic()
            app.connected=True;app.arm_target_confirmed=True;app.arm_target_count=173
            app.latest_motor=(now,169,0,0,0,0x3b,0)
            app.motor_samples=deque([(now-.5,0,0),(now,169,0)])
            app.latest_monitor_at=now;app.monitor_request_at=now
            app._refresh_target_readout()
            while not app.write_queue.empty():app.write_queue.get_nowait()
            self.assertEqual(app.angle_plot.color,'#8556af')
            self.assertEqual(app.pwm_plot.color,'#dc6b1d')
            self.assertTrue(app.angle_plot.canvas.bind('<Double-Button-1>'))
            self.assertFalse(app.pwm_plot.canvas.bind('<Double-Button-1>'))
            app.open_angle_fullscreen()
            view=app._angle_fullscreen;root.update_idletasks()
            self.assertTrue(view.top.attributes('-fullscreen'))
            bounds=view.bounds
            view.render(now)
            self.assertEqual(view.live_text.get(),'')
            self.assertEqual(app.user_angle_value.get(),'+60°')
            self.assertGreaterEqual(view.bounds[1]-view.bounds[0],180)
            self.assertFalse(any(isinstance(w,(__import__('tkinter').ttk.Button,__import__('tkinter').ttk.Entry,__import__('tkinter').ttk.Combobox)) for w in view.top.winfo_children()))
            self.assertEqual(str(view.value_labels[0]['textvariable']),str(app.actual_angle_value))
            self.assertEqual(str(view.value_labels[1]['textvariable']),str(app.user_angle_value))
            self.assertEqual(str(view.value_labels[2]['textvariable']),str(app.position_error_value))
            self.assertEqual(app.position_error_value.get(),'+1.50°')
            app.arm_target_count=1040;app.motor_samples.append((now+.1,2080,0))
            view.render(now+.1)
            self.assertNotEqual(view.bounds,bounds)
            self.assertLessEqual(view.bounds[0],0)
            self.assertGreaterEqual(view.bounds[1],720)
            for item in view.plot.canvas.find_withtag('trace'):
                coords=view.plot.canvas.coords(item)
                self.assertTrue(all(12<=y<=view.plot.canvas.winfo_height()-27 for y in coords[1::2]))
            view.render(now+2);self.assertEqual(view.live_text.get(),'数据已过期')
            app.latest_motor=(now-2,169,0,0,0,0x3b,0);app._refresh_target_readout()
            self.assertEqual(app.actual_angle_value.get(),'—')
            view.close();self.assertIsNone(app._angle_fullscreen)
            self.assertTrue(app.connected);self.assertTrue(app.write_queue.empty())
        finally:root.destroy()

    def test_repeated_integer_user_steps_do_not_accumulate_quantization(self):
        g=load_gui();root=tk.Tk();root.withdraw()
        root.state=lambda *a:None;root.geometry=lambda *a:None
        app=g.PendulumWindow(root)
        try:
            app.connected=True;app.wide_arm_firmware=True;app.arm_target_confirmed=True
            app.configured_travel=2000;app.arm_target_count=0;app.arm_step_text.set('10')
            wanted=0
            for direction in [1]*14+[-1]*28+[1]*14:
                app.latest_motor=(time.monotonic(),0,0,0,0,0x3b,0)
                app.shift_arm_target(direction)
                frame=app.write_queue.get_nowait()
                self.assertEqual(frame[2],0x1b)
                target=struct.unpack('<i',frame[4:8])[0]
                app.arm_target_count=target;app.pending_arm_target=None
                wanted+=direction*10
                self.assertEqual(app.user_target_degrees(),wanted)
            self.assertEqual(app.arm_target_count,0)
        finally:root.destroy()

if __name__=='__main__':unittest.main()
