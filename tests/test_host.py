"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
import importlib.machinery
import importlib.util
import json
import struct
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from adc_protocol import *
from native_profile import migrate_profile

class HostTests(unittest.TestCase):
    def test_unsigned_high12_and_voltage(self):
        for code in (0,1,2047,2048,2640,3000,4095):
            self.assertEqual(decode_adc(struct.pack('<HB',code,0)),(code,0))
        for code in (4096,65535):
            with self.assertRaises(ValueError):decode_adc(struct.pack('<HB',code,0))
        self.assertAlmostEqual(2640*5.12/4096,3.3)

    def test_configs_over_old_signed_limit(self):
        f=make_config(3000,1,-1,61,301,2000,8,.1,387,.09,6.4)
        self.assertEqual(f[2:4],b'\x32\x1e')
        self.assertEqual(decode_config_echo(f[4:-1])[0],3000)
        for code in (0,2048,4095):
            f=make_swing_config(600,250,code,code,code,32,160,2600,500,350)
            self.assertEqual(decode_swing_echo(f[4:-1])[2:5],(code,)*3)
        for zero in (15,4080):
            with self.assertRaises(ValueError):make_config(zero,1,1,61,301,2000,1,0,0,0,0)
        with self.assertRaises(ValueError):make_swing_config(600,250,4096,1,2,32,160,2600)
        self.assertEqual(make_wrap_config(0,2640)[4:8],struct.pack('<HH',0,2640))
        make_wrap_config(499,4095);make_tilt_limit(102)
        for lo,hi in ((500,2640),(0,999),(0,4096)):
            with self.assertRaises(ValueError):make_wrap_config(lo,hi)
        with self.assertRaises(ValueError):make_tilt_limit(103)

    def test_packaged_profiles_encode(self):
        for path in (ROOT/'tools').glob('*.json'):
            d=json.loads(path.read_text(encoding='utf-8'));b=d['balance'];s=d['swing']
            self.assertEqual(d,migrate_profile(d))
            make_config(*(int(b[k]) for k in ('零点 ADC','角度方向','编码器方向','捕获窗口','跌倒门限','行程门限')),
                        *(float(b[k]) for k in ('Kp','Ki','Kd','转臂 Kp','转臂 Kd')))
            make_swing_config(*(int(s[k]) for k in ('pump_pwm','pulse_ms','bottom_code','target_code','upper_code','bottom_window','rearm_distance','travel','stage2_pwm','stage2_ms')))
            make_wrap_config(int(d['wrap_low']),int(d['wrap_high']))
            make_tilt_limit(int(d['tilt_limit']))

    def test_physical_control_uart(self):
        path=ROOT/'sim/control_uart_capture.hex'
        if not path.exists():self.skipTest('Run tb_ads8688_control first')
        raw=bytes(int(line,16) for line in path.read_text().splitlines())
        packets=FrameDecoder().feed(raw)
        self.assertTrue(any(k==0x33 and decode_config_echo(p)[0]==3000 for k,p in packets))
        self.assertTrue(any(k==0x34 and decode_adc(p)==(3000,0) for k,p in packets))
        self.assertTrue(any(k==0x35 and len(p)==18 for k,p in packets))
        self.assertFalse(any(k in (0x23,0x24,0x25) for k,p in packets))

    def test_gui_offline(self):
        import tkinter as tk
        loader=importlib.machinery.SourceFileLoader('adc_gui_test',str(ROOT/'tools/adc_gui.pyw'))
        spec=importlib.util.spec_from_loader(loader.name,loader)
        gui=importlib.util.module_from_spec(spec);loader.exec_module(gui)
        gui.clean_history=lambda *args:(0,[])
        root=tk.Tk();root.withdraw()
        original_geometry=root.geometry
        root.geometry=lambda size=None:original_geometry(size+'+30000+30000') if size else original_geometry()
        original_state=root.state
        root.state=lambda state=None:original_state() if state is None else None
        app=gui.PendulumWindow(root)
        try:
            self.assertEqual(app.full_scale,5.12)
            self.assertEqual(app.parameters['零点 ADC'].get(),json.loads((ROOT/'tools/tuned_profile.json').read_text(encoding='utf-8'))['balance']['零点 ADC'])
            self.assertEqual(app.wrap_high_text.get(),'2640')
            self.assertEqual(app._config_frame()[2],0x32)
            self.assertIn('ADS8688',root.title())
            app.latest_code=3000
            self.assertTrue(app.apply_full_scale())
            self.assertIn('3.750',app.adc_value.get())
            import time
            app.events.put((app.session,'adc',[(time.monotonic(),4095,0)]))
            app.poll_events()
            self.assertEqual(app.latest_code,4095)
            self.assertIn('5.119',app.adc_value.get())
            app._swing_frame()
        finally:
            root.destroy()

if __name__=='__main__':unittest.main()
