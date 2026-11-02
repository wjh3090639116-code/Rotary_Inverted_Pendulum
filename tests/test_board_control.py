"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import sys,struct,unittest,importlib.util,importlib.machinery,json,time,queue
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import adc_protocol as p

def load(name,path):
    loader=importlib.machinery.SourceFileLoader(name,str(path))
    spec=importlib.util.spec_from_loader(name,loader)
    m=importlib.util.module_from_spec(spec);loader.exec_module(m);return m

class BoardControlTests(unittest.TestCase):
    def test_protocol_ranges(self):
        for degrees,expected in ((179.99,22),(180,44),(-179.99,22),(-180,44),(360,44)):
            self.assertEqual(p.step_tilt_limit(degrees,22,44),expected)
        for value in (-1,float('nan'),181):
            with self.assertRaises(ValueError):p.make_target_trim(1040,value)
        for value in (.099,600.1,float('inf')):
            with self.assertRaises(ValueError):p.make_board_config(1040,60,value,20,40)

    def test_monitor_real_high12_units(self):
        payload=struct.pack('<4ihHBBHiB',173,200,170,-256,400,3000,0x3b,0xc5,1040,87,6)
        r=p.decode_monitor(payload)
        self.assertEqual((r['user_counts'],r['internal_counts'],r['actual_counts']),(173,200,170))
        self.assertAlmostEqual(r['reciprocal_center_degrees'],87*360/1040)
        self.assertEqual(r['adc_code'],3000)
        from adc_units import sensor_voltage
        self.assertEqual(sensor_voltage(3000),3.75)

    def test_gui_exact_ack_save_and_start_gates(self):
        import tkinter as tk
        gui=load('board_control_gui',ROOT/'tools/adc_gui.pyw')
        gui.messagebox.showerror=lambda *args: self.fail(str(args))
        root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
        profile=ROOT/'tools/tuned_profile.json';original=profile.read_bytes()
        app=gui.PendulumWindow(root)
        app._confirm_log_parameter=lambda *a:None
        app._start_log=lambda:None
        try:
            self.assertTrue(app.profile_loaded)
            app.connected=True
            app.tolerance_text.set('1.5');app.send_position_tolerance()
            trim=app.write_queue.get_nowait();self.assertEqual(trim[2],0x37)
            pending=app.pending_tolerance
            app._position_tolerance_echo(p.make_target_trim(1040,2,True,60)[4:-1]+b'\0')
            self.assertEqual(app.pending_tolerance,pending)
            app._position_tolerance_echo(pending+b'\0');self.assertIsNone(app.pending_tolerance)
            app.small_step_limit_text.set('22');app.large_step_limit_text.set('44')
            app.board_amplitude_text.set('90');app.board_interval_text.set('.5');app.send_board_config()
            f=app.write_queue.get_nowait();pending=app.pending_board
            self.assertEqual(f[2],0x40)
            app._board_echo(pending[:10]+b'\0\0')
            self.assertEqual(app.pending_board,pending) 
            app._board_echo(pending[:10]+b'\0\0'+pending[10:]);self.assertIsNone(app.pending_board)
            app.latest_motor=(time.monotonic(),0,0,0,0,0x3b,0)
            app.wide_arm_firmware=True;app.arm_target_confirmed=True;app.configured_travel=2000;app.arm_target_count=0
            app.arm_step_text.set('179.99');app.shift_arm_target(1)
            f=app.write_queue.get_nowait();self.assertEqual(struct.unpack('<iH',f[4:-1]),(520,22))
            
            app.events.put((app.session,'arm_target',struct.pack('<iB',520,0)));app.poll_events()
            self.assertEqual(app.pending_arm_target,520)
            app.events.put((app.session,'arm_target',struct.pack('<iBH',520,0,22)));app.poll_events()
            self.assertIsNone(app.pending_arm_target)
            app._sync_touch_state(struct.pack('<HBBBHi',1,6,6,1,1578,173))
            self.assertEqual(app.arm_target_count,173)
            app._monitor_echo(time.monotonic(),struct.pack('<4ihHBBHiB',173,190,170,0,50,3000,0x3b,0xc5,1040,173,6))
            self.assertIn('内部目标',app.three_angles_text.get());self.assertIn('58.85',app.three_angles_text.get())
            app.save_profile();saved=json.loads(profile.read_text(encoding='utf-8'))
            self.assertEqual(saved['board_amplitude_deg'],'90');self.assertEqual(saved['board_interval_seconds'],'.5')
            self.assertEqual(saved['target_tolerance_deg'],'1.5');self.assertEqual(saved['large_step_tilt_limit'],'44')
            
            app.remote_run_requested=False;app.auto_start_pending=False;app.latest_motor=(time.monotonic(),0,0,0,0,0x28,0)
            app.start_system(manual=True)
            frames=[]
            while not app.write_queue.empty():frames.append(app.write_queue.get_nowait())
            self.assertIn(0x37,[f[2] for f in frames]);self.assertIn(0x40,[f[2] for f in frames])
            self.assertFalse(any(f[2]==0x12 for f in frames))
            for name in ('pending_config','pending_swing','pending_period','pending_outer_period','pending_wrap','pending_tilt','pending_deadzone'):
                setattr(app,name,None)
            app.poll_events();self.assertTrue(app.auto_start_pending)
            app._position_tolerance_echo(app.pending_tolerance+b'\0')
            board=app.pending_board;app._board_echo(board[:10]+b'\0\0'+board[10:])
            app._response_speed_echo(bytes((0,app.pending_response)))
            app._correction_tuning_echo(b'\0'+app.pending_tuning)
            app.poll_events();self.assertTrue(app.auto_start_pending)
            self.assertFalse(any(f[2]==0x12 for f in list(app.write_queue.queue)))
            app._pwm_limit_echo(b'\0'+app.pending_pwm_limit)
            app.poll_events();self.assertFalse(app.auto_start_pending)
            self.assertTrue(any(f[2]==0x12 for f in list(app.write_queue.queue)))
        finally:
            profile.write_bytes(original);root.destroy()

if __name__=='__main__':unittest.main()
