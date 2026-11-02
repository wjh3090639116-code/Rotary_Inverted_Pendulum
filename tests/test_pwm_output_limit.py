"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import unittest,sys,time,json,struct,tempfile
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
import adc_protocol as p
from test_handoff_ui import gui_module

class PWMOutputLimitTests(unittest.TestCase):
 def test_protocol_range_resolution_and_echo(self):
  for percent in (0,.1,12.5,50,90):
   f=p.make_pwm_output_limit(percent)
   self.assertEqual(f[2:4],b'\x48\x02')
   self.assertEqual(struct.unpack('<H',f[4:-1])[0],round(percent*10))
   self.assertEqual(p.decode_pwm_output_limit(b'\0'+f[4:-1]),(0,round(percent*10)))
  for value in (-.1,90.1,.01,float('nan'),float('inf'),True,'50'):
   with self.assertRaises(ValueError):p.make_pwm_output_limit(value)
  for payload in (b'',b'\0\0',struct.pack('<BH',2,500),struct.pack('<BH',0,901)):
   with self.assertRaises(ValueError):p.decode_pwm_output_limit(payload)

 def test_live_ack_profile_history_and_failure_do_not_start(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  profile=ROOT/'tools/tuned_profile.json';original=profile.read_bytes();old=json.loads(original)
  app=g.PendulumWindow(root)
  try:
   self.assertEqual(float(app.pwm_limit_text.get()),float(old.get('pwm_output_limit_percent',90)))
   app.connected=True;app.pwm_limit_text.set('50')
   with tempfile.TemporaryDirectory(dir=ROOT/'docs') as td:
    self.assertTrue(Path(td).resolve().is_relative_to((ROOT/'docs').resolve()))
    meta=Path(td)/'metadata.json';meta.write_text('{}');app.log_metadata_path=meta;app.log_started=time.monotonic()
    app.send_pwm_limit();frame=app.write_queue.get_nowait();self.assertEqual(frame,p.make_pwm_output_limit(50))
    self.assertFalse(app.remote_run_requested)
    app._pwm_limit_echo(struct.pack('<BH',0,450));self.assertEqual(app.pending_pwm_limit,frame[4:-1])
    app._pwm_limit_echo(struct.pack('<BH',0,500));self.assertIsNone(app.pending_pwm_limit)
    self.assertIn('±50%',app.pwm_limit_status.get())
    history=json.loads(meta.read_text(encoding='utf-8'))['parameter_history']
    self.assertEqual(history[-1]['key'],'pwm_output_limit_confirmed_per_mille');self.assertEqual(history[-1]['value'],500)
    app.log_metadata_path=None
   app.save_profile();saved=json.loads(profile.read_text(encoding='utf-8'))
   self.assertEqual(saved['pwm_output_limit_percent'],'50')
   self.assertEqual(saved['balance'],old['balance']);self.assertEqual(saved['correction_strength'],old['correction_strength'])
   app.pwm_limit_text.set('90');app._load_profile();self.assertEqual(app.pwm_limit_text.get(),'50')
   app.pending_pwm_limit=b'\xf4\1';app.pwm_limit_deadline=time.monotonic()-1;app.auto_start_pending=True
   app.poll_events();self.assertFalse(app.auto_start_pending);self.assertIn('超时',app.pwm_limit_status.get())
   app.auto_start_pending=True;app.pending_pwm_limit=b'\xf4\1'
   app._pwm_limit_echo(struct.pack('<BH',1,900));self.assertFalse(app.auto_start_pending);self.assertIn('拒绝',app.pwm_limit_status.get())
  finally:
   app.log_metadata_path=None;profile.write_bytes(original);root.destroy()

if __name__=='__main__':unittest.main()
