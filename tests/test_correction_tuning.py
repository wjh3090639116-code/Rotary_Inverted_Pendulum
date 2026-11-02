"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import unittest,sys,time,json,struct
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
import adc_protocol as p
from test_handoff_ui import gui_module
class TuningTests(unittest.TestCase):
 def test_wire_caps(self):
  for strength in range(3):
   for rate in (.5,1,3,10):
    f=p.make_correction_tuning(strength,rate)
    self.assertEqual(f[2:4],b'\x46\x03');gain,limit=struct.unpack('<BH',f[4:-1])
    self.assertEqual(gain,strength);self.assertLessEqual(limit*18000/(1040*256),rate+1e-8)
    self.assertEqual(p.decode_correction_tuning(b'\0'+f[4:-1]),(0,gain,limit))
  for rate in (0,.49,10.1,float('nan'),True):
   with self.assertRaises(ValueError):p.make_correction_tuning(0,rate)
  for gain in (-1,3,True):
   with self.assertRaises(ValueError):p.make_correction_tuning(gain,3)
 def test_explicit_enable_defaults_ack_save_and_history(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  profile=ROOT/'tools/tuned_profile.json';original=profile.read_bytes()
  fixture=json.loads(original)
  for key in ('compensation_tuning_version','correction_strength','correction_rate_deg_s'):fixture.pop(key,None)
  profile.write_text(json.dumps(fixture,ensure_ascii=False),encoding='utf-8')
  app=g.PendulumWindow(root)
  meta=ROOT/'docs/test_tuning_metadata.json'
  try:
   self.assertFalse(app.target_trim_enabled.get());self.assertEqual(app.correction_strength_text.get(),'柔和')
   self.assertEqual(float(app.correction_rate_text.get()),3)
   app.connected=True;app.log_metadata_path=meta;app.log_started=time.monotonic();meta.write_text('{}',encoding='utf-8')
   app.send_correction_tuning();f=app.write_queue.get_nowait();self.assertEqual(f[2],0x46)
   pending=app.pending_tuning;app._correction_tuning_echo(struct.pack('<BBH',0,1,44));self.assertEqual(app.pending_tuning,pending)
   app._correction_tuning_echo(b'\0'+pending);self.assertIsNone(app.pending_tuning)
   self.assertFalse(app.target_trim_enabled.get())
   history=json.loads(meta.read_text(encoding='utf-8'))['parameter_history'];self.assertEqual(history[-1]['key'],'correction_tuning_confirmed')
   app.correction_rate_text.set('1.0');app.target_trim_enabled.set(True);app.save_profile()
   saved=json.loads(profile.read_text(encoding='utf-8'));self.assertEqual(saved['compensation_tuning_version'],1);self.assertEqual(saved['correction_rate_deg_s'],'1.0')
   app.pending_tuning=b'\0\x2c\0';app.tuning_deadline=time.monotonic()-1;app.auto_start_pending=True;app.poll_events()
   self.assertFalse(app.auto_start_pending);self.assertIn('超时',app.correction_tuning_status.get())
  finally:
   app.log_metadata_path=None;profile.write_bytes(original);root.destroy()
   if meta.exists():meta.unlink()
if __name__=='__main__':unittest.main()
