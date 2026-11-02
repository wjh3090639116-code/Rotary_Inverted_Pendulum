"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import sys,unittest,struct,time,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
import adc_protocol as p
from test_handoff_ui import gui_module
class ResponseTests(unittest.TestCase):
 def test_wire_and_validation(self):
  for mode in (False,True):
   f=p.make_response_speed(mode);self.assertEqual(f[2:4],b'\x44\x01');self.assertEqual(f[4],int(mode))
   self.assertEqual(p.decode_response_speed(bytes((0,int(mode)))),(0,mode))
  for value in (0,1,'快速',None):
   with self.assertRaises(ValueError):p.make_response_speed(value)
  for payload in (b'',b'\0',b'\3\1',b'\0\2'):
   with self.assertRaises(ValueError):p.decode_response_speed(payload)
 def test_live_send_matching_ack_persistence_and_timeout(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  profile=ROOT/'tools/tuned_profile.json';saved=profile.read_bytes();app=g.PendulumWindow(root)
  try:
   self.assertEqual(app.response_mode_text.get(),'连续');app.connected=True
   app.send_response_speed();self.assertEqual(app.write_queue.get_nowait(),p.make_response_speed(True))
   app._response_speed_echo(b'\0\0');self.assertEqual(app.pending_response,1)
   app._response_speed_echo(b'\0\1');self.assertIsNone(app.pending_response)
   app.response_mode_text.set('原速');app.send_response_speed();app.write_queue.get_nowait();app._response_speed_echo(b'\0\0')
   self.assertIsNone(app.pending_response);self.assertIn('原速',app.response_status.get())
   app.save_profile();self.assertFalse(json.loads(profile.read_text(encoding='utf-8'))['target_response_fast'])
   app.auto_start_pending=True;app.pending_response=1;app.response_deadline=time.monotonic()-1
   app.poll_events();self.assertFalse(app.auto_start_pending);self.assertIn('超时',app.response_status.get())
  finally:profile.write_bytes(saved);root.destroy()
if __name__=='__main__':unittest.main()
