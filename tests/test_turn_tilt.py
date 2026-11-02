"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import unittest,sys,json,struct,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
import adc_protocol as p
from test_handoff_ui import gui_module

class TurnTiltTests(unittest.TestCase):
 def test_three_values_wire_and_exception_selection(self):
  for degrees,expected in ((0,22),(60,22),(-60,22),(180,22),(240,22),(359.99,22),(360,44),(-360,55)):
   self.assertEqual(p.turn_tilt_limit(degrees,22,44,55),expected)
  for reverse in (0,65,True):
   with self.assertRaises(ValueError):p.make_board_config(1040,360,2,22,44,reverse)
  f=p.make_board_config(1040,360,2,22,44,55)
  self.assertEqual(f[2:4],b'\x40\x10')
  self.assertEqual(struct.unpack('<HHI4H',f[4:-1]),(1040,1040,2000,22,22,44,55))
  for mode in range(3):
   echo=f[4:14]+bytes((0,mode<<5))+f[14:-1]
   self.assertEqual(len(echo),18);self.assertEqual(p.decode_board_echo(echo)[-1],mode<<5)
  self.assertEqual(p.decode_board_echo(f[4:14]+b'\0\x60'+f[14:-1])[-1],96)

 def test_host_exact_new_ack_mode_persistence_and_late_target_retry(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  profile=ROOT/'tools/tuned_profile.json';original=profile.read_bytes();old=json.loads(original)
  app=g.PendulumWindow(root)
  try:
   self.assertEqual(app.reverse_turn_limit_text.get(),str(old.get('reverse_turn_tilt_limit',old.get('large_step_tilt_limit',40))))
   app.connected=True;app.small_step_limit_text.set('22');app.large_step_limit_text.set('44');app.reverse_turn_limit_text.set('55')
   app.send_board_config();f=app.write_queue.get_nowait();pending=app.pending_board
   self.assertEqual(len(pending),16)
   app._board_echo(pending[:10]+b'\0\0'+pending[10:14]);self.assertEqual(app.pending_board,pending)
   for mode,label in ((1,'正转一圈'),(2,'反转一圈'),(0,'平时')):
    app._board_echo(pending[:10]+bytes((0,mode<<5))+pending[10:])
    self.assertTrue(app.triplet_settings_confirmed);self.assertIn(label,app.tilt_profile_status.get())
   self.assertIsNone(app.pending_board);self.assertEqual(app.confirmed_tilt_profiles,(22,44,55))
   self.assertEqual(app._step_limit(-360),55);self.assertEqual(app._step_limit(180),22)
   app.latest_motor=(time.monotonic(),0,0,0,0,0x3b,0)
   app.wide_arm_firmware=True;app.arm_target_confirmed=True;app.arm_target_count=0;app.configured_travel=2000
   app.arm_step_text.set('360');app.shift_arm_target(1);target_frame=app.write_queue.get_nowait()
   self.assertEqual(struct.unpack('<iH',target_frame[4:-1]),(1040,44))

   app.events.put((app.session,'arm_target',struct.pack('<iBH',1040,0,22)));app.poll_events()
   self.assertIsNone(app.pending_arm_target)
   app.save_profile();saved=json.loads(profile.read_text(encoding='utf-8'))
   for k,v in (('normal_tilt_limit','22'),('forward_turn_tilt_limit','44'),('reverse_turn_tilt_limit','55')):self.assertEqual(saved[k],v)
   self.assertEqual(saved['balance'],old['balance']);self.assertEqual(saved['correction_strength'],old['correction_strength'])
   app.reverse_turn_limit_text.set('1');app._load_profile();self.assertEqual(app.reverse_turn_limit_text.get(),'55')
  finally:profile.write_bytes(original);root.destroy()

if __name__=='__main__':unittest.main()
