"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
import importlib.machinery,importlib.util,sys,json,struct,unittest,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import native_profile
native_profile.clean_history=lambda *args:(0,[])
from adc_protocol import decode_touch_state,make_config,make_swing_config

class ScreenHostTests(unittest.TestCase):
 def test_screen_protocol(self):
  self.assertEqual(decode_touch_state(struct.pack('<HBBBHi',3,6,6,1,3000,1040)),(3,6,6,1,3000,1040))
  with self.assertRaises(ValueError):decode_touch_state(b'\0'*3)
 def test_gui_keeps_new_tuning_and_syncs(self):
  import tkinter as tk
  loader=importlib.machinery.SourceFileLoader('screen_gui',str(ROOT/'tools/adc_gui.pyw'))
  spec=importlib.util.spec_from_loader(loader.name,loader);gui=importlib.util.module_from_spec(spec);loader.exec_module(gui)
  gui.clean_history=lambda *args:(0,[])
  root=tk.Tk();root.withdraw();root.state=lambda *args:None
  root.geometry=lambda *args:None
  app=gui.PendulumWindow(root)
  profile=(ROOT/'tools/tuned_profile.json').read_bytes();data=json.loads(profile)
  try:
   self.assertTrue(app.profile_loaded)
   for k,v in data['balance'].items():self.assertEqual(app.parameters[k].get(),str(v))
   self.assertEqual(app._config_frame()[2],0x32)
   self.assertEqual(app._swing_frame()[2],0x28)
   app.auto_start_pending=True;app.manual_start=True
   app._sync_touch_state(struct.pack('<HBBBHi',0,0,0,0,1578,0))
   self.assertTrue(app.auto_start_pending);self.assertTrue(app.manual_start)
   app._sync_touch_state(struct.pack('<HBBBHi',1,1,5,1,1578,0))
   self.assertFalse(app.auto_start_pending);self.assertFalse(app.remote_run_requested);self.assertFalse(app.manual_start)
   self.assertTrue(app.touch_local_run)
   app._sync_touch_state(struct.pack('<HBBBHi',2,6,6,1,1578,1040))
   self.assertEqual(app.arm_target_count,1040)
   app.events.put((app.session,'touch_state',struct.pack('<HBBBHi',3,2,1,0,1578,0)))
   app.poll_events();self.assertFalse(app.remote_run_requested);self.assertEqual(app.arm_target_count,0)
   app._sync_touch_state(struct.pack('<HBBBHi',4,4,3,0,3000,0))
   self.assertEqual(app.parameters['零点 ADC'].get(),'3000')
   self.assertEqual(app.configured_zero,3000)
   before_ki=app.parameters["Ki"].get()
   app._sync_touch_state(struct.pack('<HBBBHi',5,0,6,9,3000,87))
   self.assertEqual(app.parameters["Ki"].get(),before_ki)
   self.assertIn('往复运动已开启',app.status_text.get())
   self.assertEqual(app.arm_target_count,87)
   app._sync_touch_state(struct.pack('<HBBBHi',6,0,6,1,3000,0))
   self.assertEqual(app.parameters["Ki"].get(),before_ki)
   self.assertIn('往复运动已关闭',app.status_text.get())
   self.assertEqual((ROOT/'tools/tuned_profile.json').read_bytes(),profile)
  finally:root.destroy()

if __name__=='__main__':unittest.main()
