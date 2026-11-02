"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import unittest,sys,json,tempfile,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
from test_handoff_ui import gui_module
import adc_protocol as p

class PendulumRecoveryTests(unittest.TestCase):
 def test_recovery_echo_preserves_normal_field_and_logs_actual_bound(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  before=(ROOT/'tools/tuned_profile.json').read_bytes();app=g.PendulumWindow(root)
  try:
   app.small_step_limit_text.set('28')
   with tempfile.TemporaryDirectory(dir=ROOT/'docs') as td:
    meta=Path(td)/'run.json';meta.write_text('{}');app.log_metadata_path=meta;app.log_started=time.monotonic()
    for normal,flags,label,bound in ((28,128,'摆杆恢复档',40),(28,0,'平时',28),(50,128,'摆杆恢复档',50)):
     body=p.make_board_config(1040,60,2,normal,43,47)[4:-1]
     app._board_echo(body[:10]+bytes((0,flags))+body[10:])
     self.assertIn(label,app.tilt_profile_status.get());self.assertIn(f'±{bound}',app.tilt_profile_status.get())
     self.assertEqual(app.small_step_limit_text.get(),'28')
    states=[x['value'] for x in json.loads(meta.read_text(encoding='utf-8'))['parameter_history']]
    self.assertEqual([x['normal_recovery_active'] for x in states],[True,False,True])
    self.assertTrue(all(not x['motion_ramping'] and not x['forward_ramping'] for x in states))
    app.log_metadata_path=None
   self.assertEqual((ROOT/'tools/tuned_profile.json').read_bytes(),before)
  finally:app.log_metadata_path=None;root.destroy()

if __name__=='__main__':unittest.main()
