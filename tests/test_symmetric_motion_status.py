"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import unittest,sys,json,tempfile,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
from test_handoff_ui import gui_module
import adc_protocol as p

class SymmetricStatusTests(unittest.TestCase):
 def test_all_four_actions_report_right_phase_and_tilt_profile(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  before=(ROOT/'tools/tuned_profile.json').read_bytes();app=g.PendulumWindow(root)
  try:
   body=p.make_board_config(1040,60,2,28,40,35)[4:-1]
   with tempfile.TemporaryDirectory(dir=ROOT/'docs') as td:
    self.assertTrue(Path(td).resolve().is_relative_to((ROOT/'docs').resolve()))
    meta=Path(td)/'run.json';meta.write_text('{}');app.log_metadata_path=meta;app.log_started=time.monotonic()
    for flag,name,bound in ((160,'正转一圈',40),(192,'反转一圈',35),(224,'正转180°',28),(225,'反转180°',28),(97,'反转180°',28)):
     ack=body[:10]+bytes((0,flag))+body[10:]
     p.decode_board_echo(ack);app._board_echo(ack)
     self.assertIn(name,app.tilt_profile_status.get());self.assertIn(f'±{bound}',app.tilt_profile_status.get())
     self.assertIn('限速减速' if flag&128 else '低速到位确认',app.tilt_profile_status.get())
    history=json.loads(meta.read_text(encoding='utf-8'))['parameter_history']
    states=[x['value'] for x in history]
    self.assertEqual([s['motion_ramping'] for s in states],[True,True,True,True,False])
    self.assertEqual([s['half_turn_negative'] for s in states],[False,False,False,True,True])
    self.assertEqual([s['forward_ramping'] for s in states],[True,False,True,False,False])
    app.log_metadata_path=None
   self.assertEqual((ROOT/'tools/tuned_profile.json').read_bytes(),before)
  finally:app.log_metadata_path=None;root.destroy()

if __name__=='__main__':unittest.main()
