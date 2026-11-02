"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import unittest,sys,json,tempfile,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
from test_handoff_ui import gui_module
import adc_protocol as p

class ForwardStatusTests(unittest.TestCase):
 def test_disable_is_board_command_and_forward_phases_are_logged(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  profile=ROOT/'tools/tuned_profile.json';before=profile.read_bytes();app=g.PendulumWindow(root)
  try:
   app.connected=True;app.set_reciprocal(False)
   self.assertEqual(app.write_queue.get_nowait(),p.make_reciprocal_command(False));self.assertTrue(app.write_queue.empty())
   frame=p.make_board_config(1040,60,2,28,40,35);body=frame[4:-1]
   with tempfile.TemporaryDirectory(dir=ROOT/'docs') as td:
    self.assertTrue(Path(td).resolve().is_relative_to((ROOT/'docs').resolve()))
    metadata=Path(td)/'run.json';metadata.write_text('{}');app.log_metadata_path=metadata;app.log_started=time.monotonic()
    for flags,label in ((160,'限速减速'),(32,'低速到位确认'),(0,'平时')):
     ack=body[:10]+bytes((0,flags))+body[10:]
     app._board_echo(ack);self.assertIn(label,app.tilt_profile_status.get())
     app._board_echo(ack)
    history=json.loads(metadata.read_text(encoding='utf-8'))['parameter_history']
    self.assertEqual(len(history),3);self.assertEqual([x['value']['forward_ramping'] for x in history],[True,False,False])
    self.assertEqual([x['value']['kind'] for x in history],[1,1,0])
    app.log_metadata_path=None
   self.assertEqual(profile.read_bytes(),before)
  finally:app.log_metadata_path=None;root.destroy()

if __name__=='__main__':unittest.main()
