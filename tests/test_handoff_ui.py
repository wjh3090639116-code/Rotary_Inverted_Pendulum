"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import sys,unittest,queue,threading,time,struct,importlib.util,importlib.machinery
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
import adc_protocol as p
def gui_module():
 l=importlib.machinery.SourceFileLoader('handoff_gui',str(ROOT/'tools/adc_gui.pyw'))
 m=importlib.util.module_from_spec(importlib.util.spec_from_loader(l.name,l));l.exec_module(m);return m
class HandoffTests(unittest.TestCase):
 def test_worker_honors_stop_and_only_releases_after_ack(self):
  g=gui_module();writes=[];closed=[]
  class Port:
   in_waiting=6
   def __init__(self,*a,**kw):pass
   def __enter__(self):return self
   def __exit__(self,*a):closed.append(True)
   def write(self,f):writes.append(f)
   def flush(self):pass
   def read(self,n):return p.make_frame(0x3c,b'\x01')
  g.serial=type('SerialModule',(),{'Serial':Port})
  app=object.__new__(g.PendulumWindow);app.events=queue.Queue()
  stop=threading.Event();stop.set();out=queue.Queue()
  for f in (p.make_run_command(True),p.make_arm_target(173,20),p.make_run_command(False)):out.put(f)
  app.serial_worker(7,'MOCK',115200,stop,out)
  self.assertEqual(writes,[p.make_run_command(False),p.make_board_handoff(),p.make_monitor_subscription(False)])
  self.assertTrue(closed)
  self.assertIn((7,'handoff',1),list(app.events.queue))
  self.assertEqual(list(app.events.queue)[-1],(7,'disconnected',None))
 def test_fresh_real_feedback_and_stale_internal_state(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.geometry=lambda *a:None;root.state=lambda *a:None
  app=g.PendulumWindow(root)
  try:
   app.connected=True;app.arm_target_confirmed=True;app.arm_target_count=693
   app.latest_motor=(time.monotonic(),780,0,0,0,59,0)
   app._monitor_echo(time.monotonic(),struct.pack('<4ihHBBHiB',693,780,780,0,0,1578,59,0xd1,1040,0,0))
   app._refresh_target_readout()
   self.assertEqual(app.actual_angle_value.get(),'+270.00°')
   self.assertIn('-30.00',app.position_error_value.get())
   self.assertEqual(app.internal_angle_value.get(),'+270.00°')
   app.latest_monitor_at-=2;app._refresh_target_readout()
   self.assertEqual(app.internal_angle_value.get(),'—')
   self.assertIn('过期',app.monitor_age_text.get());self.assertIn('未知',app.trim_runtime_text.get())
   self.assertEqual(app.write_queue.get_nowait(),p.make_monitor_subscription(True))
   app._refresh_target_readout();self.assertTrue(app.write_queue.empty())
   app.stop_event=threading.Event();app.remote_run_requested=True
   app._begin_disconnect();self.assertFalse(app.remote_run_requested);self.assertTrue(app.disconnecting)
   self.assertEqual(str(app.connect_button['state']),'disabled')
   app.events.put((app.session,'handoff',0));app.events.put((app.session,'disconnected',None));app.poll_events()
   self.assertIn('板端继续运行',app.status_text.get());self.assertIsNone(app.stop_event)
  finally:root.destroy()
if __name__=='__main__':unittest.main()
