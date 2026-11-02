"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
from pathlib import Path
import sys,unittest,struct,time
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
from angle_fullscreen import reference_segments
from test_handoff_ui import gui_module

class ReferenceHistoryTests(unittest.TestCase):
 def test_window_hold_gaps_and_no_backfilled_past(self):
  events=[(2,0,'用户目标'),(4,60,'用户目标'),(8,None,'用户目标'),(9,30,'往复中心')]
  self.assertEqual(reference_segments(events,0,3),[(2,3,0,'用户目标')])
  self.assertEqual(reference_segments(events,5,10),[(5,8,60,'用户目标'),(9,10,30,'往复中心')])

 def test_main_and_fullscreen_share_steps_and_reciprocal_center(self):
  import tkinter as tk
  g=gui_module();root=tk.Tk();root.withdraw();root.state=lambda *a:None;root.geometry=lambda *a:None
  profile=ROOT/'tools/tuned_profile.json';before=profile.read_bytes();app=g.PendulumWindow(root)
  try:
   t=time.monotonic();app.connected=True;app.arm_target_confirmed=True;app.arm_target_count=0
   app.record_angle_reference(t)
   app.arm_target_count=173;app.record_angle_reference(t+1)
   app.latest_motor=(t,0,0,0,0,0x3b,0);app.motor_samples.extend([(t,0,0),(t+2,173,0)])
   app.angle_plot.canvas.winfo_width=lambda:1000;app.angle_plot.canvas.winfo_height=lambda:250
   with patch.object(g.time,'monotonic',return_value=t+2):app.redraw_plots();app.open_angle_fullscreen()
   view=app._angle_fullscreen;root.update_idletasks()
   view.plot.canvas.winfo_width=lambda:1400;view.plot.canvas.winfo_height=lambda:650
   with patch.object(g.time,'monotonic',return_value=t+2):app.redraw_plots()
   def horizontal_lines(plot):
    return [plot.canvas.coords(i) for i in plot.canvas.find_withtag('user_target') if plot.canvas.coords(i)[1]==plot.canvas.coords(i)[3]]
   for plot in (app.angle_plot,view.plot):
    lines=horizontal_lines(plot);self.assertEqual(len(lines),2,str(plot.canvas))
    self.assertNotEqual(lines[0][1],lines[1][1]);self.assertAlmostEqual(lines[0][2],lines[1][0])
   
   def monitor(user,center,enabled):
    return struct.pack('<4ihHBBHiB',user,user,0,0,0,1578,0x3b,0,1040,center,6 if enabled else 0)
   app._monitor_echo(t+3,monitor(693,173,True));app.arm_target_count=693;app.record_angle_reference(t+3)
   app._monitor_echo(t+4,monitor(-347,173,True));app.arm_target_count=-347;app.record_angle_reference(t+4)
   self.assertEqual(len(app.angle_reference_history),3)
   self.assertEqual(app.angle_reference_history[-1],(t+3,173*360/1040,'往复中心'))
   with patch.object(g.time,'monotonic',return_value=t+5):app.redraw_plots()
   view.render(t+5)
   label=app.angle_plot.canvas.itemcget(app.angle_plot.canvas.find_withtag('user_target_label')[0],'text')
   self.assertEqual(label,'往复中心 +60°')
   self.assertEqual(len(horizontal_lines(app.angle_plot)),3)
   self.assertEqual(len(horizontal_lines(view.plot)),3)
   
   app._monitor_echo(t+6,monitor(693,0,True));app.record_angle_reference(t+6)
   app._monitor_echo(t+7,monitor(0,0,False));app.arm_target_count=0;app.record_angle_reference(t+7)
   self.assertEqual(app.angle_reference_history[-2][1:],(0,'往复中心'))
   self.assertEqual(app.angle_reference_history[-1][1:],(0,'用户目标'))
   view.close();app.open_angle_fullscreen();self.assertEqual(len(app.angle_reference_history),5)
   app.connected=False;app.record_angle_reference(t+8)
   self.assertIsNone(app.angle_reference_history[-1][1])
   app.clear_samples();self.assertFalse(app.angle_reference_history)
   
   app.connected=True;app._monitor_echo(t+9,monitor(693,173,True));app.clear_samples()
   self.assertEqual(app.record_angle_reference(t+9),(173*360/1040,'往复中心'))
   self.assertTrue(app.write_queue.empty());self.assertEqual(profile.read_bytes(),before)
  finally:
   if app._angle_fullscreen:app._angle_fullscreen.close()
   root.destroy()

if __name__=='__main__':unittest.main()
