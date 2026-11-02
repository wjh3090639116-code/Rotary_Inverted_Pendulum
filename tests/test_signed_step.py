"""离线辅助源码：保留仿真激励、数据处理或界面功能。"""
import sys,struct,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from adc_protocol import make_swing_config,decode_swing_echo,decode_touch_state

class SignedStepTests(unittest.TestCase):
 def test_signed_stage2_wire_format_and_bounds(self):
  base=(550,250,201,1808,1168,32,160,2000)
  for first in (-550,550):
   for second in (-900,-400,0,400,900):
    data=(first,*base[1:],second,300)
    f=make_swing_config(*data)
    self.assertEqual((f[2],f[3]),(0x28,20))
    self.assertEqual(struct.unpack('<h',f[20:22])[0],second)
    self.assertEqual(decode_swing_echo(f[4:-1]),data)
  for v in (-32768,-901,901,65535,True,0.5):
   with self.assertRaises(ValueError):make_swing_config(*base,v,300)
 def test_full_turn_touch_echo_compatible(self):
  for action,target in ((8,-1040),(9,1040)):
   self.assertEqual(decode_touch_state(struct.pack('<HBBBHi',3,action,6,1,1578,target)),(3,action,6,1,1578,target))

if __name__=='__main__':unittest.main()
