"""上电配置生成工具：根据保存的参数生成停止状态的初始化数据，不连接硬件。"""
from pathlib import Path
import json,hashlib,re,sys,argparse
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'tools'))
from adc_protocol import make_config,make_swing_config,make_control_period,make_outer_period,make_wrap_config,make_tilt_limit,make_pwm_deadzone,FrameDecoder
ap=argparse.ArgumentParser();ap.add_argument('--check',action='store_true');ap.add_argument('--profile',type=Path,default=root/'tools/tuned_profile.json');args=ap.parse_args()
raw=args.profile.read_bytes();d=json.loads(raw);b=d['balance'];s=d['swing']
frames=[make_config(*[int(b[k]) for k in ['零点 ADC','角度方向','编码器方向','捕获窗口','跌倒门限','行程门限']],*[float(b[k]) for k in ['Kp','Ki','Kd','转臂 Kp','转臂 Kd']]),make_swing_config(*[int(s[k]) for k in ['pump_pwm','pulse_ms','bottom_code','target_code','upper_code','bottom_window','rearm_distance','travel','stage2_pwm','stage2_ms']]),make_control_period(float(d['period_ms'])),make_outer_period(float(d['outer_period_ms'])),make_wrap_config(int(d['wrap_low']),int(d['wrap_high'])),make_tilt_limit(int(d['tilt_limit'])),make_pwm_deadzone(int(d['pwm_deadzone']))]
data=b''.join(frames);assert len(data)<=128 and all(k!=0x12 for k,v in FrameDecoder().feed(data))
hex_text=''.join(f'{v:02x}\n' for v in data+bytes(128-len(data)))
module=root/'src/tft_profile_boot.v';top=root/'src/ads8688_uart_top.v'
module_text=re.sub(r'localparam integer BYTE_COUNT=\d+;',f'localparam integer BYTE_COUNT={len(data)};',module.read_text(encoding='utf-8'))
cpr=int(d['counts_per_rev']);assert 6<=cpr<=1000000
old_top=top.read_text(encoding='utf-8');top_text=re.sub(r'ENABLE_SCREEN=1, COUNTS_PER_REV=\d+',f'ENABLE_SCREEN=1, COUNTS_PER_REV={cpr}',old_top)
if args.check:
 assert (root/'src/tft_profile.hex').read_text()==hex_text,'ROM differs from saved profile'
 assert module.read_text(encoding='utf-8')==module_text,'ROM length mismatch'
 assert old_top==top_text,'screen counts-per-revolution differs from saved profile'
else:
 (root/'src/tft_profile.hex').write_text(hex_text,encoding='ascii')
 module.write_text(module_text,encoding='utf-8');top.write_text(top_text,encoding='utf-8')
print(json.dumps({'saved_profile_sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(data),'auto_RUN':False,'counts_per_rev':cpr,'check_only':args.check}))
