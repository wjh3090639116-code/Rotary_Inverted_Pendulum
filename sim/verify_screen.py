"""显示资源离线分析脚本：读取像素数据并生成便于观察的显示结果。"""
from pathlib import Path
import subprocess,json,time,sys,shutil,os
p=Path(__file__).resolve().parents[1]
iver=Path(os.environ.get('IVERILOG_BIN', '.'))
if os.environ.get('IVERILOG_BIN'):iver=Path(os.environ['IVERILOG_BIN'])
elif shutil.which('iverilog'):iver=Path(shutil.which('iverilog')).parent
else:raise RuntimeError('请设置 IVERILOG_BIN 或将仿真工具加入 PATH。')
tests=['tb_ads8688_high12','tb_ads8688_control','tb_filter_equivalence','tb_native_config_outer','tb_manual_start','tb_live_pid','tb_live_outer_pd','tb_pwm_deadzone','tb_deadzone_handoff','tb_two_stage_uart_top','tb_two_stage_pulse','tb_stage1_uart_top','tb_pulse_alternation','tb_pulse_ranges','tb_swing_adc_positions','tb_swing_adc_uart','tb_tilt_range','tb_screen_control','tb_reciprocator','tb_reciprocating','tb_recip_pixels','tb_tft_ui','tb_tft_touch','tb_tft_button_anim','tb_tft_regions','tb_tft_region_scheduler','tb_landscape_adc']
if len(sys.argv)>1:tests=sys.argv[1:]
sources=[str(v) for v in (p/'src').glob('*.v')]+[str(p/'sim/ads8688_input_model.sv'),str(p/'sim/adc_filter_reference.sv'),str(p/'sim/tft_pixel_reference.sv')]
out=[]
for t in tests:
 f=next(v for v in (p/'sim').glob(t+'.*') if v.suffix in ('.v','.sv'))
 image=p/'docs'/f'{t}.vvp'
 r=subprocess.run([str(iver/'iverilog.exe'),'-g2012','-s',t,'-o',str(image),*sources,str(f)],cwd=p,capture_output=True,text=True,encoding="utf-8",errors="replace")
 if r.returncode: raise RuntimeError(t+' compile: '+r.stderr)
 r=subprocess.run([str(iver/'vvp.exe'),str(image)],cwd=p,capture_output=True,text=True,encoding="utf-8",errors="replace",timeout=180)
 (p/'docs'/f'{t}.log').write_text(r.stdout+r.stderr,encoding='utf-8')
 assert r.returncode==0 and 'PASS' in r.stdout,(t,r.stdout,r.stderr)
 print(t,'PASS',flush=True);out.append(t)
result=p/'docs/screen_simulation_results.json'
previous=json.loads(result.read_text()) if result.exists() else []
result.write_text(json.dumps(sorted(set(previous+out)),indent=2))
