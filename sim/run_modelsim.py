"""离线仿真调度脚本：整理仿真器输入文件并按指定列表运行仿真。"""
from pathlib import Path
import subprocess,sys,os
P=Path(__file__).resolve().parents[1]
M=Path(os.environ.get('MODELSIM_BIN','D:/Modelsim.10.4/win64'))
normal=P/'sim/modelsim_sources';normal.mkdir(exist_ok=True)
def lift(s,items,anchor):
    declarations=[]
    for old,decl in items:
        assert old in s,old
        declarations.append(decl)
        s=s.replace(old,('assign '+old.split('wire ',1)[1].split('=',1)[0].split()[-1]+'='+old.split('=',1)[1]) if '=' in old and old.startswith('wire ') else '',1)
    return s.replace(anchor,'\n'.join(declarations)+'\n '+anchor,1)
for p in (P/'src').glob('*.v'):
    s=p.read_text(encoding='utf-8')
    if p.name=='ads8688_uart_top.v':
        items=[
          'wire arm_target_rx_valid;',
          'wire config_rx_valid, swing_rx_valid, period_rx_valid, wrap_rx_valid, tilt_rx_valid;',
          'wire run_rx_valid, run_rx_command, run_rx_manual, query_rx_valid;',
          'reg config_checked_valid,config_checked_ok;', 'reg run_request;',
          'reg config_loaded, angle_reverse, encoder_reverse;', 'reg [239:0] config_echo;',
          'reg [11:0] zero_code, capture_window, fall_limit;', 'reg free_pending;', 'reg arm_target_pending,target_in_travel;',
          'localparam [2:0] OFF=0,WAIT_UPRIGHT=1,ACTIVE=2,FAULT=3,SWING=4,COMPLETE=5;',
          'reg [2:0] state;', 'reg signed [31:0] encoder_center,previous_position,arm_position;',
          'reg remote_run;', 'reg arm_target_with_tilt,target_tilt_valid;reg [15:0] arm_target_tilt;', 'reg signed [31:0] arm_target_command;', 'wire capture_event;',
        ]
        pairs=[(d,d) for d in items]+[
          ('wire adc_bad=sensor_fault_reason!=0;','wire adc_bad;'),
          ('wire arm_target_accept=arm_target_pending && target_operable && target_in_travel && target_tilt_valid &&'+chr(10)+'        !(target_from_reciprocal && (manual_target_valid || reciprocal_config_load || (!free_mode && !target_from_center_return)));','wire arm_target_accept;'),
          ('wire signed [31:0] arm_target_full={{15{arm_target_position[16]}},arm_target_position};','wire signed [31:0] arm_target_full;')]
        import re
        pairs.append(('wire pid_enable;', 'wire pid_enable;'))
        for pattern,decl in [(r'wire target_operable=.*?;', 'wire target_operable;'),(r'wire \[7:0\] flags=.*?;', 'wire [7:0] flags;')]:
            pairs.append((re.search(pattern,s,re.S).group(0),decl))
        s=lift(s,pairs,'localparam integer LOCAL_UI=ENABLE_SCREEN;')
    elif p.name=='tft_pixel.v' and 'dx_hold' in s:
        s=lift(s,[(x,x) for x in ['reg [8:0] dx_hold,x_hold;','reg [8:0] dy_hold,y_hold;','reg inside_hold;','reg  inside_draw_next;']], '`include "src/tft_layout.vh"')
    elif p.name=='tft_profile_boot.v':
        s=lift(s,[('localparam integer BYTE_COUNT=129;','localparam integer BYTE_COUNT=129;')],'reg [7:0] rom[0:191];')
    elif p.name=='tft_touch.v':
        s=lift(s,[('reg [2:0] state;','reg [2:0] state;')],'localparam integer HALF=')
    (normal/p.name).write_text(s,encoding='utf-8')
tests=sys.argv[1:] or ['tb_pendulum_recovery', 'tb_symmetric_motion_uart', 'tb_symmetric_reference', 'tb_center_return_uart', 'tb_forward_reference', 'tb_turn_tilt_uart', 'tb_pwm_limit_uart', 'tb_packet_equivalence', 'tb_gentle_models', 'tb_tuning_uart', 'tb_response_uart', 'tb_continuous_models', 'tb_response_modes', 'tb_response_model', 'tb_response_inertia', 'tb_target_trim', 'tb_board_targets', 'tb_trim_adaptive_speed', 'tb_trim_oscillating_bias', 'tb_trim_recorded_trace', 'tb_trim_target_handoff', 'tb_trim_target_inertia', 'tb_board_control', 'tb_handoff_uart', 'tb_settings_adapter', 'tb_ads8688_control', 'tb_live_outer_pd', 'tb_live_pid', 'tb_manual_start', 'tb_screen_control', 'tb_signed_stage2', 'tb_step_controls', 'tb_two_stage_uart_top', 'tb_v4_pixels', 'tb_v4_scheduler']
def execute(args,log):
    r=subprocess.run([str(M/args[0]),*args[1:]],cwd=P,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=240)
    (P/'docs'/log).write_text(r.stdout+r.stderr,encoding='utf-8')
    if r.returncode:raise RuntimeError((log,(r.stdout+r.stderr)[-6000:]))
    return r.stdout+r.stderr
if not (P/'sim_work').exists():execute(['vlib.exe','sim_work'],'step_vlib.log')
sources=[str(f.relative_to(P)) for f in normal.glob('*.v')]+['sim/ads8688_input_model.sv','sim/adc_filter_reference.sv','sim/uart_packet_tx_reference.sv','sim/forward_reference_baseline.sv','sim/tft_pixel_reference.sv']
for t in tests:sources.append(str(next(f for f in (P/'sim').glob(t+'.*') if f.suffix in ('.v','.sv')).relative_to(P)))
execute(['vlog.exe','-sv','-work','sim_work',*sources],'step_modelsim_compile.log')
script='onbreak {quit -code 1}; onerror {quit -code 1}; run -all; quit -code 0'
for t in tests:
    out=execute(['vsim.exe','-c','-l',str(P/'docs'/f'{t}.log'),'sim_work.'+t,'+TRIM_TRACE=sim/data/biased_return_trace.hex','-do',script],t+'.console.log')
    assert 'PASS' in out,(t,out[-2000:])
    print(t,'PASS',flush=True)


