"""参数档案管理：处理单位换算、默认字段和受限目录内的日志清理。"""
import copy,math,stat
from pathlib import Path

def migrate_profile(data):
    data=copy.deepcopy(data)
    units=data.get('adc_units')
    if units!='ads8688_high12':
        raise ValueError('仅支持本工程 ADS8688 高12位参数档案')
    if float(data.get('period_ms',1))!=1:
        raise ValueError('内环固定 1 ms')
    data.setdefault('pwm_deadzone','0')
    data['swing'].setdefault('stage2_pwm','200');data['swing'].setdefault('stage2_ms','20')
    data.update(adc_units='ads8688_high12',format_version=10,
                swing_mode='two_stage_bottom_pulse',control_mode='position_pd_native',
                period_ms='1.0',outer_period_ms=str(data.get('outer_period_ms',10)))
    return data

def clean_history(project):
    """仅清理本工程日志目录内的采集文件及其同名元数据，跳过符号链接和重解析目录。"""
    project=Path(project).resolve();root=project/'logs'
    def redirected(p):
        s=p.lstat()
        return stat.S_ISLNK(s.st_mode) or bool(getattr(s,'st_file_attributes',0)&stat.FILE_ATTRIBUTE_REPARSE_POINT)
    if not root.exists():return 0,[]
    if redirected(root) or root.resolve().parent!=project:return 0,['日志目录指向工程外，已跳过']
    count=0;errors=[]
    def visit(folder):
        nonlocal count
        for p in folder.iterdir():
            try:
                if not p.exists():continue
                if redirected(p) or not p.resolve().is_relative_to(root.resolve()):continue
                if p.is_dir():visit(p)
                elif p.is_file() and p.suffix.lower()=='.csv':
                    p.chmod(p.stat().st_mode|stat.S_IWRITE)
                    p.unlink();count+=1
                    meta=p.with_suffix('.json')
                    if meta.exists() and not redirected(meta):
                        meta.chmod(meta.stat().st_mode|stat.S_IWRITE);meta.unlink()
            except OSError as exc:errors.append(f'{p.name}: {exc}')
    try:visit(root)
    except OSError as exc:errors.append(str(exc))
    return count,errors
