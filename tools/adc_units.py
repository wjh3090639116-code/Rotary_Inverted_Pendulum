"""采集数据量纲转换：将高１２位角度码换算为界面电压显示值。"""
ADC_MODEL='ADS8688'
ADC_BITS=16
CONTROL_STORAGE_BITS=12
CONTROL_CODE_MAX=4095
SENSOR_FULL_SCALE_V=5.12
PROFILE_ADC_UNITS='ads8688_high12'
def sensor_voltage(code,full_scale=SENSOR_FULL_SCALE_V):
    return code/4096*full_scale
