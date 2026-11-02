# 工程辅助脚本：维持实现流程与引脚选项的一致性。
open_project Rotary_Inverted_Pendulum.gprj
set_option -top_module ads8688_uart_top
set_option -output_base_name Rotary_Inverted_Pendulum
set_option -timing_driven 0
set_option -opt_goal area
set_option -use_sspi_as_gpio 1
run pnr
run close
source restore_ide_sspi.tcl
