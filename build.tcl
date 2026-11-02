# 工程构建入口：保持器件、顶层及串行引脚复用设置。
open_project Rotary_Inverted_Pendulum.gprj
set_option -top_module ads8688_uart_top
set_option -output_base_name Rotary_Inverted_Pendulum
set_option -timing_driven 0
set_option -opt_goal area
set_option -use_sspi_as_gpio 1
run all
run close
source restore_ide_sspi.tcl


