// 定义 50 MHz 系统时钟及异步输入的时序约束。
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
set_false_path -from [get_ports {adc_sdo}]
set_false_path -from [get_ports {rst_n}]
