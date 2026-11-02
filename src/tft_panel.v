// 触屏子系统入口：组合触摸采样、界面状态、区域调度和像素发送。
// 界面只提交操作请求和显示控制状态，不直接改变电机驱动时序。
`default_nettype none
module tft_panel #(parameter integer CLK_HZ=50000000,COUNTS_PER_REV=1040,LCD_SPI_DIV=3)(
 input wire clk,rst_n,input wire signed [31:0] position,target,
 input wire [3:0] notice,input wire notice_valid,
 input wire [8:0] step_degrees,amplitude_degrees,input wire [7:0] status,input wire [11:0] adc,zero,input wire boot_busy,
 output wire action_valid,output wire [3:0] action,
 output wire tft_sck,tft_mosi,tft_dc,tft_rst_n,tft_cs_n,
 input wire touch_irq_n,touch_miso,output wire touch_sck,touch_mosi,touch_cs_n);
 wire [11:0] raw_x,raw_y;wire raw_valid,down;
 wire [1:0] cal_step;wire calibrated;wire [3:0] pressed;wire [3:0] message;
 wire [35:0] animation_levels;wire draw_start,data_start;
 wire goal_highlight;
 wire window_valid,window_ready,window_started,window_done;
 wire [8:0] window_x0,window_x1;wire [7:0] window_y0,window_y1;
 tft_button_anim #(.CLK_HZ(CLK_HZ)) u_animation(.clk(clk),.rst_n(rst_n),.contact_button(pressed),.levels(animation_levels));
 tft_region_scheduler #(.CLK_HZ(CLK_HZ)) u_regions(.clk(clk),.rst_n(rst_n),.levels(animation_levels),.contact_button(pressed),
 .goal_highlight(goal_highlight),.position(position),.target(target),.adc(adc),.zero(zero),.step_degrees(step_degrees),.amplitude_degrees(amplitude_degrees),.status(status),.message(message),.touch_down(down),
 .window_ready(window_ready),.window_started(window_started),.window_done(window_done),
 .window_valid(window_valid),.x0(window_x0),.x1(window_x1),.y0(window_y0),.y1(window_y1),.draw_start(draw_start),.data_start(data_start));
 wire [8:0] x;wire [7:0] y;wire frame_start;wire [15:0] pixel;
 tft_touch #(.CLK_HZ(CLK_HZ)) u_touch(.clk(clk),.rst_n(rst_n),.irq_n(touch_irq_n),.miso(touch_miso),
 .sck(touch_sck),.mosi(touch_mosi),.cs_n(touch_cs_n),.raw_x(raw_x),.raw_y(raw_y),.sample_valid(raw_valid),.down(down));
 tft_ui #(.CLK_HZ(CLK_HZ)) u_ui(.clk(clk),.rst_n(rst_n),.raw_valid(raw_valid),.down(down),.raw_x(raw_x),.raw_y(raw_y),
 .status(status),.notice(notice),.notice_valid(notice_valid),.boot_busy(boot_busy),.action_valid(action_valid),.action(action),.cal_step(cal_step),.calibrated(calibrated),.pressed(pressed),.message(message));
 tft_pixel #(.CLK_HZ(CLK_HZ),.COUNTS_PER_REV(COUNTS_PER_REV)) u_pixel(.clk(clk),.rst_n(rst_n),.frame_start(data_start),.draw_start(draw_start),.animation_levels(animation_levels),.x(x),.y(y),.goal_highlight(goal_highlight),.position(position),.target(target),.adc(adc),.zero(zero),
 .touch_down(down),.step_degrees(step_degrees),.amplitude_degrees(amplitude_degrees),.status(status),.cal_step(cal_step),.pressed(pressed),.message(message),.pixel(pixel));
 tft_spi_display #(.CLK_HZ(CLK_HZ),.SPI_DIV(LCD_SPI_DIV),.REGION_MODE(1)) u_display(.clk(clk),.rst_n(rst_n),.pixel(pixel),.x(x),.y(y),.frame_start(frame_start),
 .window_valid(window_valid),.window_x0(window_x0),.window_x1(window_x1),.window_y0(window_y0),.window_y1(window_y1),
 .window_ready(window_ready),.window_started(window_started),.window_done(window_done),
 .sck(tft_sck),.mosi(tft_mosi),.dc(tft_dc),.reset_n(tft_rst_n),.cs_n(tft_cs_n));
endmodule
`default_nettype wire
