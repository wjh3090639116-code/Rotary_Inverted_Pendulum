// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_v4_scheduler;
reg clk=0,rst_n=0;always #5 clk=~clk;
reg [35:0] levels=0;reg [3:0] contact=0;
reg [31:0] pos=0,target=0;reg [11:0] adc=1578;reg [8:0] step=60,amp=30;
reg [7:0] flags=0;reg [3:0] msg=0;reg hi=0,ready=0,started=0,done=0;
wire valid,draw,data;wire [8:0] x0,x1;wire [7:0] y0,y1;
tft_region_scheduler #(.CLK_HZ(1000)) dut(.clk(clk),.rst_n(rst_n),.levels(levels),.contact_button(contact),
 .position(pos),.target(target),.adc(adc),.zero(12'd1578),.step_degrees(step),.amplitude_degrees(amp),
 .status(flags),.message(msg),.touch_down(1'b0),.goal_highlight(hi),.window_ready(ready),.window_started(started),.window_done(done),
 .window_valid(valid),.x0(x0),.x1(x1),.y0(y0),.y1(y1),.draw_start(draw),.data_start(data));
task transfer;begin
 ready=1;@(negedge clk);ready=0;started=1;@(negedge clk);started=0;done=1;@(negedge clk);done=0;
end endtask
task drain;integer n;begin n=0;repeat(3)@(negedge clk);while(valid && n<500)begin transfer;n=n+1;end if(n==500)$fatal(1,"scheduler busy forever");end endtask
initial begin
 repeat(5)@(negedge clk);rst_n=1;drain;
 if(dut.initial_active)$fatal(1,"initial refresh unfinished");
 pos=83;repeat(4)@(negedge clk);
 if(!valid || y0!==24 || y1!==27)$fatal(1,"actual encoder must refresh data card");drain;
 step=70;repeat(4)@(negedge clk);if(!valid || y0!==96)$fatal(1,"step must refresh settings");drain;
 amp=40;repeat(4)@(negedge clk);if(!valid || y0!==96)$fatal(1,"amplitude settings");drain;
 flags=64;drain;
 target=87;hi=1;repeat(55)@(negedge clk);if(!valid || y0!==24)$fatal(1,"target highlight refresh");drain;
 hi=0;repeat(55)@(negedge clk);if(!valid || y0!==24)$fatal(1,"highlight expiry must restore card");drain;
 contact=2;levels[7:4]=8;repeat(4)@(negedge clk);
 if(!valid || x0!==215 || x1!==313 || y0!==201 || y1!==232)$fatal(1,"new STOP rectangle not urgent");drain;
 $display("PASS V4 position/ADC/target and highlight refresh; settings regrouping; new STOP priority rectangle");$finish;
end
initial begin #1000000;$fatal(1,"scheduler timeout");end
endmodule
