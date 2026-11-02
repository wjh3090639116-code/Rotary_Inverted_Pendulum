// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_target_trim;
reg clk=0,rst_n=0,enable=0,tick=0,reset=0;
always #5 clk=~clk;
reg signed [31:0] requested=0,actual=0;
reg [31:0] tolerance=256;
reg [15:0] max_trim=8,travel=100;
wire signed [31:0] internal_target;
wire signed [17:0] offset;
wire [7:0] status;
target_trim_supervisor #(.SETTLE_MS(5),.WINDOW_MS(4)) dut(
 .clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(reset),
 .target_change(1'b0),.requested(requested),.actual(actual),.tolerance_q8(tolerance),.max_trim(max_trim),.travel_limit(travel),
 .counts_per_rev(16'd1040),.internal_target(internal_target),.offset(offset),.status(status));
task cycle;begin @(negedge clk);end endtask
task ms;input integer n;integer i;begin
 for(i=0;i<n;i=i+1)begin tick=1;cycle();tick=0;cycle();end
end endtask
task fresh;begin reset=1;cycle();reset=0;end endtask
integer i,held;
initial begin
 repeat(3)cycle();rst_n=1;cycle();enable=1;requested=20;actual=16;fresh();
 ms(5);if(offset!=0)$fatal(1,"correction before settling");

 for(i=0;i<8;i=i+1)begin actual=internal_target-4;ms(4);end
 if(requested!=20 || offset!=3 || internal_target!=23)$fatal(1,"positive bias compensation/deadband %0d",offset);
 held=offset;ms(20);if(offset!=held)$fatal(1,"trim not retained within tolerance");
 requested=-20;actual=-16;fresh();ms(5);
 for(i=0;i<8;i=i+1)begin actual=internal_target+4;ms(4);end
 if(offset!=-3 || internal_target!=-23)$fatal(1,"negative bias compensation");
 requested=20;actual=-20;fresh();ms(80);
 if(offset!=8 || !status[3])$fatal(1,"large static error failed to correct to bound");
 requested=20;actual=16;fresh();ms(5);
 for(i=0;i<40;i=i+1)begin actual=(i%2)?60:-60;ms(1);end
 if(offset!=0)$fatal(1,"chased moving arm");
 actual=16;fresh();ms(80);
 if(offset!=8 || !status[3])$fatal(1,"max trim not bounded %0d",offset);
 requested=99;actual=-100;fresh();ms(80);
 if(offset!=1 || internal_target!=100 || !status[3])$fatal(1,"positive travel clamp");
 requested=-99;actual=100;fresh();ms(80);
 if(offset!=-1 || internal_target!=-100)$fatal(1,"negative travel clamp");
 requested=12;fresh();if(offset!=0 || internal_target!=12)$fatal(1,"new user target retained trim");
 enable=0;cycle();if(offset!=0 || internal_target!=12)$fatal(1,"stop did not reset");
 $display("PASS: separate requested/internal angles, settle delay, quiet window, positive/negative bias convergence, tolerance retention, no transit chasing, trim/travel bounds, reset/stop");$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
