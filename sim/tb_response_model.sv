// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_response_model;
reg clk=0,rst_n=0,tick=0;always #5 clk=~clk;
reg signed[31:0] slow_actual=0,fast_actual=0;
wire signed[31:0] slow_target,fast_target;wire signed[17:0] a,b;wire[7:0] sa,sb;
target_trim_supervisor #(.FAST_SUPPORTED(1)) slow(.clk(clk),.rst_n(rst_n),.enable(1'b1),.ms_tick(tick),.reset_history(1'b0),.target_change(1'b0),.fast_mode(1'b0),.response_change(1'b0),.requested(32'sd0),.actual(slow_actual),.tolerance_q8(32'd2219),.max_trim(16'd173),.travel_limit(16'd2000),.counts_per_rev(16'd1040),.internal_target(slow_target),.offset(a),.status(sa));
target_trim_supervisor #(.FAST_SUPPORTED(1)) fast(.clk(clk),.rst_n(rst_n),.enable(1'b1),.ms_tick(tick),.reset_history(1'b0),.target_change(1'b0),.fast_mode(1'b1),.response_change(1'b0),.requested(32'sd0),.actual(fast_actual),.tolerance_q8(32'd2219),.max_trim(16'd173),.travel_limit(16'd2000),.counts_per_rev(16'd1040),.internal_target(fast_target),.offset(b),.status(sb));
real slow_state=-105,fast_state=-105;integer t,slow_time=-1,fast_time=-1,sd=0,fd=0;
initial begin
 repeat(3)@(negedge clk);rst_n=1;
 for(t=0;t<20000;t=t+1)begin
  slow_state=slow_state+($itor(slow_target)-105-slow_state)/200.0;
  fast_state=fast_state+($itor(fast_target)-105-fast_state)/200.0;
  slow_actual=$rtoi(slow_state);fast_actual=$rtoi(fast_state);
  tick=1;@(negedge clk);tick=0;@(negedge clk);
  sd=(slow_actual<=8 && slow_actual>=-8) ? sd+1:0;
  fd=(fast_actual<=8 && fast_actual>=-8) ? fd+1:0;
  if(sd>=1000 && slow_time<0)slow_time=t-999;
  if(fd>=1000 && fast_time<0)fast_time=t-999;
 end
 if(fast_time<0 || slow_time<0 || fast_time*10>slow_time*8)$fatal(1,"model improvement insufficient fast=%d slow=%d",fast_time,slow_time);
 $display("PASS first-order200ms biased plant only: fast settles=%0dms original=%0dms; model result not physical guarantee",fast_time,slow_time);$finish;
end
endmodule
