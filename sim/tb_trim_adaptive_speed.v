// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module trim_speed_case #(parameter integer TAU_MS=300,BIAS=105,COUNTS=1040)(output reg done=0);
reg clk=0,rst_n=0,enable=0,tick=0;
always #5 clk=~clk;
reg signed [31:0] fast_actual=BIAS,slow_actual=BIAS;
wire signed [31:0] fast_target,slow_target;
wire signed [17:0] fast_offset,slow_offset;
wire [7:0] fast_status,slow_status;
target_trim_supervisor fast(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(1'b0),
 .target_change(1'b0),.requested(32'sd0),.actual(fast_actual),.tolerance_q8(32'(1479*COUNTS/1040)),.max_trim(16'(173*COUNTS/1040)),.travel_limit(16'd4000),
 .counts_per_rev(16'(COUNTS)),.internal_target(fast_target),.offset(fast_offset),.status(fast_status));
target_trim_supervisor #(.ADAPTIVE_STEP(0)) slow(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(1'b0),
 .target_change(1'b0),.requested(32'sd0),.actual(slow_actual),.tolerance_q8(32'(1479*COUNTS/1040)),.max_trim(16'(173*COUNTS/1040)),.travel_limit(16'd4000),
 .counts_per_rev(16'(COUNTS)),.internal_target(slow_target),.offset(slow_offset),.status(slow_status));
real fast_state,slow_state,fast_min,fast_max,ripple,band;
integer t,fast_time,slow_time,fast_dwell,slow_dwell,previous,largest_step;
initial begin
 fast_state=BIAS;slow_state=BIAS;fast_min=0;fast_max=0;fast_time=-1;slow_time=-1;
 fast_dwell=0;slow_dwell=0;previous=0;largest_step=0;band=8.0*COUNTS/1040;
 repeat(3)@(negedge clk);rst_n=1;enable=1;
 for(t=0;t<60000;t=t+1)begin

  fast_state=fast_state+($itor(fast_target)+BIAS-fast_state)/TAU_MS;
  slow_state=slow_state+($itor(slow_target)+BIAS-slow_state)/TAU_MS;
  ripple=5.0*COUNTS/1040*$sin(6.283185307179586*t/1200.0);
  fast_actual=$rtoi(fast_state+ripple);slow_actual=$rtoi(slow_state+ripple);
  tick=1;@(negedge clk);tick=0;@(negedge clk);
  if(fast_offset-previous>largest_step)largest_step=fast_offset-previous;
  if(previous-fast_offset>largest_step)largest_step=previous-fast_offset;
  previous=fast_offset;
  if(BIAS>0 && fast_state<fast_min)fast_min=fast_state;
  if(BIAS<0 && fast_state>fast_max)fast_max=fast_state;
  if(fast_state<=band && fast_state>=-band)fast_dwell=fast_dwell+1;else fast_dwell=0;
  if(slow_state<=band && slow_state>=-band)slow_dwell=slow_dwell+1;else slow_dwell=0;
  if(fast_time<0 && fast_dwell>=1500)fast_time=t-1499;
  if(slow_time<0 && slow_dwell>=1500)slow_time=t-1499;
 end
 if(fast_time<0 || fast_time>12000 || slow_time<0 || fast_time*100>slow_time*65)
  $fatal(1,"insufficient improvement tau=%0d bias=%0d counts=%0d fast=%0dms slow=%0dms",TAU_MS,BIAS,COUNTS,fast_time,slow_time);
 if(fast_min< -15.0*COUNTS/1040 || fast_max>15.0*COUNTS/1040)
  $fatal(1,"excess overshoot tau=%0d bias=%0d min=%0f max=%0f",TAU_MS,BIAS,fast_min,fast_max);
 if(largest_step>fast.fast_step || largest_step<2)$fatal(1,"adaptive step rate not bounded or never accelerated");
 $display("PASS adaptive response tau=%0dms bias=%0d counts=%0d: fast=%0dms previous=%0dms; overshoot=%0f/%0f counts; largest step=%0d",TAU_MS,BIAS,COUNTS,fast_time,slow_time,fast_min,fast_max,largest_step);
 done=1;
end
endmodule
module tb_trim_adaptive_speed;
wire [5:0] done;
trim_speed_case #(.TAU_MS(100)) a(done[0]);
trim_speed_case #(.TAU_MS(300)) b(done[1]);
trim_speed_case #(.TAU_MS(600)) c(done[2]);
trim_speed_case #(.TAU_MS(1000)) d(done[3]);
trim_speed_case #(.TAU_MS(300),.BIAS(-105)) e(done[4]);
trim_speed_case #(.TAU_MS(300),.BIAS(210),.COUNTS(2080)) f(done[5]);
initial begin wait(&done);$display("PASS: adaptive correction faster than previous fixed step, +/- bias, multiple mechanical lags, encoder scale, braking and bounded increments");$finish;end
initial begin #2000000;$fatal(1,"timeout");end
endmodule
