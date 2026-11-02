// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module continuous_case #(parameter integer TAU=200,NEGATIVE=0)(output reg done=0);
reg clk=0,rst_n=0,tick=0,change=0;always #5 clk=~clk;
reg signed[31:0] goal=0,af=0,aslow=0;
wire signed[31:0] rf,rs;wire signed[17:0] ofast,oslow;wire[7:0] sf,ss;
target_trim_supervisor #(.FAST_SUPPORTED(1)) fast(.clk(clk),.rst_n(rst_n),.enable(1'b1),.ms_tick(tick),.reset_history(1'b0),.target_change(change),.fast_mode(1'b1),.response_change(1'b0),.requested(goal),.actual(af),.tolerance_q8(32'd2219),.max_trim(16'd173),.travel_limit(16'd2000),.counts_per_rev(16'd1040),.internal_target(rf),.offset(ofast),.status(sf));
target_trim_supervisor #(.FAST_SUPPORTED(1)) slow(.clk(clk),.rst_n(rst_n),.enable(1'b1),.ms_tick(tick),.reset_history(1'b0),.target_change(change),.fast_mode(1'b0),.response_change(1'b0),.requested(goal),.actual(aslow),.tolerance_q8(32'd2219),.max_trim(16'd173),.travel_limit(16'd2000),.counts_per_rev(16'd1040),.internal_target(rs),.offset(oslow),.status(ss));
real pf=0,ps=0,bias=116;integer t,phase,previous=0,held,fast_dwell,slow_dwell,fast_time,slow_time,fast_total=0,slow_total=0;
task ms;begin
 pf=pf+($itor(rf)+bias-pf)/TAU;ps=ps+($itor(rs)+bias-ps)/TAU;
 af=$rtoi(pf);aslow=$rtoi(ps);
 tick=1;@(negedge clk);tick=0;@(negedge clk);
 if(ofast>173 || ofast< -173 || rf>2000 || rf< -2000)$fatal(1,"continuous limits");
 if(ofast-previous>2 || previous-ofast>2)$fatal(1,"continuous step rate");previous=ofast;
end endtask
task target(input integer value);begin
 held=ofast;bias=value>=goal ? 116.0:67.0;if(NEGATIVE!=0)bias=-bias;
 goal=value;change=1;@(negedge clk);change=0;
 if(ofast!=held || rf!=goal+held)$fatal(1,"new target did not retain bias");
 fast_dwell=0;slow_dwell=0;fast_time=-1;slow_time=-1;
 for(t=0;t<12000;t=t+1)begin
  ms();fast_dwell=(af-goal<=8 && af-goal>=-8) ? fast_dwell+1:0;
  slow_dwell=(aslow-goal<=8 && aslow-goal>=-8) ? slow_dwell+1:0;
  if(fast_dwell>=500 && fast_time<0)fast_time=t-499;
  if(slow_dwell>=500 && slow_time<0)slow_time=t-499;
 end
 if(fast_time<0)$fatal(1,"continuous never settled tau=%d goal=%d actual=%d",TAU,goal,af);
 if(slow_time>=0)begin fast_total=fast_total+fast_time;slow_total=slow_total+slow_time;end
 if(af-goal>10 || af-goal< -10)$fatal(1,"continuous final error");
end endtask
initial begin
 repeat(3)@(negedge clk);rst_n=1;
 for(t=0;t<12000;t=t+1)ms();
 target(693);target(-371);target(1040);target(217);target(-260);target(0);
 if(fast_total>=slow_total)$fatal(1,"no aggregate improvement tau=%d fast=%d old=%d",TAU,fast_total,slow_total);
 $display("PASS arbitrary targets, direction-dependent bias +/- sign tau=%d: cumulative continuous=%dms baseline=%dms (models only)",TAU,fast_total,slow_total);done=1;
end
endmodule
module tb_continuous_models;
wire[3:0] done;
continuous_case #(.TAU(100)) a(done[0]);continuous_case #(.TAU(200)) b(done[1]);
continuous_case #(.TAU(400)) c(done[2]);continuous_case #(.TAU(800),.NEGATIVE(1)) d(done[3]);
initial begin wait(&done);$display("PASS continuous arbitrary-angle matrix: multiple lags, positive/negative directional biases, target handoff, bounded steps/bias/travel; not physical validation");$finish;end
initial begin #4000000;$fatal(1,"model timeout");end
endmodule
