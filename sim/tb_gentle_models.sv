// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module gentle_case #(parameter real WN=2.0,ZETA=0.2,BIAS=105.0)(output reg done=0);
reg clk=0,rst_n=0,enable=0,tick=0,change=0;always #5 clk=~clk;
reg signed[31:0] goal=0,actual=105;wire signed[31:0] reference;wire signed[17:0] offset;wire[7:0] status;
target_trim_supervisor #(.FAST_SUPPORTED(1),.TUNING_SUPPORTED(1)) dut(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(1'b0),.target_change(change),.fast_mode(1'b1),.response_change(1'b0),.correction_strength(2'd0),.correction_rate_q8(16'd44),.requested(goal),.actual(actual),.tolerance_q8(32'd2219),.max_trim(16'd173),.travel_limit(16'd2000),.counts_per_rev(16'd1040),.internal_target(reference),.offset(offset),.status(status));
real pos,vel=0,low,high;integer t,previous=0,at_second=0,held;
task ms;begin
 vel=vel+(WN*WN*($itor(reference)+BIAS-pos)-2.0*ZETA*WN*vel)*0.001;pos=pos+vel*0.001;actual=$rtoi(pos);
 tick=1;@(negedge clk);tick=0;@(negedge clk);
 if(offset-previous>1 || previous-offset>1)$fatal(1,"gentle update exceeded one count");previous=offset;
 if(offset>173 || offset< -173 || reference>2000 || reference< -2000)$fatal(1,"gentle limits");
end endtask
task phase(input integer target);begin
 held=offset;goal=target;change=1;@(negedge clk);change=0;
 if(offset!=held || reference!=goal+held)$fatal(1,"gentle target handoff");
 low=1e9;high=-1e9;at_second=offset;
 for(t=0;t<45000;t=t+1)begin
  ms();
  if(t%1000==999)begin if(offset-at_second>10 || at_second-offset>10)$fatal(1,"3deg/s rate exceeded");at_second=offset;end
  if(t>=35000)begin if(pos-goal<low)low=pos-goal;if(pos-goal>high)high=pos-goal;end
 end
 if(high-low>20 || pos-goal>11 || pos-goal< -11)$fatal(1,"gentle failed to settle wn=%f zeta=%f goal=%d err=%f last10span=%f",WN,ZETA,goal,pos-goal,high-low);
end endtask
initial begin
 pos=BIAS;actual=$rtoi(pos);repeat(3)@(negedge clk);rst_n=1;repeat(3)@(negedge clk);enable=1;
 phase(0);phase(87);phase(-173);phase(347);
 $display("PASS gentle second-order wn=%0f zeta=%0f bias=%0f: +/-targets,3deg/s bound, quantized steps, final model stability; not mechanical validation",WN,ZETA,BIAS);done=1;
end
endmodule
module tb_gentle_models;
wire[2:0] done;
gentle_case #(.WN(2.0),.ZETA(0.2)) a(done[0]);
gentle_case #(.WN(4.0),.ZETA(0.4)) b(done[1]);
gentle_case #(.WN(1.5),.ZETA(0.3),.BIAS(-105.0)) c(done[2]);
initial begin wait(&done);$display("PASS gentle model matrix, low damping/slow plant/negative bias, no uncontrolled sustained growth");$finish;end
initial begin #8000000;$fatal(1,"gentle model timeout");end
endmodule
