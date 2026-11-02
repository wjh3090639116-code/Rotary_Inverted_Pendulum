// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_symmetric_reference;
reg clk=0;always #5 clk=~clk;
reg rst_n=0,tick=0,enable=1,start=0,cancel=0,mirror=0,checking=0;
reg signed[31:0] goal=0,actual=0;
wire signed[31:0] ref_value,baseline_ref;
wire ramping,baseline_ramping;
wire signed[31:0] baseline_goal=mirror ? -goal : goal;
wire signed[31:0] baseline_actual=mirror ? -actual : actual;
turn_motion_reference dut(.clk(clk),.rst_n(rst_n),.ms_tick(tick),.enable(enable),.start(start),.cancel(cancel),.goal(goal),.actual(actual),.reference(ref_value),.ramping(ramping));
forward_turn_reference_baseline baseline(.clk(clk),.rst_n(rst_n),.ms_tick(tick),.enable(enable),.start(start),.cancel(cancel),.goal(baseline_goal),.actual(baseline_actual),.reference(baseline_ref),.ramping(baseline_ramping));
always @(posedge clk)begin #1;if(checking)begin
 if(!mirror && ramping!==baseline_ramping)$fatal(1,"phase changed against verified forward baseline");
 if(!mirror && ref_value!==baseline_ref)$fatal(1,"positive path changed %d %d",ref_value,baseline_ref);

end end
task cycles(input integer n);repeat(n)@(negedge clk);endtask
task ms;begin tick=1;cycles(1);tick=0;cycles(1);end endtask
task run_profile(input integer from_value,to_value,input bit negative);integer i,previous,d;begin
 checking=0;enable=0;mirror=negative;goal=from_value;actual=from_value;cycles(3);
 enable=1;start=1;cycles(1);start=0;goal=to_value;cycles(2);checking=1;previous=ref_value;
 for(i=0;i<5500 && ramping;i=i+1)begin
  actual=ref_value;ms;d=ref_value-previous;
  if((negative && (d>0 || d< -11)) || (!negative && (d<0 || d>7)))$fatal(1,"direction or rate");
  if(negative ? ref_value<to_value : ref_value>to_value)$fatal(1,"reference passed endpoint");
  previous=ref_value;
 end
 if(ramping || ref_value!=to_value || i<1000 || i>5000)$fatal(1,"completion %d",i);
 if(negative && (!baseline_ramping || i>(from_value-to_value>600 ? 3000 : 2000)))$fatal(1,"reverse tuning did not shorten reference motion");
 checking=0;
end endtask
integer i,held;
initial begin
 cycles(3);rst_n=1;cycles(2);
 run_profile(-1040,0,0);run_profile(0,1040,0);
 run_profile(1040,0,1);run_profile(0,-1040,1);
 run_profile(0,520,0);run_profile(520,0,1);
 run_profile(-520,0,0);run_profile(0,-520,1);

 enable=0;goal=0;actual=0;cycles(2);enable=1;start=1;cycles(1);start=0;goal=1040;
 for(i=0;i<300;i=i+1)begin actual=ref_value;ms;end
 held=ref_value;start=1;cycles(1);start=0;goal=520;cycles(2);
 if(ref_value!=held)$fatal(1,"nominal reversal jumped reference");
 for(i=0;i<20;i=i+1)begin actual=ref_value;ms;end
 if(ref_value<=held)$fatal(1,"endpoint ahead should continue smoothly toward it");
 held=ref_value;start=1;cycles(1);start=0;goal=-520;cycles(2);
 if(ref_value!=held)$fatal(1,"direction reversal jumped reference");
 for(i=0;i<20;i=i+1)begin actual=ref_value;ms;end
 if(ref_value>=held)$fatal(1,"reverse reference did not decrease");

 enable=0;cancel=0;goal=0;actual=0;cycles(2);enable=1;start=1;cycles(1);start=0;goal=-1040;
 for(i=0;i<1500;i=i+1)ms;
 if(ref_value< -261 || ref_value> -259 || !ramping)$fatal(1,"reverse90deg lead guard %d",ref_value);
 held=ref_value;for(i=0;i<100;i=i+1)ms;
 if(ref_value!=held)$fatal(1,"reverse stalled guard did not hold");
 actual=-100;for(i=0;i<200;i=i+1)ms;
 if(ref_value>=held || ref_value< -361)$fatal(1,"reverse guard did not resume after actual moved");
 cancel=1;goal=173;cycles(2);if(ramping || ref_value!=173)$fatal(1,"ordinary target bypass");
 $display("tb_symmetric_reference PASS");$finish;
end
initial begin #5000000;$fatal(1,"timeout");end
endmodule
