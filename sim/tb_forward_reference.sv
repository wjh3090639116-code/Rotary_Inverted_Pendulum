// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_forward_reference;
reg clk=0;always #5 clk=~clk;
reg rst_n=0,tick=0,enable=1,start=0,cancel=0;
reg signed[31:0] goal=0,actual=0;wire signed[31:0] ref_value;wire ramping;
turn_motion_reference dut(.clk(clk),.rst_n(rst_n),.ms_tick(tick),.enable(enable),.start(start),.cancel(cancel),.goal(goal),.actual(actual),.reference(ref_value),.ramping(ramping));
task cycles(input integer n);repeat(n)@(negedge clk);endtask
task ms;begin tick=1;cycles(1);tick=0;cycles(1);end endtask
task begin_turn(input integer from_value,input integer to_value);begin
 enable=0;goal=from_value;actual=from_value;cycles(2);enable=1;start=1;cycles(1);start=0;goal=to_value;cycles(2);
 if(!ramping || ref_value!=from_value)$fatal(1,"start jumped to final target");
end endtask
integer i,previous,count20,count40,count80,count120;reg signed[24:0] old_q8;integer dq;
initial begin
 cycles(4);rst_n=1;cycles(2);
 begin_turn(0,1040);

 for(i=0;i<3000;i=i+1)ms;
 if(ref_value>174 || !ramping)$fatal(1,"lead guard or false completion");
 begin_turn(-1040,0);previous=-1040;count20=0;count40=0;count80=0;count120=0;
 for(i=0;i<5500 && ramping;i=i+1)begin
  actual=ref_value;old_q8=dut.reference_q8;ms;
  if(ramping)begin
   dq=dut.reference_q8-old_q8;
   if(dq<0 || dq>1774)$fatal(1,"reference rate/monotonicity %d",dq);
   case(dq)1774:count120=count120+1;1183:count80=count80+1;591:count40=count40+1;295:count20=count20+1;endcase
  end
  if(ref_value<previous || ref_value>goal)$fatal(1,"signed reference overshoot");
  previous=ref_value;
 end
 if(ramping || ref_value!=0 || i<3000 || i>5000)$fatal(1,"profile completion %d",i);
 if(count20==0 || count40==0 || count80==0 || count120==0)$fatal(1,"missing early-deceleration stages");
 begin_turn(0,1040);for(i=0;i<300;i=i+1)begin actual=ref_value;ms;end
 previous=ref_value;start=1;cycles(1);start=0;goal=2080;cycles(2);
 if(ref_value!=previous)$fatal(1,"new forward target jumped reference");
 cancel=1;goal=-1040;cycles(1);cancel=0;cycles(1);
 if(ramping || ref_value!=-1040)$fatal(1,"ordinary/reverse cancel did not bypass");
 begin_turn(0,1040);actual=131072;old_q8=dut.reference_q8;
 for(i=0;i<100;i=i+1)ms;
 if(dut.reference_q8!=old_q8)$fatal(1,"out-of-range actual aliased");
 enable=0;cycles(2);if(ramping || ref_value!=goal)$fatal(1,"STOP/fault bypass");
 $display("tb_forward_reference PASS");$finish;
end
initial begin #2000000;$fatal(1,"timeout");end
endmodule
