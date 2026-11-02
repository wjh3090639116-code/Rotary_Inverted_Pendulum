// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_trim_target_handoff;
reg clk=0,rst_n=0,enable=0,tick=0,reset=0,change=0;
always #5 clk=~clk;
reg signed [31:0] requested=0,actual=105;
wire signed [31:0] internal_target;
wire signed [17:0] offset;
wire [7:0] status;
target_trim_supervisor #(.SETTLE_MS(5),.WINDOW_MS(4)) dut(
 .clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(reset),.target_change(change),
 .requested(requested),.actual(actual),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd1000),
 .counts_per_rev(16'd1040),.internal_target(internal_target),.offset(offset),.status(status));
task cycle;begin @(negedge clk);end endtask
task ms;begin tick=1;cycle();tick=0;cycle();end endtask
integer t,held,before_reference;
task target;input integer value;begin
 held=offset;before_reference=internal_target;
 change=1;requested=value;cycle();change=0;
 if(offset!=held || internal_target!=value+held)$fatal(1,"new target first clock did not carry bias");
 if(!status[4])$fatal(1,"new target did not suspend learning");
end endtask
initial begin
 repeat(3)cycle();rst_n=1;cycle();enable=1;

 for(t=0;t<240;t=t+1)begin actual=internal_target+105;ms();end
 if(offset> -95 || offset< -110)$fatal(1,"initial bias learning failed %0d",offset);
 held=offset;target(87);

 if(internal_target-before_reference!=87)$fatal(1,"target increment includes erased bias");
 for(t=0;t<100;t=t+1)begin actual=t*2;ms();
  if(offset!=held || !status[4])$fatal(1,"learning chased target transit %0d",t);
 end

 actual=112;
 for(t=0;t<12;t=t+1)begin ms();if(offset!=held)$fatal(1,"relearned before full settled window");end
 for(t=0;t<160;t=t+1)begin actual=internal_target+125;ms();end
 if(offset>=held || offset< -140 || offset> -115)$fatal(1,"new steady bias not learned %0d",offset);
 if(requested!=87)$fatal(1,"requested target overwritten");
 target(174);held=offset;
 for(t=0;t<8;t=t+1)begin actual=200+t*4;ms();end
 if(offset!=held)$fatal(1,"rapid repeated target changed bias");
 target(-87);held=offset;
 for(t=0;t<40;t=t+1)begin actual=230-t*5;ms();end
 if(offset!=held)$fatal(1,"reversal changed bias during transit");

 reset=1;cycle();reset=0;
 if(offset!=0 || internal_target!=requested)$fatal(1,"capture/config did not clear stale bias");
 for(t=0;t<240;t=t+1)begin actual=internal_target+105;ms();end
 if(offset==0)$fatal(1,"bias relearning failed");
 enable=0;cycle();if(offset!=0)$fatal(1,"stop did not clear bias");
 $display("PASS: learned bias carried on first target clock, actual reference step equals requested step, transit/overshoot never integrated, settled bias relearned, positive/reverse/repeated steps, stop/capture reset");$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
