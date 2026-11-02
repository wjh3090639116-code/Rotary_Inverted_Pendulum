// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_response_inertia;
reg clk=0,rst_n=0,enable=0,tick=0,change=0;
always #5 clk=~clk;
reg signed [31:0] requested=0,actual_new=105,actual_reset=105;
wire signed [31:0] reference_new,reference_reset;
wire signed [17:0] offset_new,offset_reset;
wire [7:0] status_new,status_reset;
target_trim_supervisor #(.FAST_SUPPORTED(1)) carried(.fast_mode(1'b1),.response_change(1'b0),.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(1'b0),.target_change(change),
 .requested(requested),.actual(actual_new),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd2000),
 .counts_per_rev(16'd1040),.internal_target(reference_new),.offset(offset_new),.status(status_new));

target_trim_supervisor #(.FAST_SUPPORTED(1)) cleared(.fast_mode(1'b1),.response_change(1'b0),.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(change),.target_change(1'b0),
 .requested(requested),.actual(actual_reset),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd2000),
 .counts_per_rev(16'd1040),.internal_target(reference_reset),.offset(offset_reset),.status(status_reset));
real pos_new=105.0,pos_reset=105.0,vel_new=0.0,vel_reset=0.0;
real peak_new,peak_reset;
integer t,held,old_reference,old_offset,old_reset_reference;
task ms;begin

 vel_new=vel_new+(64.0*($itor(reference_new)+105.0-pos_new)-12.8*vel_new)*0.001;
 vel_reset=vel_reset+(64.0*($itor(reference_reset)+105.0-pos_reset)-12.8*vel_reset)*0.001;
 pos_new=pos_new+vel_new*0.001;pos_reset=pos_reset+vel_reset*0.001;
 actual_new=$rtoi(pos_new);actual_reset=$rtoi(pos_reset);
 tick=1;@(negedge clk);tick=0;@(negedge clk);
end endtask
task set_target;input integer value;integer delta;begin
 held=offset_new;old_reference=reference_new;old_reset_reference=reference_reset;old_offset=offset_reset;
 delta=value-requested;requested=value;change=1;@(negedge clk);change=0;
 if(offset_new!=held || reference_new-old_reference!=delta)$fatal(1,"carried reference step enlarged");
 if(reference_reset-old_reset_reference!=delta-old_offset)$fatal(1,"legacy reset comparison invalid");
end endtask
initial begin
 repeat(3)@(negedge clk);rst_n=1;enable=1;
 for(t=0;t<10000;t=t+1)ms();
 if(offset_new> -95 || offset_reset> -95)$fatal(1,"initial bias not learned");
 set_target(87);peak_new=-10000;peak_reset=-10000;
 for(t=0;t<8000;t=t+1)begin
  ms();
  if(pos_new-requested>peak_new)peak_new=pos_new-requested;
  if(pos_reset-requested>peak_reset)peak_reset=pos_reset-requested;
 end
 if(peak_new>15.0 || peak_reset<60.0 || peak_new*4>peak_reset)
  $fatal(1,"inertia comparison new=%0f reset=%0f counts",peak_new,peak_reset);
 $display("PASS second-order +30deg step: peak over target carried=%0fdeg, previous reset=%0fdeg",peak_new*360/1040,peak_reset*360/1040);
 set_target(-87);for(t=0;t<8000;t=t+1)ms();
 if(pos_new-requested>8 || pos_new-requested< -8)$fatal(1,"reverse step did not return near target");
 set_target(174);for(t=0;t<8000;t=t+1)ms();
 if(pos_new-requested>8 || pos_new-requested< -8)$fatal(1,"next step lost compensation");
 $display("PASS: same learned bias carried across positive/reverse/repeated target commands; second-order inertia included. This does not guarantee physical zero overshoot");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
