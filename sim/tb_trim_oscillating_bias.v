// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_trim_oscillating_bias;
reg clk=0,rst_n=0,enable=0,tick=0,reset=0;
always #5 clk=~clk;
reg signed [31:0] requested=0,actual=105;
reg [15:0] max_trim=173;
wire signed [31:0] internal_target;
wire signed [17:0] offset;
wire [7:0] status;
target_trim_supervisor dut(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(reset),
 .target_change(1'b0),.requested(requested),.actual(actual),.tolerance_q8(32'd1479),.max_trim(max_trim),.travel_limit(16'd2000),
 .counts_per_rev(16'd1040),.internal_target(internal_target),.offset(offset),.status(status));
task cycle;begin @(negedge clk);end endtask
task ms;begin tick=1;cycle();tick=0;cycle();end endtask
task fresh;begin reset=1;cycle();reset=0;end endtask
integer t,phase,ripple,positive_offset,negative_offset;
integer total,seen_correcting;
initial begin
 repeat(3)cycle();rst_n=1;cycle();enable=1;fresh();total=0;seen_correcting=0;

 for(t=0;t<40000;t=t+1)begin
  phase=t%1200;ripple=phase<600 ? -10+phase/30 : 30-phase/30;
  actual=internal_target+105+ripple;ms();
  if(t==999 && offset!=0)$fatal(1,"trim before settle delay");
  if(status[1])seen_correcting=1;
  if(t>=34000)total=total+actual;
 end
 positive_offset=offset;
 if(!seen_correcting || positive_offset>=-87 || total>7*6000 || total< -7*6000)
  $fatal(1,"positive oscillating bias did not converge: offset=%0d mean=%0f",offset,total/6000.0);
 if(requested!=0)$fatal(1,"user target changed");

 actual=-105;fresh();total=0;
 for(t=0;t<40000;t=t+1)begin
  phase=t%1200;ripple=phase<600 ? -10+phase/30 : 30-phase/30;
  actual=internal_target-105+ripple;ms();
  if(t>=34000)total=total+actual;
 end
 negative_offset=offset;
 if(negative_offset<=87 || total>7*6000 || total< -7*6000)
  $fatal(1,"negative oscillating bias did not converge: offset=%0d mean=%0f",offset,total/6000.0);

 actual=0;fresh();
 for(t=0;t<8000;t=t+1)begin
  phase=t%200;ripple=phase<100 ? -10+phase/5 : 30-phase/5;
  actual=ripple;ms();
 end
 if(offset!=0)$fatal(1,"integrated a zero-mean oscillation");

 actual=0;fresh();for(t=0;t<2400;t=t+1)ms();
 for(t=0;t<400;t=t+1)begin actual=t*3;ms();end
 if(offset!=0 || status[2])$fatal(1,"trim chased fast transit");
 enable=0;cycle();if(offset!=0)$fatal(1,"disable reset");
 $display("PASS: +/-36deg oscillating plant bias converges, no 30deg eligibility cutoff, requested stays fixed, 1s average suppresses centered ripple, fast-transit guard, bounded rate/offset. final offsets=%0d/%0d",positive_offset,negative_offset);$finish;
end
initial begin #5000000;$fatal(1,"timeout");end
endmodule
