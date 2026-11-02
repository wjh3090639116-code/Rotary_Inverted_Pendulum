// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_signed_stage2;
reg clk=0,rst_n=0,en=0,fresh=0,valid=1,reverse=0;
always #500 clk=~clk;
reg [11:0] adc=1600;
reg signed [15:0] first=600,second=200;
wire signed [15:0] pwm;wire stage2,done,event_out;wire [15:0] count;
swing_stage1_pulse #(.CLK_HZ(1000000),.MAX_STEP(80)) dut(
 .clk(clk),.rst_n(rst_n),.enable(en),.sample_strobe(fresh),.adc_valid(valid),
 .adc_code(adc),.wrap_span(12'd2640),.wrap_half_span(12'd1320),
 .bottom_code(12'd1600),.target_code(12'd1130),.upper_code(12'd730),
 .bottom_window(12'd20),.rearm_distance(12'd100),.first_pwm(first),.pulse_ms(16'd1),
 .angle_reverse(reverse),.two_stage_enable(1'b1),.stage2_pwm(second),.stage2_ms(16'd2),
 .upright_code(12'd730),.capture_window(12'd38),.stage2_active(stage2),
 .command(pwm),.done(done),.pulse_event(event_out),.pulse_count(count));
integer stage_events=0;reg signed [15:0] previous=0;
always @(negedge clk)if(event_out)begin
 if(previous!=0 && previous[15]==pwm[15])$fatal(1,"consecutive same-sign pulses");
 if(stage2)begin
  stage_events=stage_events+1;
  if(pwm!==dut.next_command || (pwm!=second && pwm!=-second))$fatal(1,"signed second-stage direction mismatch");
 end
 previous=pwm;
end
task sample(input [11:0] value);begin
 @(negedge clk);adc=value;fresh=1;@(negedge clk);fresh=0;repeat(998)@(negedge clk);
end endtask
task reset_case;begin
 en=0;rst_n=0;previous=0;stage_events=0;repeat(5)@(negedge clk);rst_n=1;en=1;
 sample(1600);sample(1600);
 if(pwm!==first)$fatal(1,"first-stage sign changed");
end endtask
integer i,v,j;reg signed [15:0] expected;
initial begin
 for(i=0;i<8;i=i+1)begin
  first=(i&1) ? -600 : 600;second=(i&2) ? -200 : 200;reverse=(i&4)!=0;
  reset_case;
  for(v=1550;v>=1150;v=v-50)sample(v);
  sample(1130);sample(1129);sample(1128);
  if(!stage2 || done)$fatal(1,"stage2 transition");

  if(dut.configured_pwm!==second)$fatal(1,"stage2 inherited stage1 direction");
  expected=reverse ? second : -second;
  for(v=1150;v<=1600;v=v+50)sample(v);sample(1605);
  if(expected[15]!=first[15])begin
   if(pwm!==expected || stage_events!=1)$fatal(1,"first eligible return wrong signed pulse case=%d",i);
  end else if(stage_events!=0)$fatal(1,"alternation guard bypassed at phase transition");
  for(j=0;j<2;j=j+1)begin
   for(v=1650;v<=1800;v=v+50)sample(v);
   for(v=1750;v>=1400;v=v-50)sample(v);
   for(v=1450;v<=1600;v=v+50)sample(v);sample(1605);
  end
  if(stage_events<3)$fatal(1,"signed setting stopped alternating pulses case=%d events=%d",i,stage_events);
  valid=0;repeat(10)@(negedge clk);if(pwm!==0)$fatal(1,"ADC invalid didn't stop signed pulse");
  valid=1;en=0;repeat(10)@(negedge clk);if(pwm!==0 || stage2)$fatal(1,"STOP/reset");
 end
 $display("PASS all 8 first/second PWM sign and angle-reverse combinations; independent sign, alternation, phase transition, ADC and STOP guards");$finish;
end
initial begin #(64'd2000000000);$fatal(1,"signed stage2 timeout");end
endmodule
