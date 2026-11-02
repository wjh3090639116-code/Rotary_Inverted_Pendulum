// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_v4_pixels;
reg clk=0,rst_n=0,frame=0;always #5 clk=~clk;
reg [8:0] x=0;reg [7:0] y=0,status=7;reg [11:0] adc=1578;
reg signed [31:0] position=83,target=87;reg [8:0] step=60,amp=30;
wire [15:0] pixel;wire high;
tft_pixel #(.CLK_HZ(1000)) dut(.clk(clk),.rst_n(rst_n),.frame_start(frame),.draw_start(frame),
 .animation_levels(36'd0),.x(x),.y(y),.position(position),.target(target),.adc(adc),.zero(12'd1578),
 .step_degrees(step),.amplitude_degrees(amp),.status(status),.touch_down(1'b0),.cal_step(2'd3),.pressed(4'd0),.message(4'd0),.pixel(pixel),.goal_highlight(high));
task snapshot;begin repeat(3)@(negedge clk);frame=1;@(negedge clk);frame=0;repeat(100)@(negedge clk);end endtask
integer i,j,k,f;
initial begin
 repeat(5)@(negedge clk);rst_n=1;snapshot;
 if(dut.angle!==287 || dut.goal!==30 || dut.adc_snapshot!==1578)$fatal(1,"measured/target angle source or decimal conversion");
 if(!high)$fatal(1,"target update highlight missing");
 repeat(500)@(negedge clk);if(high)$fatal(1,"highlight did not expire after0.5s");
 for(k=0;k<2;k=k+1)begin
  if(k==1)begin position=-1040;target=1040;step=360;amp=360;status=71;end
  snapshot;
  if(k==1 && (dut.angle!==-3600 || dut.goal!==360))$fatal(1,"one turn display modulo/sign error");
  f=$fopen(k==0 ? "docs/v4_normal.ppm" : "docs/v4_target_changed.ppm","w");$fwrite(f,"P3\n320 240\n255\n");
  for(j=0;j<240;j=j+1)for(i=0;i<320;i=i+1)begin
   x=i;y=j;repeat(8)@(negedge clk);
   if(^pixel===1'bx)$fatal(1,"unknown pixel %d,%d",i,j);
   $fwrite(f,"%d %d %d\n",pixel[15:11]*255/31,pixel[10:5]*255/63,pixel[4:0]*255/31);
  end
  $fclose(f);
 end
 adc=4095;position=0;target=0;snapshot;
 if(dut.angle!==0 || dut.goal!==0 || dut.adc_snapshot!==4095)$fatal(1,"ADC unsigned or angle zero");
 adc=0;snapshot;if(dut.adc_snapshot!==0)$fatal(1,"ADC zero");
 position=1;target=-1;snapshot;if(dut.angle!==3 || dut.goal!==0)$fatal(1,"signed rounding");
 target=173;snapshot;if(dut.goal!==60)$fatal(1,"60deg nominal became quantized59.9deg");
 x=234;y=62;repeat(3)@(negedge clk);if(dut.decimal_flag || dut.glyph!==13)$fatal(1,"target decimal glyph retained");
 x=15;y=59;repeat(3)@(negedge clk);if(!dut.decimal_flag)$fatal(1,"actual precision removed");
 target=-173;snapshot;if(dut.goal!==-60)$fatal(1,"negative integer nominal");
 target=1;position=2147483647;snapshot;if(!dut.ao)$fatal(1,"out-of-range must be marked");
 position=1040;target=0;snapshot;target=87;repeat(100)@(negedge clk);
 if(dut.goal!==0)$fatal(1,"mid-region data snapshot changed");
 snapshot;if(dut.goal!==30)$fatal(1,"target refresh");
 $display("PASS V4 actual encoder angle independent of ADC/target; decimal/sign/full-turn/overflow; highlight0.5s; complete RGB565 screenshots");$finish;
end
initial begin #50000000;$fatal(1,"V4 pixel timeout");end
endmodule
