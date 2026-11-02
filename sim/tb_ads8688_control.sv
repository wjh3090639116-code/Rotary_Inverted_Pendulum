// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_ads8688_control;
reg clk=0,rst_n=0,rxd=1;always #10 clk=~clk;
reg [15:0] input_code=16'hbb8f;reg [1:0] fault=0;
wire cs,sclk,sdi,sdo,tx,a,b,p,stby;
ads8688_uart_top #(.ENABLE_SCREEN(0),.BAUD(1000000),.ADC_BOOT_CYCLES(200),.ADC_WARMUP_CYCLES(1000)) dut(
 .clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.adc_cs_n(cs),.adc_sclk(sclk),
 .adc_sdi(sdi),.adc_sdo(sdo),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),
 .uart_txd(tx),.motor_in1(a),.motor_in2(b),.motor_pwm(p),.motor_stby(stby));
ads8688_input_model model(.cs_n(cs),.sclk(sclk),.sdi(sdi),
 .host_rst_n(rst_n),.fault(fault),.sdo(sdo),.input_code(input_code));
integer samples=0,ticks=0;time last_sample=0,last_tick=0;
always @(negedge clk)if(rst_n)begin
 if(dut.tick)begin
  if(last_tick && $time-last_tick!=1000000)$fatal(1,"control tick period changed");
  last_tick=$time;ticks=ticks+1;
 end
 if(dut.adc_sample_pulse && dut.adc_valid)begin
  if(last_sample && $time-last_sample!=1000000)$fatal(1,"filter cadence changed");
  last_sample=$time;samples=samples+1;
  if(dut.adc_filter_raw!==3000 || dut.adc_code!==3000)$fatal(1,"high12 code >2047 corrupted");
 end
end
task byte_tx(input [7:0] value);integer n;begin
 @(negedge clk);rxd=0;repeat(50)@(negedge clk);
 for(n=0;n<8;n=n+1)begin rxd=value[n];repeat(50)@(negedge clk);end
 rxd=1;repeat(50)@(negedge clk);
end endtask
task frame(input [7:0] kind,len,input [239:0] payload);integer n;reg [7:0] sum;begin
 sum=kind^len;byte_tx(8'ha5);byte_tx(8'h5a);byte_tx(kind);byte_tx(len);
 for(n=0;n<len;n=n+1)begin byte_tx(payload[8*n+:8]);sum=sum^payload[8*n+:8];end
 byte_tx(sum);repeat(500)@(negedge clk);
end endtask
integer bytes_seen=0,fh,bit_index;reg [7:0] received_byte;
initial begin
 fh=$fopen("control_uart_capture.hex","w");
 forever begin
  @(negedge tx);#1500;
  for(bit_index=0;bit_index<8;bit_index=bit_index+1)begin received_byte[bit_index]=tx;#1000;end
  if(!tx)$fatal(1,"TX stop bit");
  $fwrite(fh,"%02x\n",received_byte);bytes_seen=bytes_seen+1;
 end
end
initial begin
 #100;rst_n=1;wait(dut.adc_valid);$display("ADC ready at %t",$time);
 if(stby || dut.motor_enable)$fatal(1,"unconfigured motor enabled");

 frame(8'h22,30,{32'd0,32'd0,32'd0,32'd0,32'd256,16'd2000,16'd301,16'd61,8'd1,8'd1,16'd3000});
 if(dut.config_loaded)$fatal(1,"legacy ADC scaling config accepted");
 frame(8'h32,30,{32'd0,32'd0,32'd0,32'd0,32'd256,16'd2000,16'd301,16'd61,8'd1,8'd1,16'd3000});
 if(!dut.config_loaded || dut.zero_code!==3000)$fatal(1,"unsigned high12 zero rejected");
 frame(8'h14,4,{208'd0,16'd4095,16'd0});
 wait(dut.adc_valid);
 frame(8'h12,1,240'd2);$display("Manual sent at %t state=%d request=%b adc=%b code=%d zero=%d",$time,dut.state,dut.run_request,dut.adc_valid,dut.adc_code,dut.zero_code);
 wait(dut.state==2);#1000;
 if(!stby || dut.angle!==0)$fatal(1,"manual capture with >2047 code failed");
 #21000000;
 fault=1;wait(!dut.adc_valid);#100;
 if(stby || dut.motor_enable || p)$fatal(1,"SPI failure did not immediately disable motor");
 wait(dut.state==3);if(dut.fault_reason!=1)$fatal(1,"ADC failure reason not preserved");
 if(samples<20 || ticks<20 || bytes_seen<30)$fatal(1,"missing acquisition/telemetry");
 $fclose(fh);
 $display("PASS integration: actual SPI -> raw16[15:4] -> filter -> UART/manual PID at code3000, 1kHz, old-config rejection, runtime ADC fault disables motor");
 $finish;
end
initial begin #40000000;$fatal(1,"control integration timeout state=%d samples=%d link=%b filter=%b error=%d age=%d",dut.state,samples,dut.adc_link_valid,dut.filter_valid,dut.adc_error,dut.u_adc.age);end
endmodule
