// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_pwm_limit_uart;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire tx;
ads8688_uart_top #(.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000),.ADC_BOOT_CYCLES(100),.ADC_WARMUP_CYCLES(100)) dut(
 .clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.step_down_btn_n(1'b1),.amplitude_up_btn_n(1'b1),.amplitude_down_btn_n(1'b1),
 .adc_sdo(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.uart_txd(tx),.touch_irq_n(1'b1),.touch_miso(1'b0));
wire[7:0] rx;wire rv;
uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) mon(.clk(clk),.rst_n(rst_n),.rxd(tx),.byte_data(rx),.byte_valid(rv));
integer parse=0,len=0,idx=0,acks=0,before_acks;
reg[7:0] kind,crc;reg[239:0] payload;reg[23:0] last_limit;
always @(posedge clk)if(rv)case(parse)
 0:if(rx=='ha5)parse=1;
 1:if(rx=='h5a)parse=2;else parse=0;
 2:begin kind=rx;crc=rx;parse=3;end
 3:begin len=rx;crc=crc^rx;idx=0;payload=0;parse=len==0 ? 5:4;end
 4:begin payload[8*idx+:8]=rx;crc=crc^rx;idx=idx+1;if(idx==len)parse=5;end
 5:begin
  if(crc!=rx)$fatal(1,"wire checksum");
  if(kind=='h49)begin if(len!=3)$fatal(1,"limit length");acks=acks+1;last_limit=payload[23:0];end
  parse=0;
 end
endcase
task cycles(input integer n);repeat(n)@(negedge clk);endtask
task byte_tx(input[7:0] v);integer i;begin
 @(negedge clk);rxd=0;cycles(10);for(i=0;i<8;i=i+1)begin rxd=v[i];cycles(10);end rxd=1;cycles(10);
end endtask
task frame(input[7:0] typ,n,input[15:0] data,input bad);integer i;reg[7:0] sum;begin
 sum=typ^n;byte_tx('ha5);byte_tx('h5a);byte_tx(typ);byte_tx(n);
 for(i=0;i<n;i=i+1)begin byte_tx(data[i*8+:8]);sum=sum^data[i*8+:8];end byte_tx(sum^bad);cycles(5000);
end endtask
task set_limit(input[15:0] value);begin frame('h48,2,value,0);if(last_limit!={value,8'd0})$fatal(1,"limit ACK %h",last_limit);end endtask
task check_drive(input integer command,input integer duty_high,input bit negative);integer i,h;begin
 cycles(100);
 if($signed(dut.applied_command)!=command || $signed(dut.u_motor.latched)!=command)$fatal(1,"actual command exceeded ceiling %d %d",dut.applied_command,dut.u_motor.latched);
 if(command!=0 && (dut.motor_in1==negative || dut.motor_in2!=negative))$fatal(1,"direction changed");
 h=0;for(i=0;i<50;i=i+1)begin cycles(1);if(dut.motor_pwm)h=h+1;end
 if(h!=duty_high)$fatal(1,"physical PWM duty %d expected%d",h,duty_high);
end endtask
initial begin
 force dut.adc_code=12'd1578;force dut.adc_valid=1;force dut.adc_bad=0;force dut.adc_sample_pulse=0;
 cycles(5);rst_n=1;wait(!dut.profile_busy);cycles(10000);
 if(dut.pwm_output_limit!=900)$fatal(1,"legacy90percent default changed");
 frame('h2a,2,100,0);if(dut.pwm_deadzone!=100 || dut.pid_output_limit!=800)$fatal(1,"deadzone headroom");
 set_limit(500);if(dut.pid_output_limit!=400)$fatal(1,"PID must reserve deadzone headroom");
 force dut.run_request=1;force dut.state=3'd4;force dut.motor_enable=1;

 force dut.swing_command=16'sd550;check_drive(500,25,0);
 force dut.swing_command=-16'sd400;check_drive(-500,25,1);

 force dut.state=3'd2;force dut.angle_command=16'sd850;check_drive(500,25,0);
 force dut.angle_command=-16'sd850;check_drive(-500,25,1);
 set_limit(300);check_drive(-300,15,1);if(dut.pid_output_limit!=200)$fatal(1,"live cap antiwindup");
 set_limit(50);check_drive(-50,2,1);if(dut.pid_output_limit!=0)$fatal(1,"headroom unsigned underflow");
 set_limit(0);check_drive(0,0,0);if(!dut.run_request)$fatal(1,"zero cap changed RUN state");
 set_limit(900);check_drive(-900,45,1);
 frame('h48,2,901,0);if(dut.pwm_output_limit!=900 || last_limit!={16'd900,8'd1})$fatal(1,"invalid cap committed");
 before_acks=acks;frame('h48,2,400,1);if(dut.pwm_output_limit!=900 || acks!=before_acks)$fatal(1,"bad CRC cap committed");
 before_acks=acks;frame('h48,1,0,0);if(dut.pwm_output_limit!=900 || acks!=before_acks)$fatal(1,"bad length accepted");
 before_acks=acks;frame('h17,0,0,0);cycles(10000);if(acks<=before_acks || last_limit!={16'd900,8'd0})$fatal(1,"query cap reply");
 force dut.motor_enable=0;#1;if(dut.motor_stby || dut.motor_pwm || dut.applied_command!=0)$fatal(1,"STOP gate bypassed");
 $display("tb_pwm_limit_uart PASS");$finish;
end
initial begin #500000000;$fatal(1,"timeout");end
endmodule
