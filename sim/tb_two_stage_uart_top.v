// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_two_stage_uart_top;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire scl,sda,tx,a,b,p,s;
reg [239:0] config_data;reg [159:0] pulse_data;
reg [11:0] code=1600;reg fresh=0;reg signed [31:0] absolute_encoder=500;
integer echoes=0;reg [159:0] echo;
ads8688_uart_top #(.ENABLE_SCREEN(0),.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000)) dut(.clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.adc_sdo(1'b0),.adc_cs_n(),.adc_sclk(),.adc_sdi(),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.uart_txd(tx),.motor_in1(a),.motor_in2(b),.motor_pwm(p),.motor_stby(s));
always @(negedge clk)if(dut.send_swing)begin
 echoes=echoes+1;echo=dut.packet_payload[159:0];
 if(dut.u_tx.frame_type!==8'h29 || dut.u_tx.frame_length!==20)$fatal(1,"stage1 echo schema");
end
task byte_tx;input [7:0] data;integer n;begin @(negedge clk);rxd=0;repeat(10)@(negedge clk);for(n=0;n<8;n=n+1)begin rxd=data[n];repeat(10)@(negedge clk);end rxd=1;repeat(10)@(negedge clk);end endtask
task frame;input [7:0] kind,len;input [239:0] payload;input bad;integer n;reg [7:0] sum;begin sum=kind^len;byte_tx(8'ha5);byte_tx(8'h5a);byte_tx(kind);byte_tx(len);for(n=0;n<len;n=n+1)begin byte_tx(payload[8*n+:8]);sum=sum^payload[8*n+:8];end byte_tx(sum^bad);repeat(15000)@(negedge clk);end endtask
task sample;input [11:0] v;begin @(negedge clk);code=v;fresh=1;@(negedge clk);fresh=0;repeat(998)@(negedge clk);end endtask
integer before_echo,n,v;
initial begin
 repeat(5)@(negedge clk);rst_n=1;
 force dut.adc_valid=1;force dut.adc_code=code;force dut.adc_sample_pulse=fresh;
 force dut.encoder_position=absolute_encoder;
 config_data={32'sd81920,32'sd9216,32'sd180224,32'sd656,32'sd3456,16'd2000,16'd188,16'd38,8'd1,8'd1,16'd730};
 frame(8'h32,30,config_data,0);if(!dut.config_loaded)$fatal(1,"config not loaded");
 pulse_data={16'd30,16'd200,16'd2600,16'd100,16'd20,16'd730,16'd1130,16'd1600,16'd220,16'd600};
 frame(8'h28,20,{80'd0,pulse_data},0);
 if(echoes!=1 || echo!==pulse_data || !dut.stage2_configured || dut.stage2_pwm!=200 || dut.stage2_ms!=30)$fatal(1,"two-stage config/echo");

 pulse_data[143:128]=-400;frame(8'h28,20,{80'd0,pulse_data},0);
 if($signed(dut.stage2_pwm)!==-400 || echo!==pulse_data)$fatal(1,"negative stage2 config/echo");
 before_echo=echoes;pulse_data[143:128]=-901;frame(8'h28,20,{80'd0,pulse_data},0);
 if(echoes!=before_echo || $signed(dut.stage2_pwm)!==-400)$fatal(1,"negative PWM overflow accepted");
 pulse_data[143:128]=-900;frame(8'h28,20,{80'd0,pulse_data},0);
 if($signed(dut.stage2_pwm)!==-900)$fatal(1,"negative bound");
 pulse_data[143:128]=900;frame(8'h28,20,{80'd0,pulse_data},0);
 if(dut.stage2_pwm!==900)$fatal(1,"positive bound");
 pulse_data[143:128]=200;frame(8'h28,20,{80'd0,pulse_data},0);
 before_echo=echoes;pulse_data[143:128]=901;frame(8'h28,20,{80'd0,pulse_data},0);
 if(echoes!=before_echo)$fatal(1,"stage2 PWM overflow accepted");pulse_data[143:128]=200;
 pulse_data[159:144]=1001;frame(8'h28,20,{80'd0,pulse_data},0);
 if(echoes!=before_echo)$fatal(1,"stage2 duration overflow accepted");pulse_data[159:144]=30;
 frame(8'h28,20,{80'd0,pulse_data},1);if(echoes!=before_echo)$fatal(1,"bad checksum accepted");
 frame(8'h12,1,240'd1,0);if(dut.state!==4 || dut.encoder_center!==500)$fatal(1,"stage1/start origin");
 sample(1600);sample(1600);if(dut.drive_command!==600 || !s)$fatal(1,"first PWM");
 absolute_encoder=700;
 for(v=1550;v>=1150;v=v-50)sample(v);sample(1130);sample(1129);sample(1128);repeat(2000)@(negedge clk);
 if(dut.state!==4 || !dut.run_request || !dut.stage2_active || !dut.flags[7] || !dut.flags[6] || dut.encoder_center!==500 || dut.pid_enable)$fatal(1,"stage2 incorrectly stopped/recentered/PID");
 frame(8'h28,20,{80'd0,pulse_data},0);if(echoes!=before_echo)$fatal(1,"running change accepted");
 for(v=1150;v<=1550;v=v+50)sample(v);sample(1580);sample(1605);
 if(dut.applied_command!==-200)$fatal(1,"stage2 negative pulse");
 sample(1640);sample(1673);sample(0);sample(80);sample(120);wait(dut.u_stage1.remaining==0);sample(80);sample(40);sample(0);sample(1673);sample(1640);sample(1605);sample(1595);
 if(dut.applied_command!==200)$fatal(1,"stage2 positive pulse");
 for(v=1550;v>=800;v=v-50)sample(v);
 absolute_encoder=900;sample(770);sample(760);sample(755);repeat(100)@(negedge clk);
 if(dut.state!==2 || !dut.pid_enable || !dut.run_request || dut.encoder_center!==900 || dut.arm_position!==0 || dut.previous_position!==0 || dut.arm_target_position!==0 || dut.arm_velocity_filtered_q8!==0)$fatal(1,"PID capture origin/outer target/speed reset");
 if(dut.travel_upper!==2900 || dut.travel_lower!==-1100 || dut.pulse_command!==0 || dut.flags[7] || dut.flags[6])$fatal(1,"capture bounds/pulse/flags");
 repeat(3000)@(negedge clk);if(dut.applied_command===0)$fatal(1,"PID did not compute");
 absolute_encoder=901;repeat(4000)@(negedge clk);
 if(dut.arm_position!==1 || dut.encoder_center!==900)$fatal(1,"encoder coordinate after capture");
 frame(8'h12,1,240'd1,0);if(dut.encoder_center!==900 || dut.state!==2)$fatal(1,"keepalive resets capture origin");
 frame(8'h17,0,0,0);if(echo!==pulse_data || echoes<=before_echo)$fatal(1,"query new echo");
 sample(1000);repeat(3000)@(negedge clk);if(dut.state!==3 || dut.motor_enable || dut.fault_reason!==6)$fatal(1,"fall protection after capture");
 frame(8'h12,1,240'd0,0);if(dut.state!==0 || dut.motor_enable)$fatal(1,"remote stop");
 $display("PASS: real UART two-stage config/echo/query/range/running rejection, stage2 +/-200 pulses, PID handoff at upright, captured encoder origin=900 and zero target/velocity, keepalive, fall protection and stop");$finish;
end
initial begin #400000000;$fatal(1,"top timeout");end
endmodule
