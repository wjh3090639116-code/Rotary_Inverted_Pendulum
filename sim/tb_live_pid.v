// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_live_pid;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire scl,sda,tx,a,b,p,s;
reg [239:0] config_data;reg [95:0] gains;
reg [11:0] code=1600;reg fresh=0;reg signed [31:0] absolute_encoder=500;
integer replies=0,updates=0,ticks=0;reg [103:0] echo;reg signed [23:0] i_before,dummy;
reg [95:0] previous_gains;
ads8688_uart_top #(.ENABLE_SCREEN(0),.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000)) dut(.clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.adc_sdo(1'b0),.adc_cs_n(),.adc_sclk(),.adc_sdi(),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.uart_txd(tx),.motor_in1(a),.motor_in2(b),.motor_pwm(p),.motor_stby(s));
always @(negedge clk)if(dut.send_live_pid)begin
 replies=replies+1;echo=dut.packet_payload[103:0];
 if(dut.u_tx.frame_type!==8'h2d || dut.u_tx.frame_length!==13)$fatal(1,"live echo type/length");
end
always @(posedge clk)if(rst_n)begin
 if(dut.live_pid_commit)begin
  if(!dut.tick || !dut.pid_enable || dut.config_accept)$fatal(1,"unsafe/async gain commit");
  i_before=dut.u_pid.integral;updates=updates+1;
  #1;
  if(dut.u_pid.integral!==i_before || dut.state!==2 || !dut.run_request || !dut.manual_mode || dut.encoder_center!==900 || dut.arm_target_position!==0 || !s)$fatal(1,"live update reset integral/run/origin/target/PWM");
 end
end

always @(posedge clk)if(rst_n && dut.tick && dut.pid_enable_d)begin
 previous_gains=dut.live_pid_actual;ticks=ticks+1;#1;
 if({{8{dut.u_pid.sampled_kd[23]}},dut.u_pid.sampled_kd,{8{dut.u_pid.sampled_ki[23]}},dut.u_pid.sampled_ki,{8{dut.u_pid.sampled_kp[23]}},dut.u_pid.sampled_kp}!==previous_gains)$fatal(1,"mixed PID gain tuple");
end
task byte_tx;input [7:0] data;integer n;begin @(negedge clk);rxd=0;repeat(10)@(negedge clk);for(n=0;n<8;n=n+1)begin rxd=data[n];repeat(10)@(negedge clk);end rxd=1;repeat(10)@(negedge clk);end endtask
task frame;input [7:0] kind,len;input [239:0] payload;input bad;integer n;reg [7:0] sum;begin sum=kind^len;byte_tx(8'ha5);byte_tx(8'h5a);byte_tx(kind);byte_tx(len);for(n=0;n<len;n=n+1)begin byte_tx(payload[8*n+:8]);sum=sum^payload[8*n+:8];end byte_tx(sum^bad);repeat(15000)@(negedge clk);end endtask
task sample;input [11:0] v;begin @(negedge clk);code=v;fresh=1;@(negedge clk);fresh=0;repeat(998)@(negedge clk);end endtask
integer before_reply,v;reg signed [23:0] accumulated;
initial begin
 repeat(5)@(negedge clk);rst_n=1;
 force dut.adc_valid=1;force dut.adc_code=code;force dut.adc_sample_pulse=fresh;force dut.encoder_position=absolute_encoder;
 frame(8'h17,0,0,0);if(replies!=1 || echo!==0)$fatal(1,"query capability/default");
 config_data={32'sd81920,32'sd9216,32'sd180224,32'sd656,32'sd3456,16'd2000,16'd188,16'd38,8'd1,8'd1,16'd730};
 frame(8'h32,30,config_data,0);frame(8'h2a,2,240'd100,0);frame(8'h12,1,240'd2,0);
 gains={32'sd16384,32'sd1024,32'sd4160};
 frame(8'h2c,12,{144'd0,gains},0);
 if(echo[7:0]!==1 || dut.state!==1 || dut.kp_q8!==3456 || updates!=0 || dut.motor_enable)$fatal(1,"gain update in manual WAIT");
 before_reply=replies;frame(8'h2c,12,{144'd0,gains},1);if(replies!=before_reply || dut.kp_q8!==3456)$fatal(1,"bad checksum accepted");
 absolute_encoder=900;sample(760);sample(755);sample(750);repeat(100)@(negedge clk);
 if(dut.state!==2 || dut.encoder_center!==900)$fatal(1,"manual handoff");
 force dut.angle=13'sd0;force dut.target_tilt=13'sd1;repeat(50000)@(negedge clk);accumulated=dut.u_pid.integral;
 if(accumulated==0)$fatal(1,"integral precondition");
 frame(8'h2c,12,{144'd0,gains},0);
 if(updates!=1 || echo!=={gains,8'd0} || dut.live_pid_actual!==gains || dut.config_echo[175:80]!==gains || dut.u_pid.integral<accumulated || !dut.pid_enable || dut.applied_command==0)$fatal(1,"live tuple/echo/PID/retained integral");
 frame(8'h12,1,240'd2,0);if(dut.encoder_center!==900 || dut.state!==2)$fatal(1,"heartbeat during live tuning");
 before_reply=replies;frame(8'h2c,11,{144'd0,gains},0);if(replies!=before_reply || updates!=1)$fatal(1,"wrong length accepted");
 frame(8'h2c,12,{144'd0,32'sd16384,32'sd1024,32'sd131073},0);
 if(echo[7:0]!==2 || updates!=1 || dut.live_pid_actual!==gains)$fatal(1,"P range");
 frame(8'h2c,12,{144'd0,32'sd16384,32'sd1048577,32'sd4160},0);
 if(echo[7:0]!==2 || updates!=1)$fatal(1,"I range");
 frame(8'h2c,12,{144'd0,32'sd2097153,32'sd1024,32'sd4160},0);
 if(echo[7:0]!==2 || updates!=1)$fatal(1,"D range");
 frame(8'h2c,12,0,0);if(echo[7:0]!==2 || updates!=1)$fatal(1,"all zero live tuple accepted");
 gains={-32'sd2097152,-32'sd1048576,-32'sd131072};frame(8'h2c,12,{144'd0,gains},0);
 if(echo!=={gains,8'd0} || dut.live_pid_actual!==gains || updates!=2)$fatal(1,"negative gain limits");
 gains={32'sd2097152,32'sd1048576,32'sd131072};frame(8'h2c,12,{144'd0,gains},0);
 if(echo!=={gains,8'd0} || dut.live_pid_actual!==gains || updates!=3)$fatal(1,"positive gain limits");
 gains={32'sd16384,32'sd0,32'sd4160};frame(8'h2c,12,{144'd0,gains},0);
 if(updates!=4 || dut.ki_q12!==0 || dut.u_pid.integral!==0)$fatal(1,"PD Ki=0 existing integral behavior");
 frame(8'h17,0,0,0);if(echo!=={gains,8'd0})$fatal(1,"query actual gains");
 frame(8'h12,1,240'd0,0);frame(8'h2c,12,{144'd0,gains},0);
 if(echo[7:0]!==1 || updates!=4 || dut.motor_enable || dut.state!==0)$fatal(1,"OFF gain update");
 if(ticks<100)$fatal(1,"cadence coverage");
 $display("PASS: real UART live PID at 1kHz, atomic tuple, retained integral/state/origin/target/PWM, manual WAIT/OFF rejection, signed limits/checksum/length/all-zero rejection, PD and actual query; %d accepted updates / %d ticks",updates,ticks);$finish;
end
initial begin #500000000;$fatal(1,"live PID timeout");end
endmodule
