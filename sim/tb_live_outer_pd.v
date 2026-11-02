// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_live_outer_pd;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire scl,sda,tx,a,b,p,s;
reg [239:0] config_data;reg [63:0] gains;reg [95:0] inner_gains;
reg [11:0] code=1600;reg fresh=0;reg signed [31:0] absolute_encoder=500;
integer replies=0,updates=0,outer_calcs=0;reg [71:0] echo;reg [63:0] sampled_pair;
reg signed [23:0] previous_integral;reg signed [31:0] expected_origin=900;
ads8688_uart_top #(.ENABLE_SCREEN(0),.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000)) dut(.clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.adc_sdo(1'b0),.adc_cs_n(),.adc_sclk(),.adc_sdi(),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.uart_txd(tx),.motor_in1(a),.motor_in2(b),.motor_pwm(p),.motor_stby(s));
always @(negedge clk)if(dut.send_live_pd)begin
 replies=replies+1;echo=dut.packet_payload[71:0];
 if(dut.u_tx.frame_type!==8'h2f || dut.u_tx.frame_length!==9)$fatal(1,"outer echo type/length");
end
always @(posedge clk)if(rst_n && dut.live_pd_commit)begin
 if(!dut.outer_tick || !dut.pid_enable || dut.config_accept)$fatal(1,"outer commit not at safe period boundary");
 previous_integral=dut.u_pid.integral;updates=updates+1;#1;
 if(dut.u_pid.integral<previous_integral || dut.state!==2 || !dut.run_request || !dut.manual_mode || dut.encoder_center!==expected_origin || dut.arm_target_position!==60 || !s)$fatal(1,"outer update reset inner/state/origin/target/PWM");
end
always @(posedge clk)if(rst_n && dut.outer_speed_valid && dut.pid_enable)begin
 sampled_pair=dut.live_pd_actual;outer_calcs=outer_calcs+1;#1;
 if({{8{dut.u_outer_pd.sampled_kd[23]}},dut.u_outer_pd.sampled_kd,{8{dut.u_outer_pd.sampled_kp[23]}},dut.u_outer_pd.sampled_kp}!==sampled_pair)$fatal(1,"mixed outer coefficient pair");
end
task byte_tx;input [7:0] data;integer n;begin @(negedge clk);rxd=0;repeat(10)@(negedge clk);for(n=0;n<8;n=n+1)begin rxd=data[n];repeat(10)@(negedge clk);end rxd=1;repeat(10)@(negedge clk);end endtask
task frame;input [7:0] kind,len;input [239:0] payload;input bad;integer n;reg [7:0] sum;begin sum=kind^len;byte_tx(8'ha5);byte_tx(8'h5a);byte_tx(kind);byte_tx(len);for(n=0;n<len;n=n+1)begin byte_tx(payload[8*n+:8]);sum=sum^payload[8*n+:8];end byte_tx(sum^bad);repeat(15000)@(negedge clk);end endtask
task sample;input [11:0] v;begin @(negedge clk);code=v;fresh=1;@(negedge clk);fresh=0;repeat(998)@(negedge clk);end endtask
integer before_reply,before_updates;
initial begin
 repeat(5)@(negedge clk);rst_n=1;
 force dut.adc_valid=1;force dut.adc_code=code;force dut.adc_sample_pulse=fresh;force dut.encoder_position=absolute_encoder;
 frame(8'h17,0,0,0);if(replies!=1 || echo!==0)$fatal(1,"query capability/default");
 config_data={32'sd81920,32'sd9216,32'sd180224,32'sd656,32'sd3456,16'd2000,16'd188,16'd38,8'd1,8'd1,16'd730};
 frame(8'h32,30,config_data,0);frame(8'h16,2,240'd32,0);frame(8'h2a,2,240'd100,0);frame(8'h12,1,240'd2,0);
 gains={32'sd0,32'sd8192};frame(8'h2e,8,{176'd0,gains},0);
 if(echo[7:0]!==1 || dut.state!==1 || updates!=0 || dut.arm_kp_q16!==9216 || dut.motor_enable)$fatal(1,"outer change in WAIT");
 before_reply=replies;frame(8'h2e,8,{176'd0,gains},1);if(replies!=before_reply)$fatal(1,"outer bad checksum");
 absolute_encoder=900;sample(760);sample(755);sample(750);repeat(100)@(negedge clk);
 if(dut.state!==2 || dut.encoder_center!==900)$fatal(1,"manual handoff");
 force dut.angle=13'sd0;force dut.angle_error=13'sd1;force dut.outer_velocity_q8=32'sd0;
 frame(8'h1b,4,240'd60,0);absolute_encoder=910;repeat(50000)@(negedge clk);
 if(dut.arm_target_position!==60 || dut.arm_position!==10 || dut.u_pid.integral<=0)$fatal(1,"active target/integral precondition");
 frame(8'h2e,8,{176'd0,gains},0);
 if(updates!=1 || echo!=={gains,8'd0} || dut.live_pd_actual!==gains || dut.config_echo[239:176]!==gains || dut.target_tilt!==6 || dut.kp_q8!==3456 || dut.outer_period_us!==10000)$fatal(1,"outer live gain effect/echo/period/inner");
 gains={32'sd131072,32'sd16384};force dut.outer_velocity_q8=32'sd512;
 frame(8'h2e,8,{176'd0,gains},0);
 if(updates!=2 || dut.target_tilt!==9)$fatal(1,"outer D effect: .25*50 - 2*2 must round to 9");
 before_reply=replies;frame(8'h2e,7,{176'd0,gains},0);if(replies!=before_reply)$fatal(1,"outer invalid length");
 frame(8'h2e,8,{176'd0,32'sd0,32'sd1048577},0);if(echo[7:0]!==2 || updates!=2)$fatal(1,"outer P range");
 frame(8'h2e,8,{176'd0,32'sd4194305,32'sd0},0);if(echo[7:0]!==2 || updates!=2)$fatal(1,"outer D range");
 gains={-32'sd4194304,-32'sd1048576};frame(8'h2e,8,{176'd0,gains},0);if(echo!=={gains,8'd0} || updates!=3)$fatal(1,"negative outer limits");
 gains={32'sd4194304,32'sd1048576};frame(8'h2e,8,{176'd0,gains},0);if(echo!=={gains,8'd0} || updates!=4)$fatal(1,"positive outer limits");
 gains=0;frame(8'h2e,8,0,0);if(echo!==0 || updates!=5 || dut.target_tilt!==0 || !dut.pid_enable)$fatal(1,"zero outer pair must disable only outer action");

 force dut.tick=0;force dut.outer_tick=0;
 gains={32'sd131072,32'sd16384};inner_gains={32'sd16384,32'sd1024,32'sd4160};
 frame(8'h2e,8,{176'd0,gains},0);frame(8'h2c,12,{144'd0,inner_gains},0);
 if(!dut.live_pd_pending || !dut.live_pid_pending)$fatal(1,"concurrent pending precondition");
 @(negedge clk);force dut.tick=1;force dut.outer_tick=1;@(negedge clk);release dut.tick;release dut.outer_tick;
 repeat(20000)@(negedge clk);
 if(dut.live_pd_actual!==gains || dut.live_pid_actual!==inner_gains || dut.config_echo[239:176]!==gains || dut.config_echo[175:80]!==inner_gains || updates!=6)$fatal(1,"simultaneous inner/outer commit lost update");
 frame(8'h17,0,0,0);if(echo!=={gains,8'd0})$fatal(1,"outer actual query");
 frame(8'h12,1,240'd0,0);frame(8'h2e,8,{176'd0,gains},0);if(echo[7:0]!==1 || updates!=6 || dut.motor_enable)$fatal(1,"outer OFF update");
 expected_origin=absolute_encoder;
 frame(8'h1f,2,240'd50000,0);frame(8'h12,1,240'd2,0);sample(750);sample(750);sample(750);
 frame(8'h1b,4,240'd60,0);gains={32'sd65536,32'sd8192};frame(8'h2e,8,{176'd0,gains},0);repeat(50000)@(negedge clk);
 if(dut.live_pd_actual!==gains || updates!=7 || dut.outer_period_us!==50000 || dut.state!==2)$fatal(1,"50 ms outer boundary update");
 if(outer_calcs<10)$fatal(1,"outer calculation coverage");
 $display("PASS: real UART outer PD atomic period-boundary updates and numeric P/D effect; signed bounds/checksum/length/WAIT/OFF rejection, zero pair, simultaneous inner+outer gains, retained integral/origin/target/PWM, query and 50ms period; %d outer commits",updates);$finish;
end
initial begin #1200000000;$fatal(1,"outer PD timeout");end
endmodule
