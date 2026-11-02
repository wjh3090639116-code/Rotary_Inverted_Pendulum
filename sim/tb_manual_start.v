// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_manual_start;
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
 frame(8'h32,30,config_data,0);frame(8'h2a,2,240'd100,0);
 frame(8'h12,1,240'd2,0);
 if(dut.state!==1 || !dut.run_request || !dut.manual_mode || dut.motor_enable || s || dut.applied_command!==0)$fatal(1,"manual start must wait without any PWM");
 sample(1600);sample(1600);sample(1600);
 if(dut.motor_enable || dut.pulse_count!==0 || dut.applied_command!==0)$fatal(1,"manual mode emitted swing pulses");
 absolute_encoder=700;for(v=1550;v>=800;v=v-50)sample(v);sample(770);
 if(dut.state!==1 || dut.motor_enable)$fatal(1,"manual captured outside window");
 absolute_encoder=900;sample(760);sample(755);sample(750);repeat(100)@(negedge clk);
 if(dut.state!==2 || !dut.pid_enable || dut.encoder_center!==900 || dut.arm_position!==0 || dut.arm_target_position!==0 || dut.arm_velocity_filtered_q8!==0)$fatal(1,"manual PID handoff/encoder origin");
 repeat(3000)@(negedge clk);if(dut.applied_command===0 || dut.pwm_deadzone!==100)$fatal(1,"manual PID final deadzone chain");
 frame(8'h12,1,240'd2,0);frame(8'h12,1,240'd1,0);
 if(dut.state!==2 || !dut.manual_mode || dut.encoder_center!==900)$fatal(1,"heartbeat changes running mode/origin");
 frame(8'h12,1,240'd0,0);if(dut.state!==0 || s || dut.applied_command!==0)$fatal(1,"manual stop safety");
 frame(8'h12,1,240'd2,0);if(dut.state!==1 || dut.motor_enable)$fatal(1,"restart manual must wait");
 force dut.adc_valid=0;repeat(3000)@(negedge clk);
 if(dut.state!==3 || dut.motor_enable || dut.fault_reason!==1)$fatal(1,"manual ADC fault protection");
 frame(8'h12,1,240'd0,0);force dut.adc_valid=1;code=2560;
 frame(8'h12,1,240'd1,0);sample(2560);sample(2560);
 if(dut.state!==4 || dut.manual_mode || dut.applied_command!==350)$fatal(1,"auto start after manual must still pulse with deadzone");
 frame(8'h12,1,240'd0,0);
 $display("PASS: manual UART mode waits with zero PWM, trusted upright capture enables PID with deadzone, encoder origin/outer target/velocity reset, heartbeats preserve mode, stop/ADC fault and subsequent auto mode");$finish;
end
initial begin #400000000;$fatal(1,"top timeout");end
endmodule
