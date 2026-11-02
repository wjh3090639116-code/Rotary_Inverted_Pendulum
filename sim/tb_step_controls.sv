// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_step_controls;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire cs,sclk,sdi,sdo,tx,a,b,p,stby;
reg [15:0] input_code=16'h62a0;reg [1:0] fault=0;
reg [11:0] mock_code=1578;reg mock_valid=1;reg mock_bad=0;
reg plus_n=0,minus_n=0;
reg inject_valid=0;reg [3:0] inject_action=0;
ads8688_uart_top #(.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000),.ADC_BOOT_CYCLES(200),.ADC_WARMUP_CYCLES(1000)) dut(
 .clk(clk),.rst_n(rst_n),.run_btn_n(plus_n),.step_down_btn_n(minus_n),.adc_cs_n(cs),.adc_sclk(sclk),.adc_sdi(sdi),.adc_sdo(sdo),
 .encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.uart_txd(tx),.motor_in1(a),.motor_in2(b),.motor_pwm(p),.motor_stby(stby),
 .touch_irq_n(1'b1),.touch_miso(1'b0));
ads8688_input_model model(.cs_n(cs),.sclk(sclk),.sdi(sdi),.host_rst_n(rst_n),.fault(fault),.sdo(sdo),.input_code(input_code));
task action(input [3:0] v);begin
 @(negedge clk);inject_action=v;inject_valid=1;@(negedge clk);inject_valid=0;repeat(20)@(negedge clk);
end endtask
task byte_tx(input [7:0] v);integer n;begin
 @(negedge clk);rxd=0;repeat(10)@(negedge clk);for(n=0;n<8;n=n+1)begin rxd=v[n];repeat(10)@(negedge clk);end
 rxd=1;repeat(10)@(negedge clk);
end endtask
task run_cmd(input [7:0] v);begin byte_tx('ha5);byte_tx('h5a);byte_tx('h12);byte_tx(1);byte_tx(v);byte_tx('h13^v);repeat(10)@(negedge clk);end endtask
task press_key(input integer key);begin
 @(negedge clk);if(key==0)plus_n=0;else minus_n=0;
 repeat(21000)@(negedge clk);
 if(dut.run_request || stby)$fatal(1,"step key started motor");
 if(key==0)plus_n=1;else minus_n=1;repeat(21000)@(negedge clk);
end endtask
integer i,acks=0;reg [239:0] saved;reg [11:0] expected_zero;
always @(negedge clk)if(dut.send_ui)begin
 acks=acks+1;
 if(dut.u_tx.frame_type!==8'h36 || dut.u_tx.frame_length!==11)$fatal(1,"screen frame collision");
end
initial begin
 force dut.adc_code=mock_code;force dut.adc_valid=mock_valid;force dut.adc_bad=mock_bad;force dut.adc_sample_pulse=1;
 force dut.ui_valid=inject_valid;force dut.ui_action=inject_action;
 repeat(5)@(negedge clk);rst_n=1;
 wait(!dut.profile_busy);$display("BOOT done config=%b zero=%d link=%b error=%d",dut.config_loaded,dut.zero_code,dut.adc_link_valid,dut.adc_error);repeat(1000)@(negedge clk);
 if(!dut.config_loaded || dut.run_request || stby)$fatal(1,"boot config must remain stopped");
 if(dut.zero_code!==1578 || dut.swing_pump_pwm!==550 || dut.pulse_ms!==250 || dut.stage2_pwm!==400 || dut.stage2_ms!==300 || dut.outer_period_us!==25000 || dut.travel_limit!==2000 || dut.bottom_code!==201)$fatal(1,"wrong profile copied");
 if(dut.config_echo!==240'h0006a2a2000018fb0001a345000001a000000a0a07d0012d003d0101062a || dut.period_echo!==1000 || dut.wrap_echo!==32'h0a500000 || dut.tilt_echo!==32 || dut.pwm_deadzone!==0)$fatal(1,"full boot parameters differ from confirmed successful run");
 wait(dut.adc_valid);saved=dut.config_echo;
 if(dut.ui_step_degrees!==60)$fatal(1,"wrong initial step");

 repeat(25000)@(negedge clk);if(dut.ui_step_degrees!==60)$fatal(1,"boot-held key fired");
 plus_n=1;minus_n=1;repeat(21000)@(negedge clk);
 plus_n=0;repeat(1000)@(negedge clk);plus_n=1;repeat(21000)@(negedge clk);
 if(dut.ui_step_degrees!==60)$fatal(1,"bounce accepted");
 press_key(0);if(dut.ui_step_degrees!==70)$fatal(1,"BT0 +10");
 minus_n=0;repeat(80000)@(negedge clk);if(dut.ui_step_degrees!==60)$fatal(1,"held key repeated");
 minus_n=1;repeat(21000)@(negedge clk);

 for(i=0;i<7;i=i+1)press_key(1);
 if(dut.ui_step_degrees!==10)$fatal(1,"minimum clamp");
 for(i=0;i<38;i=i+1)press_key(0);
 if(dut.ui_step_degrees!==360)$fatal(1,"maximum clamp");
 for(i=0;i<35;i=i+1)press_key(1);
 if(dut.ui_step_degrees!==10)$fatal(1,"back to 10");
 plus_n=0;minus_n=0;repeat(21000)@(negedge clk);
 if(dut.ui_step_degrees!==10)$fatal(1,"simultaneous keys must cancel");
 plus_n=1;minus_n=1;repeat(21000)@(negedge clk);
 action(9);if(dut.arm_target_full!==0 || dut.run_request)$fatal(1,"stopped full-turn ran motor");
 wait(dut.adc_valid);action(3);wait(dut.state==2);repeat(20)@(negedge clk);
 if(!dut.manual_mode || dut.remote_run || !dut.remote_lockout || !stby)$fatal(1,"manual/local owner failed mode=%b remote=%b lock=%b stby=%b pid=%b",dut.manual_mode,dut.remote_run,dut.remote_lockout,stby,dut.pid_enable);
 for(i=0;i<36;i=i+1)action(6);
 if(dut.arm_target_full!==1040 || dut.ui_residual!==0)$fatal(1,"36 +10 taps not exact turn %d carry %d",dut.arm_target_full,dut.ui_residual);
 action(8);if(dut.arm_target_full!==0 || dut.ui_step_degrees!==10)$fatal(1,"-360 must subtract exactly and retain step");
 action(9);if(dut.arm_target_full!==1040)$fatal(1,"+360");
 action(9);if(dut.arm_target_full!==1040 || dut.ui_notice!==7)$fatal(1,"travel rejection must not clamp/move");
 action(7);for(i=0;i<36;i=i+1)action(5);
 if(dut.arm_target_full!==-1040 || dut.ui_residual!==0)$fatal(1,"36 -10 taps not exact turn");
 action(9);if(dut.arm_target_full!==0)$fatal(1,"reverse path full turn");

 action(6);if(dut.arm_target_full!==29 || dut.ui_residual!==-4)$fatal(1,"10 degree quantization");
 plus_n=0;repeat(21000)@(negedge clk);plus_n=1;repeat(21000)@(negedge clk);
 action(6);if(dut.arm_target_full!==87 || dut.ui_residual!==-12)$fatal(1,"mixed 10+20 degree carry");
 action(8);if(dut.arm_target_full!==-953 || dut.ui_residual!==-12)$fatal(1,"turn lost fractional carry");
 action(9);action(5);if(dut.arm_target_full!==29 || dut.ui_residual!==-4)$fatal(1,"mixed inverse step");
 action(7);if(dut.ui_residual!==0)$fatal(1,"center lost carry reset");

 action(6);@(negedge clk);inject_action=9;inject_valid=1;
 @(negedge clk);inject_action=2;@(negedge clk);inject_valid=0;
 repeat(2000)@(negedge clk);if(dut.arm_target_full!==0 || stby)$fatal(1,"STOP lost to target");
 action(2);repeat(2000)@(negedge clk);
 if(dut.run_request || stby || dut.ui_reply[87:56]!==0)$fatal(1,"local STOP");
 run_cmd(1);if(dut.run_request)$fatal(1,"old PC heartbeat restarted local STOP");
 run_cmd(0);run_cmd(2);wait(dut.state==2);
 if(!dut.remote_run)$fatal(1,"PC owner not restored after STOP");
 action(2);repeat(2000)@(negedge clk);
 mock_code=3000;repeat(10)@(negedge clk);wait(dut.adc_valid);
 expected_zero=dut.adc_code;action(4);
 if(dut.zero_code!==expected_zero || dut.zero_code<=2047 || dut.ui_reply[55:40]!=={4'b0,expected_zero})$fatal(1,"12bit zero/echo broken zero=%d reply=%d",dut.zero_code,dut.ui_reply[55:40]);
 wait(dut.adc_valid);action(1);wait(dut.state==4);
 if(dut.manual_mode || dut.remote_run)$fatal(1,"auto local start");
 action(2);repeat(2000)@(negedge clk);
 mock_valid=0;mock_bad=1;wait(!dut.adc_valid);action(1);
 if(dut.run_request || stby || dut.ui_notice!==9)$fatal(1,"no ADC start guard");
 if(acks<3)$fatal(1,"missing UI synchronization");
 $display("PASS physical BT0/BT1, 20ms debounce, boot-held, no-repeat, simultaneous cancellation, 10..360 clamps; mixed-step exact carry; +/-360; travel/STOP; original control guards");$finish;
end
initial begin #(64'd6000000000);$fatal(1,"step control timeout state=%d code=%d boot=%b link=%b err=%d config=%b",dut.state,dut.adc_code,dut.profile_busy,dut.adc_link_valid,dut.adc_error,dut.config_loaded);end
endmodule
