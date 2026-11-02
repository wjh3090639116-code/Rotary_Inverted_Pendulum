// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_pendulum_recovery;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
reg mock_tick=0;reg signed [31:0] actual=0;
reg signed [12:0] pendulum_error=0;
ads8688_uart_top #(.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000),.ADC_BOOT_CYCLES(100),.ADC_WARMUP_CYCLES(100)) dut(
 .clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.step_down_btn_n(1'b1),.amplitude_up_btn_n(1'b1),.amplitude_down_btn_n(1'b1),
 .adc_sdo(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.touch_irq_n(1'b1),.touch_miso(1'b0));
task cycles(input integer n);repeat(n)@(negedge clk);endtask
task byte_tx(input [7:0] v);integer i;begin
 @(negedge clk);rxd=0;cycles(10);for(i=0;i<8;i=i+1)begin rxd=v[i];cycles(10);end rxd=1;cycles(10);
end endtask
task frame(input [7:0] typ,n,input [239:0] data);integer i;reg [7:0] sum;begin
 sum=typ^n;byte_tx('ha5);byte_tx('h5a);byte_tx(typ);byte_tx(n);
 for(i=0;i<n;i=i+1)begin byte_tx(data[i*8+:8]);sum=sum^data[i*8+:8];end byte_tx(sum);cycles(30);
end endtask
task ms(input integer n);integer i;begin for(i=0;i<n;i=i+1)begin mock_tick=1;cycles(1);mock_tick=0;cycles(1);end end endtask
task expect_bound(input integer bound,input bit active,input bit armed);begin
 cycles(3);
 if(dut.tilt_limit!=bound || dut.normal_recovery_active!=active || dut.normal_recovery_armed!=armed)
  $fatal(1,"recovery bound/state %d %d %d expected %d %d %d",dut.tilt_limit,dut.normal_recovery_active,dut.normal_recovery_armed,bound,active,armed);
end endtask
reg [11:0] held_zero;reg[79:0] held_board;reg[31:0] held_kp;reg[9:0] held_pwm;
initial begin
 force dut.adc_code=12'd1578;force dut.adc_valid=1;force dut.adc_bad=0;
 force dut.adc_sample_pulse=0;force dut.arm_position=actual;force dut.angle_error=pendulum_error;
 force dut.outer_velocity_q8=0;
 cycles(5);rst_n=1;wait(!dut.profile_busy);cycles(10000);
 frame('h40,16,{16'd47,16'd43,16'd28,16'd28,32'd1750,16'd173,16'd1040});cycles(6000);
 frame('h37,8,{16'd0,16'd173,32'd740});cycles(6000);
 held_zero=dut.zero_code;held_board=dut.board_config;
 force dut.run_request=1;force dut.state=3'd2;force dut.tick=mock_tick;force dut.outer_tick=0;
 cycles(10);expect_bound(40,1,0);
 if(dut.small_step_limit!=28 || !dut.board_flags[7] || dut.turn_tilt_kind!=0)$fatal(1,"capture did not preserve configured normal");
 ms(499);expect_bound(40,1,0);ms(3);expect_bound(28,0,1);

 pendulum_error=40;ms(4);expect_bound(28,0,1);pendulum_error=0;ms(1);
 pendulum_error=-39;ms(20);expect_bound(28,0,1);
 pendulum_error=-40;ms(4);expect_bound(28,0,1);ms(1);expect_bound(40,1,0);

 pendulum_error=0;actual=20;ms(600);expect_bound(40,1,0);
 actual=0;force dut.outer_velocity_q8=32'd2000;ms(600);expect_bound(40,1,0);
 force dut.outer_velocity_q8=0;pendulum_error=33;ms(600);expect_bound(40,1,0);
 pendulum_error=32;ms(499);expect_bound(40,1,0);ms(3);expect_bound(28,0,1);

 frame('h1b,6,{16'd28,32'sd173});cycles(6000);pendulum_error=50;ms(20);expect_bound(28,0,0);
 actual=173;pendulum_error=0;ms(502);expect_bound(28,0,1);
 pendulum_error=50;ms(5);expect_bound(40,1,0);

 frame('h1b,6,{16'd43,32'sd1213});cycles(6000);expect_bound(43,0,0);
 if(dut.turn_tilt_kind!=1)$fatal(1,"forward profile lost");
 frame('h1b,6,{16'd47,32'sd173});cycles(6000);expect_bound(47,0,0);
 if(dut.turn_tilt_kind!=2)$fatal(1,"reverse profile lost");
 frame('h1b,6,{16'd28,32'sd693});cycles(6000);expect_bound(28,0,0);
 if(dut.turn_tilt_kind!=3)$fatal(1,"half-turn profile lost");
 frame('h1b,6,{16'd28,32'sd0});cycles(6000);actual=0;pendulum_error=0;ms(502);expect_bound(28,0,1);
 frame('h42,1,1);cycles(6000);pendulum_error=50;ms(10);expect_bound(28,0,0);
 frame('h42,1,0);cycles(6000);

 force dut.pid_enable=0;cycles(5);expect_bound(28,0,0);
 release dut.pid_enable;cycles(5);expect_bound(40,1,0);
 if(dut.zero_code!=held_zero || dut.board_config!=held_board || dut.small_step_limit!=28)$fatal(1,"recovery changed calibration or profile");
 force dut.pid_enable=0;force dut.run_request=0;cycles(5);
 frame('h40,16,{16'd47,16'd43,16'd50,16'd50,32'd1750,16'd173,16'd1040});cycles(6000);
 force dut.run_request=1;release dut.pid_enable;cycles(5);expect_bound(50,1,0);
 $display("tb_pendulum_recovery PASS");$finish;
end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
