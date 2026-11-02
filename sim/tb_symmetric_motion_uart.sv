// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_symmetric_motion_uart;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire tx;reg mock_tick=0,ui_valid=0;reg [3:0] ui_action=0;
reg signed [31:0] actual=0;
ads8688_uart_top #(.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000),.ADC_BOOT_CYCLES(100),.ADC_WARMUP_CYCLES(100)) dut(
 .clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.step_down_btn_n(1'b1),.amplitude_up_btn_n(1'b1),.amplitude_down_btn_n(1'b1),
 .adc_sdo(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),.uart_rxd(rxd),.uart_txd(tx),.touch_irq_n(1'b1),.touch_miso(1'b0));
wire [7:0] rx;wire rv;
uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) mon(.clk(clk),.rst_n(rst_n),.rxd(tx),.byte_data(rx),.byte_valid(rv));
integer parse=0,len=0,idx=0,trim_acks=0,board_acks=0,target_acks=0,monitor_frames=0;
reg [7:0] kind,crc;reg [239:0] payload;
reg [71:0] last_trim;reg [143:0] last_board;reg [55:0] last_target;
reg [231:0] last_monitor;integer speed_acks=0;reg[15:0] last_speed;integer speed_before,tuning_acks=0,tuning_before;reg[31:0] last_tuning;
always @(posedge clk)if(rv)case(parse)
 0:if(rx=='ha5)parse=1;
 1:if(rx=='h5a)parse=2;else parse=0;
 2:begin kind=rx;crc=rx;parse=3;end
 3:begin len=rx;crc=crc^rx;idx=0;payload=0;parse=len==0 ? 5:4;end
 4:begin payload[8*idx+:8]=rx;crc=crc^rx;idx=idx+1;if(idx==len)parse=5;end
 5:begin
   if(crc!=rx)$fatal(1,"wire reply CRC");
   case(kind)
    'h47:begin if(len!=4)$fatal(1,"tuning length");tuning_acks=tuning_acks+1;last_tuning=payload[31:0];end
    'h45:begin if(len!=2)$fatal(1,"speed reply length");speed_acks=speed_acks+1;last_speed=payload[15:0];end
    'h38:begin if(len!=9)$fatal(1,"trim wire length");trim_acks=trim_acks+1;last_trim=payload[71:0];end
    'h41:begin if(len!=16 && len!=18)$fatal(1,"board wire length");board_acks=board_acks+1;last_board=payload[143:0];end
    'h0d:begin if(len==7)begin target_acks=target_acks+1;last_target=payload[55:0];end end
    'h39:begin if(len!=29)$fatal(1,"monitor wire length");monitor_frames=monitor_frames+1;last_monitor=payload[231:0];end
   endcase
   parse=0;
 end
endcase
task cycles(input integer n);repeat(n)@(negedge clk);endtask
task byte_tx(input [7:0] v);integer i;begin
 @(negedge clk);rxd=0;cycles(10);for(i=0;i<8;i=i+1)begin rxd=v[i];cycles(10);end rxd=1;cycles(10);
end endtask
task frame(input [7:0] typ,n,input [239:0] data,input bad);integer i;reg [7:0] sum;begin
 sum=typ^n;byte_tx('ha5);byte_tx('h5a);byte_tx(typ);byte_tx(n);
 for(i=0;i<n;i=i+1)begin byte_tx(data[i*8+:8]);sum=sum^data[i*8+:8];end byte_tx(sum^bad);cycles(20);
end endtask
task action(input [3:0] v);begin ui_action=v;ui_valid=1;cycles(1);ui_valid=0;cycles(5);end endtask
task ms(input integer n);integer i;begin for(i=0;i<n;i=i+1)begin mock_tick=1;cycles(1);mock_tick=0;cycles(1);end end endtask
reg plant=0;
always @(negedge clk)if(plant)actual=dut.internal_target+105;
integer held,old_acks,nominal;reg [79:0] held_config;
task follow_to_normal(input integer expected_kind,input integer expected_tilt);integer i;begin
 for(i=0;i<6000 && dut.turn_tilt_kind!=0;i=i+1)begin actual=dut.internal_target;ms(1);end
 cycles(4);if(dut.turn_tilt_kind!=0 || dut.tilt_limit!=18 || dut.motion_ramping)$fatal(1,"soft motion did not restore normal %d",i);
end endtask
initial begin
 force dut.adc_code=12'd1578;force dut.adc_valid=1;force dut.adc_bad=0;
 force dut.adc_sample_pulse=0;force dut.arm_position=actual;
 force dut.ui_valid=ui_valid;force dut.ui_action=ui_action;
 cycles(5);rst_n=1;wait(!dut.profile_busy);cycles(10000);
 frame('h40,16,{16'd55,16'd42,16'd18,16'd18,32'd100,16'd173,16'd1040},0);cycles(6000);
 frame('h37,8,{16'd0,16'd173,32'd740},0);cycles(6000);
 force dut.run_request=1;force dut.state=3'd2;force dut.tick=mock_tick;force dut.outer_tick=0;
 actual=0;cycles(10);
 frame('h1b,6,{16'd18,32'sd520},0);cycles(6000);
 if(dut.arm_target_full!=520 || dut.turn_tilt_kind!=3 || dut.half_turn_negative || !dut.motion_ramping || dut.tilt_limit!=18 || dut.internal_target!=0)$fatal(1,"forward180 profile/start");
 follow_to_normal(3,18);
 frame('h1b,6,{16'd18,32'sd0},0);cycles(6000);
 if(dut.turn_tilt_kind!=3 || !dut.half_turn_negative || !dut.motion_ramping || dut.tilt_limit!=18 || dut.internal_target!=520)$fatal(1,"reverse180 profile/start");
 if(last_board[95:93]!=7 || !last_board[88])$fatal(1,"negative half-turn actual state echo");
 follow_to_normal(3,18);
 frame('h1b,6,{16'd55,-32'sd1040},0);cycles(6000);
 if(dut.turn_tilt_kind!=2 || !dut.motion_ramping || dut.tilt_limit!=55 || dut.internal_target!=0)$fatal(1,"reverse360 profile/start");
 actual=-1040;force dut.outer_velocity_q8=-32'sd3000;ms(7000);cycles(3);
 if(dut.motion_ramping || dut.turn_tilt_kind!=2 || dut.tilt_limit!=55)$fatal(1,"reverse360 high-speed crossing released brake");
 release dut.outer_velocity_q8;ms(102);cycles(3);if(dut.turn_tilt_kind!=0 || dut.tilt_limit!=18)$fatal(1,"reverse360 low-speed completion");
 frame('h1b,6,{16'd42,32'sd0},0);cycles(6000);
 if(dut.turn_tilt_kind!=1 || !dut.motion_ramping || dut.tilt_limit!=42)$fatal(1,"verified forward360 no longer activates");
 frame('h1b,6,{16'd18,32'sd173},0);cycles(6000);
 if(dut.motion_ramping || dut.turn_tilt_kind!=0 || dut.internal_target!=173)$fatal(1,"ordinary target changed");

 frame('h40,16,{16'd55,16'd42,16'd18,16'd18,32'd100,16'd520,16'd1040},0);cycles(6000);
 frame('h42,1,1,0);cycles(6000);ms(110);cycles(30);
 if(dut.arm_target_full!=693 || dut.turn_tilt_kind!=0 || dut.motion_ramping)$fatal(1,"reciprocal180 activated soft manual motion");
 frame('h42,1,0,0);cycles(6000);if(dut.arm_target_full!=173)$fatal(1,"reciprocal return center lost");

 force dut.ui_step_units=6'd18;cycles(10);action(6);cycles(6000);
 if(dut.arm_target_full!=693 || dut.turn_tilt_kind!=3 || dut.half_turn_negative || dut.tilt_limit!=18)$fatal(1,"touch forward180");
 action(5);cycles(6000);if(dut.arm_target_full!=173 || dut.turn_tilt_kind!=3 || !dut.half_turn_negative || dut.tilt_limit!=18)$fatal(1,"touch reverse180");
 force dut.pid_enable=0;cycles(5);if(dut.motion_ramping || dut.turn_tilt_kind!=0)$fatal(1,"STOP/fault did not cancel");
 $display("tb_symmetric_motion_uart PASS");$finish;
end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
