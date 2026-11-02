// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_turn_tilt_uart;
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
initial begin
 force dut.adc_code=12'd1578;force dut.adc_valid=1;force dut.adc_bad=0;
 force dut.adc_sample_pulse=0;force dut.arm_position=actual;
 force dut.ui_valid=ui_valid;force dut.ui_action=ui_action;
 cycles(5);rst_n=1;wait(!dut.profile_busy);cycles(10000);

 frame('h40,16,{16'd55,16'd42,16'd18,16'd18,32'd2000,16'd173,16'd1040},0);cycles(10000);
 if(!dut.triplet_limits_loaded || dut.small_step_limit!=18 || dut.large_step_limit!=42 || dut.reverse_turn_limit!=55 || dut.tilt_limit!=18)$fatal(1,"three settings not applied");
 if(last_board[143:128]!=55 || last_board[127:112]!=42 || last_board[111:96]!=18)$fatal(1,"18-byte actual profiles ACK");
 frame('h37,8,{16'd0,16'd173,32'd740},0);cycles(5000);
 force dut.run_request=1;force dut.state=3'd2;force dut.tick=mock_tick;force dut.outer_tick=0;
 actual=0;cycles(10);
 frame('h1b,6,{16'd42,32'sd173},0);cycles(6000);
 if(dut.tilt_limit!=18 || dut.turn_tilt_kind!=0 || last_target[55:40]!=18)$fatal(1,"ordinary step used whole-turn profile");
 frame('h1b,6,{16'd42,32'sd1213},0);cycles(6000);ms(1);cycles(3);
 if(dut.turn_tilt_kind!=1 || dut.tilt_limit!=42)$fatal(1,"forward whole-turn profile");
 actual=1210;ms(1);cycles(3);if(dut.turn_tilt_kind!=1)$fatal(1,"outside tolerance completed");
 actual=1217;ms(1);cycles(3);if(dut.turn_tilt_kind!=1)$fatal(1,"crossing outside tolerance completed");
 actual=1213+131072;ms(1);cycles(3);if(dut.turn_tilt_kind!=1)$fatal(1,"truncated feedback falsely completed");
 actual=1211;force dut.outer_velocity_q8=32'sd3000;ms(7000);cycles(3);
 if(dut.motion_ramping || dut.turn_tilt_kind!=1 || dut.tilt_limit!=42)$fatal(1,"high-speed target crossing released braking limit");
 release dut.outer_velocity_q8;ms(102);cycles(3);if(dut.turn_tilt_kind!=0 || dut.tilt_limit!=18)$fatal(1,"measured tolerance did not restore normal");
 frame('h1b,6,{16'd55,32'sd173},0);cycles(6000);
 if(dut.turn_tilt_kind!=2 || dut.tilt_limit!=55)$fatal(1,"reverse whole-turn profile");
 frame('h1b,6,{16'd55,32'sd173},0);cycles(6000);if(dut.turn_tilt_kind!=2)$fatal(1,"retry disturbed active turn");
 actual=176;ms(1);cycles(3);if(dut.turn_tilt_kind!=2)$fatal(1,"reverse outside tolerance");
 actual=175;ms(7000);cycles(3);if(dut.turn_tilt_kind!=0 || dut.tilt_limit!=18)$fatal(1,"reverse restore");
 frame('h1b,6,{16'd55,32'sd173},0);cycles(6000);
 if(dut.turn_tilt_kind!=0 || dut.tilt_limit!=18 || last_target[55:40]!=18)$fatal(1,"completed retry rearmed turn");
 frame('h1b,6,{16'd42,32'sd700},0);cycles(6000);if(dut.tilt_limit!=18)$fatal(1,"ordinary large step used exception");
 frame('h1b,6,{16'd42,32'sd1740},0);cycles(6000);if(dut.turn_tilt_kind!=1)$fatal(1,"new turn");
 frame('h1b,6,{16'd18,32'sd1040},0);cycles(6000);if(dut.turn_tilt_kind!=0)$fatal(1,"ordinary replacement did not cancel turn");
 action(7);cycles(8000);if(dut.arm_target_full!=0 || dut.turn_tilt_kind!=0 || dut.tilt_limit!=18)$fatal(1,"touch center treated as reverse turn");
 action(9);cycles(8000);if(dut.arm_target_full!=1040 || dut.turn_tilt_kind!=1 || dut.tilt_limit!=42)$fatal(1,"touch forward turn");
 actual=1040;ms(7000);cycles(3);if(dut.tilt_limit!=18)$fatal(1,"touch forward restore");
 action(8);cycles(8000);if(dut.arm_target_full!=0 || dut.turn_tilt_kind!=2 || dut.tilt_limit!=55)$fatal(1,"touch reverse turn");
 actual=0;ms(7000);cycles(3);if(dut.tilt_limit!=18)$fatal(1,"touch reverse restore");

 frame('h40,16,{16'd55,16'd42,16'd18,16'd18,32'd100,16'd1040,16'd1040},0);cycles(6000);
 frame('h42,1,1,0);cycles(6000);ms(110);cycles(30);
 if(dut.arm_target_full!=1040 || dut.turn_tilt_kind!=0 || dut.tilt_limit!=18)$fatal(1,"reciprocal endpoint used turn exception");
 frame('h42,1,0,0);cycles(6000);

 frame('h40,16,{16'd0,16'd42,16'd18,16'd18,32'd100,16'd1040,16'd1040},0);cycles(6000);
 if(dut.reverse_turn_limit!=55 || last_board[87:80]!=1)$fatal(1,"invalid reverse limit partially committed");
 frame('h1b,6,{16'd42,32'sd1040},0);cycles(6000);
 frame('h1b,6,{16'd55,32'sd0},0);cycles(6000);if(dut.turn_tilt_kind!=2)$fatal(1,"last reverse");
 force dut.pid_enable=0;cycles(4);if(dut.turn_tilt_kind!=0 || dut.tilt_limit!=18)$fatal(1,"STOP/fault did not clear turn state");
 $display("tb_turn_tilt_uart PASS");$finish;
end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
