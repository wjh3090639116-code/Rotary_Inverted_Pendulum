// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_center_return_uart;
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
 frame('h40,16,{16'd33,16'd44,16'd22,16'd22,32'd100,16'd260,16'd1040},0);cycles(6000);
 frame('h37,8,{16'd1,16'd173,32'd1479},0);cycles(5000);
 force dut.run_request=1;force dut.state=3'd2;force dut.tick=mock_tick;force dut.outer_tick=0;
 actual=173;cycles(10);frame('h1b,6,{16'd22,32'sd173},0);cycles(6000);
 force dut.u_target_trim.offset=-18'sd20;
 frame('h42,1,1,0);cycles(6000);ms(110);cycles(30);
 if(dut.panel_center!=173 || dut.arm_target_full!=433)$fatal(1,"nonzero reciprocal center / first endpoint");
 ms(100);cycles(30);if(dut.arm_target_full!=-87)$fatal(1,"second endpoint");
 frame('h42,1,0,0);cycles(6000);
 if(dut.free_mode || dut.arm_target_full!=173 || dut.panel_center!=173 || dut.tilt_limit!=22)$fatal(1,"GUI disable did not restore original user center");
 if(dut.internal_target!=153)$fatal(1,"return center lost existing compensation");
 frame('h1b,6,{16'd22,32'sd700},0);cycles(6000);
 frame('h42,1,0,0);cycles(6000);if(dut.arm_target_full!=700)$fatal(1,"duplicate disable replayed old center");
 frame('h42,1,1,0);cycles(6000);ms(110);cycles(30);
 if(dut.panel_center!=700 || dut.arm_target_full!=960)$fatal(1,"restart did not capture new center");
 action(0);ms(1);cycles(30);
 if(dut.free_mode || dut.arm_target_full!=700)$fatal(1,"touch disable return");
 frame('h42,1,1,0);cycles(6000);ms(110);cycles(30);

 force dut.reciprocal_rx_valid=1;force dut.reciprocal_rx_enable=0;
 force dut.arm_target_uart_valid=1;force dut.arm_target_uart=32'sd500;
 force dut.arm_target_uart_has_tilt=1;force dut.arm_target_uart_tilt=16'd22;
 cycles(1);release dut.reciprocal_rx_valid;release dut.reciprocal_rx_enable;
 release dut.arm_target_uart_valid;release dut.arm_target_uart;release dut.arm_target_uart_has_tilt;release dut.arm_target_uart_tilt;
 cycles(50);if(dut.arm_target_full!=500 || dut.center_return_pending)$fatal(1,"stale center overrode manual target");
 frame('h42,1,1,0);cycles(6000);ms(110);cycles(30);
 force dut.pid_enable=0;force dut.reciprocal_rx_valid=1;force dut.reciprocal_rx_enable=0;cycles(1);
 release dut.reciprocal_rx_valid;release dut.reciprocal_rx_enable;cycles(50);
 if(dut.center_return_pending)$fatal(1,"STOP/fault queued motion");
 $display("tb_center_return_uart PASS");$finish;
end
initial begin #1000000000;$fatal(1,"timeout");end
endmodule
