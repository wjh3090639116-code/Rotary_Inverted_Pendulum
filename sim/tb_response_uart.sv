// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_response_uart;
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
reg [71:0] last_trim;reg [127:0] last_board;reg [55:0] last_target;
reg [231:0] last_monitor;integer speed_acks=0;reg[15:0] last_speed;integer speed_before;
always @(posedge clk)if(rv)case(parse)
 0:if(rx=='ha5)parse=1;
 1:if(rx=='h5a)parse=2;else parse=0;
 2:begin kind=rx;crc=rx;parse=3;end
 3:begin len=rx;crc=crc^rx;idx=0;payload=0;parse=len==0 ? 5:4;end
 4:begin payload[8*idx+:8]=rx;crc=crc^rx;idx=idx+1;if(idx==len)parse=5;end
 5:begin
   if(crc!=rx)$fatal(1,"wire reply CRC");
   case(kind)
    'h45:begin if(len!=2)$fatal(1,"speed reply length");speed_acks=speed_acks+1;last_speed=payload[15:0];end
    'h38:begin if(len!=9)$fatal(1,"trim wire length");trim_acks=trim_acks+1;last_trim=payload[71:0];end
    'h41:begin if(len!=16)$fatal(1,"board wire length");board_acks=board_acks+1;last_board=payload[127:0];end
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
 if(dut.run_request || !dut.config_loaded || !dut.trim_loaded || dut.trim_config[48])$fatal(1,"independent screen boot config, no RUN");
 if(dut.trim_config[31:0]!=1479 || dut.trim_config[47:32]!=173 || dut.board_config!={16'd20,32'd2000,16'd173,16'd1040})$fatal(1,"boot new settings");
 if(dut.zero_code!=1578 || dut.stage2_pwm!=400 || dut.stage1_upper!=1578 || dut.travel_limit!=2000 || dut.outer_period_us!=25000)$fatal(1,"base configuration changed");
 if(trim_acks<1 || board_acks<1)$fatal(1,"boot replies missing");
 if(!dut.response_fast)$fatal(1,"boot speed must be fast");
 frame('h44,1,0,0);cycles(5000);if(dut.response_fast || last_speed!=0)$fatal(1,"standard config");
 frame('h44,1,1,0);cycles(5000);if(!dut.response_fast || last_speed!=16'h0100)$fatal(1,"fast config");
 speed_before=speed_acks;frame('h44,1,0,1);cycles(5000);if(speed_acks!=speed_before || !dut.response_fast)$fatal(1,"bad speed CRC");
 frame('h44,1,2,0);cycles(5000);if(!dut.response_fast || last_speed!=16'h0101)$fatal(1,"invalid speed accepted");
 frame('h37,8,{16'd1,16'd173,32'd1479},0);cycles(5000);

 frame('h40,14,{16'd44,16'd22,16'd44,32'd100,16'd260,16'd1040},0);cycles(12000);
 if(dut.small_step_limit!=22 || dut.large_step_limit!=44 || last_board[79:0]!={16'd44,32'd100,16'd260,16'd1040} || last_board[87:80]!=0)$fatal(1,"new board settings / wire ACK");
 held_config=dut.board_config;
 frame('h40,14,{16'd65,16'd22,16'd44,32'd100,16'd260,16'd1040},0);cycles(12000);
 if(dut.board_config!=held_config || last_board[87:80]!=1 || dut.large_step_limit!=44)$fatal(1,"invalid config partially committed");
 old_acks=board_acks;frame('h40,14,{16'd44,16'd22,16'd44,32'd200,16'd260,16'd1040},1);cycles(12000);
 if(board_acks!=old_acks || dut.board_config!=held_config)$fatal(1,"bad CRC committed");

 force dut.run_request=1;force dut.state=3'd2;force dut.tick=mock_tick;force dut.outer_tick=0;
 cycles(10);plant=1;ms(22000);
 if(dut.target_trim_offset> -95 || dut.target_trim_offset< -110)$fatal(1,"baseline trim not learning on actual feedback %d",dut.target_trim_offset);
 held=dut.target_trim_offset;
 frame('h44,1,0,0);cycles(5000);if(dut.response_fast || dut.target_trim_offset!=held || last_speed!=0)$fatal(1,"live slow switch lost offset");
 frame('h44,1,1,0);cycles(5000);if(!dut.response_fast || dut.target_trim_offset!=held)$fatal(1,"live fast switch lost offset");
 frame('h1b,6,{16'd22,32'sd173},0);cycles(10000);
 if(dut.arm_target_full!=173 || dut.internal_target!=173+held || dut.target_trim_offset!=held || dut.tilt_limit!=22)$fatal(1,"target first clock bias / joint tilt");
 if(last_target[31:0]!=173 || last_target[39:32]!=0 || last_target[55:40]!=22)$fatal(1,"joint target ACK");

 ms(100);nominal=dut.u_target_trim.settle_count;
 frame('h1b,6,{16'd22,32'sd173},0);
 if(dut.u_target_trim.settle_count!=nominal)$fatal(1,"retry cleared trim history");
 frame('h37,8,{16'd1,16'd173,32'd2000},0);cycles(10000);
 if(dut.target_trim_offset!=held || dut.trim_config[31:0]!=2000 || last_trim[71:64]!=0)$fatal(1,"live tolerance reset learned bias");
 frame('h37,8,{16'd1,16'd200,32'd2000},0);cycles(10000);
 if(dut.trim_config[47:32]!=173 || last_trim[71:64]!=2)$fatal(1,"live max trim accepted");
 frame('h1b,6,{16'd65,32'sd300},0);cycles(10000);
 if(dut.arm_target_full!=173 || dut.tilt_limit!=22 || last_target[39:32]!=3)$fatal(1,"invalid target tilt partially committed");
 frame('h1b,6,{16'd44,32'sd3000},0);cycles(10000);
 if(dut.arm_target_full!=173 || dut.tilt_limit!=22)$fatal(1,"travel rejection changed tilt");

 action(9);cycles(5000);
 if(dut.arm_target_full!=1213 || dut.tilt_limit!=44 || dut.target_trim_offset!=held)$fatal(1,"screen whole turn / user baseline");
 action(7);cycles(5000);action(6);cycles(5000);
 if(dut.arm_target_full!=173 || dut.tilt_limit!=22)$fatal(1,"screen small-step limit");

 frame('h42,1,1,0);cycles(20);
 if(!dut.free_mode || dut.arm_target_full!=173)$fatal(1,"reciprocal immediate step");
 ms(99);cycles(5);if(dut.arm_target_full!=173)$fatal(1,"first interval shortened");
 ms(1);cycles(10);if(dut.arm_target_full!=433 || dut.tilt_limit!=44 || dut.panel_center!=173 || dut.target_trim_offset!=held)$fatal(1,"first reciprocal endpoint / bias");
 ms(100);cycles(10);if(dut.arm_target_full!=-87 || dut.panel_center!=173)$fatal(1,"second endpoint");
 frame('h42,1,0,0);ms(200);if(dut.arm_target_full!=173)$fatal(1,"toggle off did not return center");

 frame('h40,14,{16'd45,16'd23,16'd45,32'd100,16'd260,16'd1040},0);cycles(10);
 if(dut.panel_center!=173 || dut.arm_target_full!=173 || dut.tilt_limit!=44)$fatal(1,"limits-only update recentered / changed current tilt");

 frame('h3a,1,1,0);cycles(80000);
 if(monitor_frames<1 || !last_monitor[175] || last_monitor[31:0]!==dut.arm_target_full || last_monitor[63:32]!==dut.internal_target || last_monitor[79:64]===16'hxxxx)$fatal(1,"dual monitor frame");
 release dut.run_request;release dut.state;action(2);cycles(10);
 if(dut.target_trim_offset!=0 || dut.free_mode)$fatal(1,"STOP did not clear learned state");
 $display("PASS response speed actual UART0x44/45 range/CRC/live switch retains bias; board control: trim and reciprocal connected, exact boot/defaults, real UART CRC/range/state replies, bias retained across UART/touch/turn/reciprocal, joint target+tilt atomicity, delayed centered reciprocal and hold-off, limits-only no recenter, real monitor, STOP");$finish;
end
initial begin #1000000000;$fatal(1,"port integration timeout");end
endmodule

