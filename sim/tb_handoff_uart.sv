// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_handoff_uart;
reg clk=0,rst_n=0,rxd=1;always #500 clk=~clk;
wire cs,sclk,sdi,sdo,tx,a,b,p,stby;
reg [15:0] input_code=16'h62a0;reg [1:0] fault=0;
reg [11:0] mock_code=1578;reg mock_valid=1;reg mock_bad=0;
reg inject_valid=0;reg [3:0] inject_action=0;
ads8688_uart_top #(.CLK_HZ(1000000),.BAUD(100000),.ADC_SPI_HZ(125000),.ADC_BOOT_CYCLES(200),.ADC_WARMUP_CYCLES(1000)) dut(
 .clk(clk),.rst_n(rst_n),.run_btn_n(1'b1),.adc_cs_n(cs),.adc_sclk(sclk),.adc_sdi(sdi),.adc_sdo(sdo),
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
integer i,acks=0;reg [239:0] saved;reg [11:0] expected_zero;
always @(negedge clk)if(dut.send_ui)begin
 acks=acks+1;
 if(dut.u_tx.frame_type!==8'h36 || dut.u_tx.frame_length!==11)$fatal(1,"screen frame collision");
end

wire [7:0] wire_byte;wire wire_valid;
uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) mon(.clk(clk),.rst_n(rst_n),.rxd(tx),.byte_data(wire_byte),.byte_valid(wire_valid));
integer parsing=0,n=0,idx=0,handbacks=0;reg [7:0] typ,sum;reg [239:0] data;reg [7:0] last_handoff;
always @(posedge clk)if(wire_valid)case(parsing)
0:if(wire_byte=='ha5)parsing=1;
1:if(wire_byte=='h5a)parsing=2;else parsing=0;
2:begin typ=wire_byte;sum=wire_byte;parsing=3;end
3:begin n=wire_byte;sum=sum^wire_byte;idx=0;data=0;parsing=n==0 ? 5:4;end
4:begin data[idx*8+:8]=wire_byte;sum=sum^wire_byte;idx=idx+1;if(idx==n)parsing=5;end
5:begin if(sum!=wire_byte)$fatal(1,"CRC");if(typ=='h3c)begin handbacks=handbacks+1;last_handoff=data[7:0];if(n!=1)$fatal(1,"ACK length");end parsing=0;end
endcase
integer old_center,old_target;
initial begin
 force dut.adc_code=mock_code;force dut.adc_valid=mock_valid;force dut.adc_bad=mock_bad;force dut.adc_sample_pulse=1;
 force dut.ui_valid=inject_valid;force dut.ui_action=inject_action;
 repeat(5)@(negedge clk);rst_n=1;wait(!dut.profile_busy);repeat(1000)@(negedge clk);
 run_cmd(3);repeat(3000)@(negedge clk);if(last_handoff!=1 || dut.run_request)$fatal(1,"handoff must not start");
 run_cmd(2);wait(dut.state==2);repeat(100)@(negedge clk);if(!dut.remote_run)$fatal(1,"remote owner");
 action(6);old_center=dut.encoder_center;old_target=dut.arm_target_full;
 run_cmd(3);repeat(5000)@(negedge clk);
 if(last_handoff!=0 || dut.remote_run || !dut.run_request || dut.state!=2 || dut.encoder_center!=old_center || dut.arm_target_full!=old_target)$fatal(1,"handoff changed controller state or nominal target");
 repeat(1200000)@(negedge clk);
 if(!dut.run_request || dut.state!=2 || !stby)$fatal(1,"run stopped after former watchdog expired");
 run_cmd(0);repeat(2000)@(negedge clk);if(dut.run_request || stby)$fatal(1,"STOP lost priority");
 run_cmd(2);wait(dut.state==2);repeat(1200000)@(negedge clk);
 if(dut.run_request || stby)$fatal(1,"unexpected cable loss must still expire lease");

 run_cmd(0);run_cmd(2);wait(dut.state==2);
 @(negedge clk);force dut.run_uart_valid=1;force dut.run_uart_detach=1;force dut.run_uart_command=1;
 inject_valid=1;inject_action=2;@(negedge clk);inject_valid=0;
 release dut.run_uart_valid;release dut.run_uart_detach;release dut.run_uart_command;
 repeat(3000)@(negedge clk);
 if(dut.run_request || stby || !dut.remote_lockout || last_handoff!=1)$fatal(1,"local STOP versus handoff");
 if(handbacks<3)$fatal(1,"missing actual UART acknowledgements");
 $display("PASS actual UART handoff03/3C: does not start, preserves ACTIVE/origin/target, survives1.2s without PC lease, explicit/local STOP wins, unexpected loss watchdog preserved");$finish;
end
initial begin #4000000000;$fatal(1,"handoff timeout");end
endmodule

