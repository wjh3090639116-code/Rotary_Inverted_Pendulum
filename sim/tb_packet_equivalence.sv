// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_packet_equivalence;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,valid=0;reg[7:0] kind=0,len=0;reg[239:0] data=0;
 wire ready,refready,tx,reftx;
 uart_packet_tx #(.CLK_HZ(4000000),.BAUD(1000000),.MAX_PAYLOAD_BYTES(30)) dut
 (.clk(clk),.rst_n(rst_n),.frame_valid(valid),.frame_ready(ready),.frame_type(kind),.frame_length(len),.frame_payload(data),.uart_txd(tx));
 uart_packet_tx_reference #(.CLK_HZ(4000000),.BAUD(1000000),.MAX_PAYLOAD_BYTES(30)) reference_tx
 (.clk(clk),.rst_n(rst_n),.frame_valid(valid),.frame_ready(refready),.frame_type(kind),.frame_length(len),.frame_payload(data),.uart_txd(reftx));
 always @(negedge clk)if(rst_n)begin
  if(ready!==refready || tx!==reftx)$fatal(1,"UART wire/handshake mismatch");
 end
 integer n,j;
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  for(n=0;n<=30;n=n+1)begin
   wait(ready);@(negedge clk);kind=$random;len=n;
   for(j=0;j<30;j=j+1)data[j*8+:8]=$random;
   valid=1;@(negedge clk);valid=0;

   len=30;kind=~kind;data=~data;
   wait(!ready);wait(ready);repeat(50)@(negedge clk);
  end
  len=31;valid=1;repeat(20)@(negedge clk);
  if(ready)$fatal(1,"invalid payload accepted");
  valid=0;len=30;wait(ready);valid=1;@(negedge clk);valid=0;
  repeat(130)@(negedge clk);rst_n=0;repeat(3)@(negedge clk);rst_n=1;
  repeat(100)@(negedge clk);
  $display("tb_packet_equivalence PASS");$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
