// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_settings_adapter;
reg clk=0,rst_n=0,enabled=1,up=0,down=0;always #5 clk=~clk;
reg [15:0] amplitude=173;
wire [31:0] degrees;wire valid;wire [15:0] counts,limit;
reciprocal_settings_adapter dut(.clk(clk),.rst_n(rst_n),.enabled(enabled),.up(up),.down(down),
 .amplitude_counts(amplitude),.small_limit(16'd22),.large_limit(16'd44),
 .amplitude_degrees(degrees),.key_valid(valid),.key_counts(counts),.key_limit(limit));
task cycle(input integer n);repeat(n)@(negedge clk);endtask
integer a,events=0,last_counts,last_limit;
always @(posedge clk)if(valid)begin events=events+1;last_counts=counts;last_limit=limit;end
task press(input bit positive);begin up=positive;down=!positive;cycle(1);up=0;down=0;cycle(45);end endtask
initial begin
 cycle(3);rst_n=1;cycle(45);

 for(a=1;a<=32767;a=a+1)begin
  amplitude=a;cycle(40);
  if(degrees!=(a*360+520)/1040)$fatal(1,"unit conversion changed %d -> %d",a,degrees);
 end
 amplitude=231;cycle(40);press(1);
 if(events!=1 || last_counts!=260 || last_limit!=44)$fatal(1,"80+10 threshold");
 amplitude=260;cycle(40);press(0);
 if(events!=2 || last_counts!=231 || last_limit!=22)$fatal(1,"90-10 threshold");
 up=1;down=1;cycle(1);up=0;down=0;cycle(45);if(events!=2)$fatal(1,"simultaneous keys");
 amplitude=1040;cycle(40);press(1);if(events!=2)$fatal(1,"upper bound");
 amplitude=29;cycle(40);press(0);if(events!=2)$fatal(1,"lower bound");
 amplitude=173;cycle(40);up=1;cycle(1);up=0;cycle(8);amplitude=260;cycle(45);
 if(events!=2 || degrees!=90)$fatal(1,"old key result overwrote newer host amplitude");
 $display("PASS settings adapter: all32767 count inputs exactly match former rounding; +/-10 keys, tilt band, limits, simultaneous cancellation, stale update rejected");$finish;
end
initial begin #30000000;$fatal(1,"adapter timeout");end
endmodule
