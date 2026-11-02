// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_board_targets;
reg clk=0,rst_n=0,enable=0,ms_tick=0,negative=0,reciprocal=0,load=0,external=0,ready=1;
always #500 clk=~clk;
reg [3:0] keys=0;reg [79:0] cfg;
reg signed [31:0] target=0,external_value=0;
wire valid,active,blocked;wire signed [31:0] requested;wire [15:0] tilt;
reg [15:0] last_tilt=0;integer moves=0;
board_target_control dut(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(ms_tick),.step_pulse(keys),
 .small_step_limit(16'd20),.large_step_limit(16'd40),.negative_direction(negative),.reciprocal_enable(reciprocal),.config_load(load),.config_data(cfg),
 .current_target(target),.external_target_value(external_value),.external_target_accept(external),.travel_limit(16'd3000),
 .request_valid(valid),.request_ready(ready),.request_target(requested),.request_tilt(tilt),.reciprocal_active(active),.range_blocked(blocked));
always @(posedge clk)begin
 if(external)target<=external_value;
 else if(valid && ready)begin target<=requested;last_tilt<=tilt;moves=moves+1;end
end
task cycle;begin @(negedge clk);end endtask
task configure;input integer counts,amp,period;begin
 cfg={16'd20,32'(period),16'(amp),16'(counts)};load=1;cycle();load=0;repeat(3)cycle();
end endtask
task set_target;input integer value;begin
 external_value=value;external=1;cycle();external=0;repeat(3)cycle();
end endtask
task step;input [3:0] key;input integer expected,limit;begin
 keys=key;cycle();keys=0;repeat(5)cycle();
 if(target!=expected || last_tilt!=limit)$fatal(1,"key target/tilt got %0d/%0d wanted %0d/%0d",target,last_tilt,expected,limit);
end endtask
task ms;input integer n;integer i;begin for(i=0;i<n;i=i+1)begin ms_tick=1;cycle();ms_tick=0;repeat(3)cycle();end end endtask
integer prior;reg signed [31:0] held;
initial begin
 cfg=0;repeat(3)cycle();rst_n=1;configure(1040,50,5);
 keys=1;cycle();keys=0;repeat(4)cycle();if(moves!=0)$fatal(1,"inactive key moved target");
 enable=1;step(1,173,20);step(2,520,20);step(4,1040,40);step(8,2080,40);
 negative=1;step(1,1907,20);
 configure(1041,50,5);set_target(0);negative=0;
 step(1,174,20);step(2,521,20);step(4,1041,40);step(8,2082,40);
 configure(1040,50,5);set_target(100);negative=0;reciprocal=1;repeat(3)cycle();
 prior=moves;ms(4);if(moves!=prior)$fatal(1,"reciprocal first interval shortened");
 ms(1);if(target!=150)$fatal(1,"first positive endpoint");
 ms(5);if(target!=50)$fatal(1,"negative endpoint or center drift");
 ms(5);if(target!=150)$fatal(1,"symmetric endpoint drift");
 reciprocal=0;held=target;ms(20);if(target!=held)$fatal(1,"OFF failed to hold target");
 reciprocal=1;negative=1;repeat(3)cycle();ms(5);if(target!=100)$fatal(1,"new center/first negative endpoint");
 set_target(400);ms(5);if(target!=350)$fatal(1,"external target did not recenter");
 ms(5);if(target!=450)$fatal(1,"external center opposite endpoint");

 ready=0;ms(5);if(!valid)$fatal(1,"pending request not held");
 reciprocal=0;cycle();ready=1;held=target;ms(10);if(target!=held)$fatal(1,"cancelled request executed");
 reciprocal=1;repeat(3)cycle();ready=0;ms(5);if(!valid)$fatal(1,"pending setup");
 configure(1040,80,7);reciprocal=0;ready=1;held=target;ms(10);
 if(target!=held)$fatal(1,"configuration change retained old timed target");
 set_target(2950);configure(1040,100,5);reciprocal=1;repeat(5)cycle();ms(10);
 if(!blocked || target!=2950)$fatal(1,"invalid symmetric range not blocked");
 negative=1;step(1,2777,20);ms(5);
 if(target!=2677 || blocked)$fatal(1,"manual recenter did not recover from range block");
 ready=0;ms(5);enable=0;cycle();ready=1;held=target;ms(10);
 if(valid || active || target!=held)$fatal(1,"stop did not cancel local movement");
 $display("PASS: 60/120/180/360 cumulative keys, direction, ties-even encoder rounding, 20/40 limits, symmetric fixed center/interval, off holds, external recenter, pending cancellation, travel block/recovery, inactive/stop guard");$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
