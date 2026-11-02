// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_trim_recorded_trace;
reg clk=0,rst_n=0,enable=0,tick=0;
always #5 clk=~clk;
reg signed [31:0] actual=0;
reg [31:0] trace[0:9999];
wire signed [31:0] old_target,new_target;
wire signed [17:0] old_offset,new_offset;
wire [7:0] old_status,new_status;
legacy_target_trim_reference old_dut(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(1'b0),
 .requested(32'sd0),.actual(actual),.tolerance_q8(32'd1479),.max_trim(16'd87),.travel_limit(16'd2000),
 .counts_per_rev(16'd1040),.internal_target(old_target),.offset(old_offset),.status(old_status));
target_trim_supervisor new_dut(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(1'b0),
 .target_change(1'b0),.requested(32'sd0),.actual(actual),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd2000),
 .counts_per_rev(16'd1040),.internal_target(new_target),.offset(new_offset),.status(new_status));
reg [2047:0] trace_file;
integer t,corrections;
initial begin
 if(!$value$plusargs("TRIM_TRACE=%s",trace_file))trace_file="../data/biased_return_trace.hex";
 $readmemh(trace_file,trace);
 repeat(3)@(negedge clk);rst_n=1;enable=1;corrections=0;
 for(t=0;t<10000;t=t+1)begin
  actual=$signed(trace[t]);tick=1;@(negedge clk);tick=0;@(negedge clk);
  if(new_status[1])corrections=corrections+1;
 end
 if(old_offset!=0)$fatal(1,"old supervisor did not reproduce zero internal target: %0d",old_offset);
 if(new_offset>=-20 || corrections==0)$fatal(1,"new supervisor did not respond to recorded bias: %0d",new_offset);
 if(new_offset< -173)$fatal(1,"new supervisor exceeded bound");
 $display("PASS: replay 10s of recorded target=0, actual around 37deg. old internal=%0d, corrected internal=%0d counts. Replay tests correction response only, not physical convergence",old_target,new_target);$finish;
end
endmodule

`default_nettype none

module legacy_target_trim_reference #(
 parameter integer SETTLE_MS=1000,WINDOW_MS=200
)(
 input wire clk,rst_n,enable,ms_tick,reset_history,
 input wire signed [31:0] requested,actual,
 input wire [31:0] tolerance_q8,
 input wire [15:0] max_trim,travel_limit,counts_per_rev,
 output wire signed [31:0] internal_target,
 output reg signed [17:0] offset,
 output wire [7:0] status);
reg [15:0] settle_count,window_count;
reg signed [31:0] minimum,maximum;
reg stable,correcting,limited;
wire signed [32:0] error=$signed({requested[31],requested})-$signed({actual[31],actual});
wire [32:0] error_abs=error[32] ? -error : error;
wire [40:0] error_q8={error_abs,8'd0};
wire [15:0] stable_band=counts_per_rev/720<2 ? 16'd2 : counts_per_rev/720;
wire signed [31:0] next_min=actual<minimum ? actual : minimum;
wire signed [31:0] next_max=actual>maximum ? actual : maximum;
wire signed [32:0] spread=$signed({next_max[31],next_max})-$signed({next_min[31],next_min});
wire signed [32:0] sum=$signed({requested[31],requested})+$signed({{15{offset[17]}},offset});
wire signed [32:0] bound=$signed({17'd0,travel_limit});
assign internal_target=!enable ? requested : sum>bound ? $signed({16'd0,travel_limit}) :
 sum< -bound ? -$signed({16'd0,travel_limit}) : sum[31:0];
assign status={4'd0,limited,stable,correcting,enable};
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin offset<=0;settle_count<=0;window_count<=0;minimum<=0;maximum<=0;stable<=0;correcting<=0;limited<=0;end
 else if(!enable || reset_history)begin
  offset<=0;settle_count<=0;window_count<=0;minimum<=actual;maximum<=actual;stable<=0;correcting<=0;limited<=0;
 end else if(ms_tick)begin
  correcting<=0;
  if(settle_count<SETTLE_MS)begin
   settle_count<=settle_count+1'b1;window_count<=0;minimum<=actual;maximum<=actual;stable<=0;
  end else if(error_abs>max_trim)begin
   window_count<=0;minimum<=actual;maximum<=actual;stable<=0;
  end else if(window_count==0)begin
   minimum<=actual;maximum<=actual;window_count<=1;stable<=0;
  end else begin
   minimum<=next_min;maximum<=next_max;
   if(window_count+1>=WINDOW_MS)begin
    window_count<=0;stable<=spread<=$signed({17'd0,stable_band});
    if(spread<=$signed({17'd0,stable_band}) && error_q8>tolerance_q8)begin
     if(error>0)begin
      if(offset<$signed({2'd0,max_trim}) && sum<bound)begin offset<=offset+1'b1;correcting<=1;limited<=0;end
      else limited<=1;
     end else begin
      if(offset>-$signed({2'd0,max_trim}) && sum> -bound)begin offset<=offset-1'b1;correcting<=1;limited<=0;end
      else limited<=1;
     end
    end else if(error_q8<=tolerance_q8)limited<=0;
   end else window_count<=window_count+1'b1;
  end
 end
end
endmodule
`default_nettype wire
