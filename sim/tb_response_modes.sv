// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps
module tb_response_modes;
reg clk=0,rst_n=0,tick=0,enable=0,reset=0,change=0,mode_change=0,mode=1;
always #5 clk=~clk;
reg signed[31:0] requested=0,actual=0;
wire signed[31:0] std_target,ref_target,fast_target;wire signed[17:0] std_offset,ref_offset,fast_offset;wire[7:0] std_status,ref_status,fast_status;
target_trim_supervisor #(.FAST_SUPPORTED(1)) standard(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(reset),.target_change(change),.fast_mode(1'b0),.response_change(1'b0),.requested(requested),.actual(actual),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd1000),.counts_per_rev(16'd1040),.internal_target(std_target),.offset(std_offset),.status(std_status));
trim_original_reference original(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(reset),.target_change(change),.requested(requested),.actual(actual),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd1000),.counts_per_rev(16'd1040),.internal_target(ref_target),.offset(ref_offset),.status(ref_status));
target_trim_supervisor #(.FAST_SUPPORTED(1)) fast(.clk(clk),.rst_n(rst_n),.enable(enable),.ms_tick(tick),.reset_history(reset),.target_change(change),.fast_mode(mode),.response_change(mode_change),.requested(requested),.actual(actual),.tolerance_q8(32'd1479),.max_trim(16'd173),.travel_limit(16'd1000),.counts_per_rev(16'd1040),.internal_target(fast_target),.offset(fast_offset),.status(fast_status));
task cycle;begin @(negedge clk);if(rst_n && ({std_target,std_offset,std_status}!=={ref_target,ref_offset,ref_status}))$fatal(1,"standard differs from baseline");end endtask
task ms;begin tick=1;cycle();tick=0;cycle();end endtask
integer i,first_std=-1,first_fast=-1,last_fast=0,last_std=0,held;
initial begin
 repeat(3)cycle();rst_n=1;enable=1;requested=100;
 for(i=0;i<5000;i=i+1)begin ms();
  if(first_std<0 && std_offset!=0)first_std=i;
  if(first_fast<0 && fast_offset!=0)first_fast=i;
  if(fast_offset-last_fast>6 || std_offset-last_std>6)$fatal(1,"step bound");
  last_fast=fast_offset;last_std=std_offset;
 end
 if(first_fast<100 || first_fast>150 || first_std<1950 || first_std>2050)$fatal(1,"response onset fast=%d std=%d",first_fast,first_std);
 if(fast_offset<=std_offset)$fatal(1,"fast did not correct more in equal time");
 held=fast_offset;mode=0;mode_change=1;cycle();mode_change=0;
 if(fast_offset!=held || fast_target!=requested+held)$fatal(1,"live switch jumped internal target");
 for(i=0;i<1800;i=i+1)begin ms();if(fast_offset!=held)$fatal(1,"mode statistics not restarted");end
 change=1;requested=-200;cycle();change=0;
 if(fast_offset!=held || fast_target!=requested+held)$fatal(1,"new target lost offset");
 for(i=0;i<200;i=i+1)begin actual=i*4;ms();end
 if(fast_offset!=held)$fatal(1,"learned transit");
 for(i=0;i<3000;i=i+1)begin actual=-200+(i%2 ? 3:-3);ms();end
 if(fast_offset!=held)$fatal(1,"integrated zero-mean ripple");
 enable=0;cycle();if(fast_offset!=0)$fatal(1,"STOP retained trim");
 $display("PASS standard cycle-equivalent baseline; first correction fast=%0dms std=%0dms, doubled rate, live mode and target handoff retains offset, transit freeze, no zero-mean integration, bounded steps/STOP",first_fast,first_std);$finish;
end
endmodule

`default_nettype none

module trim_original_reference #(
 parameter integer SETTLE_MS=1000,WINDOW_MS=200,AVG_WINDOWS=5,ADAPTIVE_STEP=1
)(
 input wire clk,rst_n,enable,ms_tick,reset_history,target_change,
 input wire signed [31:0] requested,actual,
 input wire [31:0] tolerance_q8,
 input wire [15:0] max_trim,travel_limit,counts_per_rev,
 output wire signed [31:0] internal_target,
 output reg signed [17:0] offset,
 output wire [7:0] status);
reg [15:0] settle_count,window_count;
reg [2:0] history_index,filled_windows;
reg signed [48:0] error_sum,history[0:AVG_WINDOWS-1];
reg signed [51:0] rolling_sum;
reg signed [31:0] minimum,maximum,window_start;
reg signed [31:0] history_min[0:AVG_WINDOWS-1],history_max[0:AVG_WINDOWS-1],history_start[0:AVG_WINDOWS-1];
reg target_settling;
reg eligible,correcting,limited;
wire signed [32:0] error=$signed({requested[31],requested})-$signed({actual[31],actual});
wire signed [48:0] completed_sum=error_sum+$signed({{16{error[32]}},error});
wire signed [52:0] new_total=$signed({rolling_sum[51],rolling_sum})+
 $signed({{4{completed_sum[48]}},completed_sum})-$signed({{4{history[history_index][48]}},history[history_index]});
wire [52:0] average_abs=new_total[52] ? -new_total : new_total;
wire [48:0] recent_abs=completed_sum[48] ? -completed_sum : completed_sum;
localparam [63:0] TOTAL_SAMPLES=WINDOW_MS*AVG_WINDOWS,RECENT_SAMPLES=WINDOW_MS;
wire [63:0] average_threshold={32'd0,tolerance_q8}*TOTAL_SAMPLES;
wire [63:0] recent_threshold={32'd0,tolerance_q8}*RECENT_SAMPLES;
wire average_outside={average_abs,8'd0}>average_threshold;
wire recent_outside={recent_abs,8'd0}>recent_threshold;
wire average_ready=filled_windows>=AVG_WINDOWS-1;

wire [15:0] ripple_band=counts_per_rev/18<2 ? 16'd2 : counts_per_rev/18;
wire [15:0] drift_band=counts_per_rev/36<2 ? 16'd2 : counts_per_rev/36;
wire signed [31:0] next_min=actual<minimum ? actual : minimum;
wire signed [31:0] next_max=actual>maximum ? actual : maximum;
wire signed [32:0] spread=$signed({next_max[31],next_max})-$signed({next_min[31],next_min});
wire signed [32:0] drift=$signed({actual[31],actual})-$signed({window_start[31],window_start});
wire [32:0] drift_abs=drift[32] ? -drift : drift;
wire motion_ok=spread<=$signed({17'd0,ripple_band}) && drift_abs<=drift_band;

reg signed [31:0] rolling_min,rolling_max;
integer j;
always @*begin
 rolling_min=next_min;rolling_max=next_max;
 for(j=0;j<AVG_WINDOWS;j=j+1)if(j!=history_index)begin
  if(history_min[j]<rolling_min)rolling_min=history_min[j];
  if(history_max[j]>rolling_max)rolling_max=history_max[j];
 end
end
wire [2:0] oldest_index=history_index==AVG_WINDOWS-1 ? 0 : history_index+1'b1;
wire signed [32:0] rolling_drift=$signed({actual[31],actual})-
 $signed({history_start[oldest_index][31],history_start[oldest_index]});
wire [32:0] rolling_drift_abs=rolling_drift[32] ? -rolling_drift : rolling_drift;
wire signed [32:0] rolling_spread=$signed({rolling_max[31],rolling_max})-$signed({rolling_min[31],rolling_min});
wire [15:0] settled_range=counts_per_rev/45<2 ? 16'd2 : counts_per_rev/45;
wire [15:0] settled_drift=counts_per_rev/180<2 ? 16'd2 : counts_per_rev/180;
wire transition_settled=average_ready && rolling_spread<=$signed({17'd0,settled_range}) && rolling_drift_abs<=settled_drift;

wire [15:0] fast_step=counts_per_rev/180<1 ? 16'd1 : (counts_per_rev+16'd90)/180;
wire [15:0] medium_step=counts_per_rev/360<1 ? 16'd1 : (counts_per_rev+16'd180)/360;
wire [63:0] fast_recent=recent_threshold+({48'd0,fast_step}*RECENT_SAMPLES*64'd512);
wire [63:0] fast_average=average_threshold+({48'd0,fast_step}*TOTAL_SAMPLES*64'd512);
wire [63:0] medium_recent=recent_threshold+({48'd0,medium_step}*RECENT_SAMPLES*64'd512);
wire [63:0] medium_average=average_threshold+({48'd0,medium_step}*TOTAL_SAMPLES*64'd512);
wire [15:0] adaptive_step=ADAPTIVE_STEP==0 ? 16'd1 :
 ({recent_abs,8'd0}>fast_recent && {average_abs,8'd0}>fast_average) ? fast_step :
 ({recent_abs,8'd0}>medium_recent && {average_abs,8'd0}>medium_average) ? medium_step : 16'd1;

wire moving_towards=drift!=0 && drift[32]==completed_sum[48];
wire [63:0] closing_threshold=recent_threshold+({31'd0,drift_abs}*RECENT_SAMPLES*64'd512);
wire brake=ADAPTIVE_STEP!=0 && moving_towards && {recent_abs,8'd0}<=closing_threshold;
wire signed [32:0] sum=$signed({requested[31],requested})+$signed({{15{offset[17]}},offset});
wire signed [32:0] bound=$signed({17'd0,travel_limit});
wire signed [32:0] positive_offset_room=$signed({17'd0,max_trim})-$signed({{15{offset[17]}},offset});
wire signed [32:0] negative_offset_room=$signed({17'd0,max_trim})+$signed({{15{offset[17]}},offset});
wire signed [32:0] positive_travel_room=bound-sum,negative_travel_room=bound+sum;
wire signed [32:0] positive_room=positive_offset_room<positive_travel_room ? positive_offset_room : positive_travel_room;
wire signed [32:0] negative_room=negative_offset_room<negative_travel_room ? negative_offset_room : negative_travel_room;
wire [15:0] positive_step=positive_room<=0 ? 16'd0 : positive_room<$signed({17'd0,adaptive_step}) ? positive_room[15:0] : adaptive_step;
wire [15:0] negative_step=negative_room<=0 ? 16'd0 : negative_room<$signed({17'd0,adaptive_step}) ? negative_room[15:0] : adaptive_step;
assign internal_target=!enable ? requested : sum>bound ? $signed({16'd0,travel_limit}) :
 sum< -bound ? -$signed({16'd0,travel_limit}) : sum[31:0];
assign status={2'd0,enable && filled_windows<AVG_WINDOWS,enable && (target_settling || settle_count<SETTLE_MS),
 limited,eligible,correcting,enable};
integer i;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  offset<=0;settle_count<=0;window_count<=0;minimum<=0;maximum<=0;window_start<=0;
  eligible<=0;correcting<=0;limited<=0;error_sum<=0;rolling_sum<=0;history_index<=0;filled_windows<=0;
  target_settling<=0;
  for(i=0;i<AVG_WINDOWS;i=i+1)begin history[i]<=0;history_min[i]<=0;history_max[i]<=0;history_start[i]<=0;end
 end else if(!enable || reset_history)begin
  offset<=0;settle_count<=0;window_count<=0;minimum<=actual;maximum<=actual;window_start<=actual;
  eligible<=0;correcting<=0;limited<=0;error_sum<=0;rolling_sum<=0;history_index<=0;filled_windows<=0;
  target_settling<=0;
  for(i=0;i<AVG_WINDOWS;i=i+1)begin history[i]<=0;history_min[i]<=0;history_max[i]<=0;history_start[i]<=0;end
 end else if(target_change)begin

  settle_count<=0;window_count<=0;minimum<=actual;maximum<=actual;window_start<=actual;
  eligible<=0;correcting<=0;limited<=0;error_sum<=0;rolling_sum<=0;history_index<=0;filled_windows<=0;
  target_settling<=1;
  for(i=0;i<AVG_WINDOWS;i=i+1)begin history[i]<=0;history_min[i]<=0;history_max[i]<=0;history_start[i]<=0;end
 end else if(ms_tick)begin
  if(settle_count<SETTLE_MS)begin
   settle_count<=settle_count+1'b1;window_count<=0;minimum<=actual;maximum<=actual;
   eligible<=0;correcting<=0;
  end else if(window_count==0)begin
   minimum<=actual;maximum<=actual;window_start<=actual;error_sum<={{16{error[32]}},error};window_count<=1;
  end else begin
   minimum<=next_min;maximum<=next_max;error_sum<=completed_sum;
   if(window_count+1>=WINDOW_MS)begin
    window_count<=0;rolling_sum<=new_total[51:0];history[history_index]<=completed_sum;
    history_min[history_index]<=next_min;history_max[history_index]<=next_max;history_start[history_index]<=window_start;
    history_index<=history_index==AVG_WINDOWS-1 ? 0 : history_index+1'b1;
    if(filled_windows<AVG_WINDOWS)filled_windows<=filled_windows+1'b1;
    if(target_settling && transition_settled)target_settling<=0;
    eligible<=average_ready && motion_ok && !target_settling;correcting<=0;limited<=0;

    if(!target_settling && average_ready && motion_ok && average_outside && recent_outside && new_total[52]==completed_sum[48] && !brake)begin
     if(!new_total[52])begin
      if(positive_step!=0)begin offset<=offset+$signed({2'd0,positive_step});correcting<=1;limited<=positive_step<adaptive_step;end
      else limited<=1;
     end else begin
      if(negative_step!=0)begin offset<=offset-$signed({2'd0,negative_step});correcting<=1;limited<=negative_step<adaptive_step;end
      else limited<=1;
     end
    end
   end else window_count<=window_count+1'b1;
  end
 end
end
endmodule
`default_nettype wire

