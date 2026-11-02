// 内部目标补偿：根据真实编码器误差调整用户目标之外的有限偏置。
// 运动过快、倾角饱和或运行条件不满足时暂停学习；目标切换保留已学偏置并重置统计窗口。
// 连续补偿采用误差与位移滤波、方向判断、分数余量和变化率约束，限制内部目标变化。
`default_nettype none

module target_trim_supervisor #(
 parameter integer SETTLE_MS=1000,WINDOW_MS=200,AVG_WINDOWS=5,ADAPTIVE_STEP=1,FAST_SUPPORTED=0,TUNING_SUPPORTED=0
)(
 input wire clk,rst_n,enable,ms_tick,reset_history,target_change,
 input wire fast_mode,response_change,
 input wire [1:0] correction_strength,
 input wire [15:0] correction_rate_q8,
 input wire signed [31:0] requested,actual,
 input wire [31:0] tolerance_q8,
 input wire [15:0] max_trim,travel_limit,counts_per_rev,
 output wire signed [31:0] internal_target,
 output // 补偿状态及统计量与用户目标独立保存，切换命令不能直接清除已学偏置。
reg signed [17:0] offset,
 output wire [7:0] status);

wire use_fast=FAST_SUPPORTED!=0 && fast_mode;
wire [15:0] active_settle=SETTLE_MS;
wire [15:0] active_window=WINDOW_MS;
function [63:0] response_scaled;input [63:0] value;begin
 response_scaled=value;
end endfunction
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
wire [63:0] average_threshold=response_scaled({32'd0,tolerance_q8}*TOTAL_SAMPLES);
wire [63:0] recent_threshold=response_scaled({32'd0,tolerance_q8}*RECENT_SAMPLES);
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
wire [63:0] fast_recent=recent_threshold+response_scaled(({48'd0,fast_step}*RECENT_SAMPLES*64'd512));
wire [63:0] fast_average=average_threshold+response_scaled(({48'd0,fast_step}*TOTAL_SAMPLES*64'd512));
wire [63:0] medium_recent=recent_threshold+response_scaled(({48'd0,medium_step}*RECENT_SAMPLES*64'd512));
wire [63:0] medium_average=average_threshold+response_scaled(({48'd0,medium_step}*TOTAL_SAMPLES*64'd512));
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

reg signed [8:0] fast_fraction;
wire signed [32:0] fast_delta=$signed({actual[31],actual})-$signed({window_start[31],window_start});
wire signed [17:0] fast_error_count=error>32767 ? 18'sd32767 : error< -32767 ? -18'sd32767 : error[17:0];
wire signed [17:0] fast_delta_count=fast_delta>32767 ? 18'sd32767 : fast_delta< -32767 ? -18'sd32767 : fast_delta[17:0];
wire signed [24:0] fast_raw_error={fast_error_count[16:0],8'd0};
wire signed [24:0] fast_raw_velocity={fast_delta_count[16:0],8'd0};
wire signed [24:0] fast_error=error_sum[24:0],fast_velocity=rolling_sum[24:0];
wire signed [25:0] fast_de=$signed({fast_raw_error[24],fast_raw_error})-$signed({fast_error[24],fast_error});
wire signed [25:0] fast_dv=$signed({fast_raw_velocity[24],fast_raw_velocity})-$signed({fast_velocity[24],fast_velocity});
wire signed [25:0] fast_einc=fast_de[25] ? -((-fast_de)>>>2) : fast_de>>>2;
wire signed [25:0] fast_vinc=fast_dv[25] ? -((-fast_dv)>>>2) : fast_dv>>>2;
wire signed [25:0] fast_en=$signed({fast_error[24],fast_error})+fast_einc;
wire signed [25:0] fast_vn=$signed({fast_velocity[24],fast_velocity})+fast_vinc;
wire [25:0] fast_abs_error=fast_en[25] ? -fast_en : fast_en;
wire [25:0] fast_abs_velocity=fast_vn[25] ? -fast_vn : fast_vn;
wire [24:0] fast_abs_raw_error=fast_raw_error[24] ? -fast_raw_error : fast_raw_error;

wire [31:0] fast_speed_limit=(counts_per_rev*32'd256)/600<64 ? 32'd64 : (counts_per_rev*32'd256)/600;
wire fast_motion_ok=fast_abs_velocity<=fast_speed_limit;
wire [31:0] fast_prediction=({6'd0,fast_abs_velocity}<<4)+({6'd0,fast_abs_velocity}<<2);
wire fast_brake=fast_vn!=0 && fast_vn[25]==fast_en[25] &&
    {6'd0,fast_abs_error}<={32'd0}+tolerance_q8+fast_prediction;
wire fast_outside=fast_abs_error>tolerance_q8 && fast_abs_raw_error>tolerance_q8 && fast_raw_error[24]==fast_en[25];
wire [31:0] fast_excess=fast_outside ? {6'd0,fast_abs_error}-tolerance_q8 : 32'd0;

wire [31:0] fast_amount=TUNING_SUPPORTED==0 ? fast_excess>>3 :
 correction_strength==0 ? fast_excess>>7 : correction_strength==1 ? fast_excess>>6 : fast_excess>>5;
wire [9:0] fast_rate_limit=TUNING_SUPPORTED==0 ? 10'd512 : correction_rate_q8>512 ? 10'd512 : correction_rate_q8[9:0];
wire [9:0] fast_capped=fast_amount>fast_rate_limit ? fast_rate_limit : fast_amount[9:0];
wire signed [10:0] fast_increment=fast_en[25] ? -$signed({1'b0,fast_capped}) : $signed({1'b0,fast_capped});
wire signed [11:0] fast_accum=$signed({{3{fast_fraction[8]}},fast_fraction})+$signed({fast_increment[10],fast_increment});
wire signed [3:0] fast_integer=fast_accum[11] ? -((-fast_accum)>>>8) : fast_accum>>>8;
wire signed [11:0] fast_remainder=fast_accum-($signed({{8{fast_integer[3]}},fast_integer})<<<8);
wire [3:0] fast_integer_abs=fast_integer[3] ? -fast_integer : fast_integer;
wire [3:0] fast_positive=positive_room<=0 ? 4'd0 : positive_room<$signed({29'd0,fast_integer_abs}) ? positive_room[3:0] : fast_integer_abs;
wire [3:0] fast_negative=negative_room<=0 ? 4'd0 : negative_room<$signed({29'd0,fast_integer_abs}) ? negative_room[3:0] : fast_integer_abs;
assign status=use_fast ? {2'd0,enable && settle_count<100,enable && settle_count<100,limited,eligible,correcting,enable} : {2'd0,enable && filled_windows<AVG_WINDOWS,enable && (target_settling || settle_count<active_settle),
 limited,eligible,correcting,enable};
integer i;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  offset<=0;fast_fraction<=0;settle_count<=0;window_count<=0;minimum<=0;maximum<=0;window_start<=0;
  eligible<=0;correcting<=0;limited<=0;error_sum<=0;rolling_sum<=0;history_index<=0;filled_windows<=0;
  target_settling<=0;
  for(i=0;i<AVG_WINDOWS;i=i+1)begin history[i]<=0;history_min[i]<=0;history_max[i]<=0;history_start[i]<=0;end
 end else if(!enable || reset_history)begin
  offset<=0;fast_fraction<=0;settle_count<=0;window_count<=0;minimum<=actual;maximum<=actual;window_start<=actual;
  eligible<=0;correcting<=0;limited<=0;error_sum<=0;rolling_sum<=0;history_index<=0;filled_windows<=0;
  target_settling<=0;
  for(i=0;i<AVG_WINDOWS;i=i+1)begin history[i]<=0;history_min[i]<=0;history_max[i]<=0;history_start[i]<=0;end
 end else if(target_change || (FAST_SUPPORTED!=0 && response_change))begin
  fast_fraction<=0;

  settle_count<=0;window_count<=0;minimum<=actual;maximum<=actual;window_start<=actual;
  eligible<=0;correcting<=0;limited<=0;error_sum<=0;rolling_sum<=0;history_index<=0;filled_windows<=0;
  target_settling<=1;
  for(i=0;i<AVG_WINDOWS;i=i+1)begin history[i]<=0;history_min[i]<=0;history_max[i]<=0;history_start[i]<=0;end
 end else if(ms_tick)begin
  if(use_fast)begin
   if(settle_count<100)settle_count<=settle_count+1'b1;
   if(window_count>=19)begin
    window_count<=0;window_start<=actual;
    error_sum<={{23{fast_en[25]}},fast_en};rolling_sum<={{26{fast_vn[25]}},fast_vn};
    eligible<=settle_count>=100 && fast_motion_ok;correcting<=0;limited<=0;
    if(settle_count>=100 && fast_motion_ok && fast_outside && !fast_brake)begin
     if(fast_fraction!=0 && fast_fraction[8]!=fast_en[25])fast_fraction<=0;
     else begin
      fast_fraction<=fast_remainder[8:0];
      if(fast_integer>0)begin
       if(fast_positive!=0)begin offset<=offset+$signed({14'd0,fast_positive});correcting<=1;end
       if(fast_positive<fast_integer_abs)begin limited<=1;fast_fraction<=0;end
      end else if(fast_integer<0)begin
       if(fast_negative!=0)begin offset<=offset-$signed({14'd0,fast_negative});correcting<=1;end
       if(fast_negative<fast_integer_abs)begin limited<=1;fast_fraction<=0;end
      end
     end
    end else fast_fraction<=0;
   end else window_count<=window_count+1'b1;
  end else begin
  if(settle_count<active_settle)begin
   settle_count<=settle_count+1'b1;window_count<=0;minimum<=actual;maximum<=actual;
   eligible<=0;correcting<=0;
  end else if(window_count==0)begin
   minimum<=actual;maximum<=actual;window_start<=actual;error_sum<={{16{error[32]}},error};window_count<=1;
  end else begin
   minimum<=next_min;maximum<=next_max;error_sum<=completed_sum;
   if(window_count+1>=active_window)begin
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
end
endmodule
`default_nettype wire

