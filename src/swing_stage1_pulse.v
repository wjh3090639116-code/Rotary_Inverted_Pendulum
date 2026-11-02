// 两阶段自动起摆：识别最低点附近的往返运动并输出交替方向的定时脉冲。
// 根据角度极性及电位器跨界位置确定有效运动弧；捕获直立状态后交还闭环控制。
`default_nettype none

module swing_stage1_pulse #(parameter integer CLK_HZ=50000000, MAX_STEP=128)(
 input wire clk,rst_n,enable,sample_strobe,adc_valid,
 input wire [11:0] adc_code,wrap_span,wrap_half_span,
 input wire [11:0] bottom_code,target_code,upper_code,bottom_window,rearm_distance,
 input wire signed [15:0] first_pwm,
 input wire [15:0] pulse_ms,
 input wire angle_reverse,
 input wire two_stage_enable,
 input wire signed [15:0] stage2_pwm,
 input wire [15:0] stage2_ms,
 input wire [11:0] upright_code,capture_window,
 output reg stage2_active,
 output wire signed [15:0] command,
 output reg done,
 output reg pulse_event,
 output reg [15:0] pulse_count);

reg seen,first_sent,armed,clean_tracking,departed_positive;
reg [11:0] previous_code;
reg signed [12:0] previous_bottom;
reg signed [15:0] pulse_command;
reg [31:0] remaining;
reg [1:0] target_samples;
reg [1:0] sample_pipe;
reg signed [12:0] raw_delta_registered,raw_bottom_registered,raw_capture_registered;
wire signed [12:0] raw_delta=$signed({1'b0,adc_code})-$signed({1'b0,previous_code});
reg signed [12:0] delta;
wire signed [12:0] delta_next=raw_delta_registered>$signed({1'b0,wrap_half_span}) ? raw_delta_registered-$signed({1'b0,wrap_span}) :
 raw_delta_registered<-$signed({1'b0,wrap_half_span}) ? raw_delta_registered+$signed({1'b0,wrap_span}) : raw_delta_registered;
wire signed [12:0] raw_bottom=$signed({1'b0,adc_code})-$signed({1'b0,bottom_code});
reg signed [12:0] distance;
wire signed [12:0] distance_next=raw_bottom_registered>$signed({1'b0,wrap_half_span}) ? raw_bottom_registered-$signed({1'b0,wrap_span}) :
 raw_bottom_registered<-$signed({1'b0,wrap_half_span}) ? raw_bottom_registered+$signed({1'b0,wrap_span}) : raw_bottom_registered;
wire [12:0] magnitude=distance[12] ? -distance : distance;
wire step_ok=delta<=MAX_STEP && delta>=-MAX_STEP;
wire crossed=(previous_bottom<=0 && distance>0 && delta>0) ||
             (previous_bottom>=0 && distance<0 && delta<0);
wire velocity_positive=angle_reverse ? delta<0 : delta>0;

wire signed [15:0] configured_pwm=stage2_active ? stage2_pwm : first_pwm;
wire signed [15:0] next_command=velocity_positive ? -configured_pwm : configured_pwm;
wire [15:0] configured_ms=stage2_active ? stage2_ms : pulse_ms;
wire signed [12:0] raw_capture=$signed({1'b0,adc_code})-$signed({1'b0,upright_code});
reg signed [12:0] capture_delta;
wire signed [12:0] capture_delta_next=raw_capture_registered>$signed({1'b0,wrap_half_span}) ? raw_capture_registered-$signed({1'b0,wrap_span}) :
 raw_capture_registered<-$signed({1'b0,wrap_half_span}) ? raw_capture_registered+$signed({1'b0,wrap_span}) : raw_capture_registered;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  sample_pipe<=0;raw_delta_registered<=0;raw_bottom_registered<=0;raw_capture_registered<=0;
  delta<=0;distance<=0;capture_delta<=0;
 end else begin
  sample_pipe<=enable ? {sample_pipe[0],sample_strobe} : 2'b0;
  raw_delta_registered<=raw_delta;raw_bottom_registered<=raw_bottom;raw_capture_registered<=raw_capture;
  delta<=delta_next;distance<=distance_next;capture_delta<=capture_delta_next;
 end
end
wire capture_region=capture_delta<=$signed({1'b0,capture_window}) &&
 capture_delta>=-$signed({1'b0,capture_window});
wire returning_from_departure=departed_positive ? (distance<0 && delta<0) : (distance>0 && delta>0);

wire [12:0] trigger_band={1'b0,bottom_window}+MAX_STEP;

function in_decreasing_arc;
 input [11:0] code,start_code,end_code;
 begin
  in_decreasing_arc=start_code>=end_code ?
    (code<=start_code && code>=end_code) :
    (code<=start_code || code>=end_code);
 end
endfunction
wire decreasing=in_decreasing_arc(target_code,bottom_code,upper_code);
wire toward_upper=decreasing ? delta<0 : delta>0;
wire holding_or_toward_upper=decreasing ? delta<=0 : delta>=0;
wire target_region=decreasing ? in_decreasing_arc(adc_code,target_code,upper_code) :
 in_decreasing_arc(adc_code,upper_code,target_code);
wire approaching_target=decreasing ? in_decreasing_arc(adc_code,bottom_code,target_code) :
 in_decreasing_arc(adc_code,target_code,bottom_code);
wire approaching_upright=decreasing ? in_decreasing_arc(adc_code,bottom_code,upright_code) :
 in_decreasing_arc(adc_code,upright_code,bottom_code);
assign command=enable && adc_valid && !done && remaining!=0 ? pulse_command : 16'sd0;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  seen<=0;first_sent<=0;armed<=0;clean_tracking<=0;departed_positive<=0;previous_code<=0;previous_bottom<=0;
  pulse_command<=0;remaining<=0;target_samples<=0;done<=0;stage2_active<=0;pulse_event<=0;pulse_count<=0;
 end else if(!enable)begin
  seen<=0;first_sent<=0;armed<=0;clean_tracking<=0;departed_positive<=0;remaining<=0;target_samples<=0;
  done<=0;stage2_active<=0;pulse_event<=0;pulse_count<=0;pulse_command<=0;
 end else begin
  pulse_event<=0;
  if(remaining!=0)remaining<=remaining-1'b1;
  if(!adc_valid)begin remaining<=0;seen<=0;armed<=0;clean_tracking<=0;target_samples<=0;end
  else if(sample_pipe[1] && !done)begin
   previous_code<=adc_code;previous_bottom<=distance;seen<=1;
   if(!seen)begin target_samples<=0;end
   else if(!step_ok)begin

    clean_tracking<=0;target_samples<=0;
   end else begin

    if(!stage2_active)begin
      if(!holding_or_toward_upper && !target_region)begin clean_tracking<=0;target_samples<=0;end
      if(first_sent && approaching_target && !target_region && magnitude>bottom_window && toward_upper)
        clean_tracking<=1;
      if(clean_tracking && target_region && holding_or_toward_upper)begin
        if(target_samples<3)target_samples<=target_samples+1'b1;
        if(target_samples>=2)begin
          remaining<=0;target_samples<=0;
          if(two_stage_enable)stage2_active<=1;
          else begin done<=1;armed<=0;end
        end
      end else target_samples<=0;
    end else begin
      if(!holding_or_toward_upper && !capture_region)clean_tracking<=0;
      if(approaching_upright && !capture_region && magnitude>bottom_window && toward_upper)clean_tracking<=1;
      if(clean_tracking && capture_region && holding_or_toward_upper)begin
        if(target_samples>=1)begin done<=1;remaining<=0;armed<=0;end
        else target_samples<=target_samples+1'b1;
      end else target_samples<=0;
    end
    if(!first_sent && magnitude<=bottom_window)begin
      first_sent<=1;armed<=0;pulse_command<=first_pwm;
      remaining<=pulse_ms*(CLK_HZ/1000);pulse_event<=1;pulse_count<=1;
    end else if(first_sent)begin

      if(magnitude>=rearm_distance && !armed)begin
        armed<=1;departed_positive<=!distance[12];
      end
      if(armed && returning_from_departure)begin
        if(magnitude<=trigger_band)begin

          if(remaining==0)begin
            armed<=0;
            if(next_command!=0 && next_command[15]!=pulse_command[15])begin
              pulse_command<=next_command;
              remaining<=configured_ms*(CLK_HZ/1000);
              pulse_event<=1;if(pulse_count!=16'hffff)pulse_count<=pulse_count+1'b1;
            end
          end
        end else begin

          armed<=magnitude>=rearm_distance;
          departed_positive<=!distance[12];
        end
      end
    end
   end
  end
 end
end
endmodule
`default_nettype wire
