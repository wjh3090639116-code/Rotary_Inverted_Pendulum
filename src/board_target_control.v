// 本地目标管理：统一处理步进命令与围绕中心的对称往复目标。
// 输出经顶层公共目标检查通道提交，不重复实现驱动或闭环控制。
`default_nettype none

module board_target_control(
 input wire clk,rst_n,enable,ms_tick,
 input wire [3:0] step_pulse,
 input wire negative_direction,reciprocal_enable,
 input wire config_load,
 input wire [79:0] config_data,
 input wire signed [31:0] current_target,external_target_value,
 input wire external_target_accept,
 input wire [15:0] travel_limit,
 input wire [15:0] small_step_limit,large_step_limit,
 output wire request_valid,
 input wire request_ready,
 output reg signed [31:0] request_target,
 output reg [15:0] request_tilt,
 output reg reciprocal_active,range_blocked,
 output wire signed [31:0] reciprocal_center);
reg [15:0] counts_per_rev,amplitude,reciprocal_tilt;
reg [31:0] interval_ms,elapsed_ms;
reg signed [31:0] center;
assign reciprocal_center=center;
reg next_negative,request_is_step,pending_valid;
assign request_valid=pending_valid && enable && (request_is_step || reciprocal_enable);
reg signed [31:0] step_delta;
reg [15:0] step_tilt;
wire signed [32:0] upper=$signed({center[31],center})+$signed({17'd0,amplitude});
wire signed [32:0] lower=$signed({center[31],center})-$signed({17'd0,amplitude});
wire endpoints_ok=upper<=$signed({17'd0,travel_limit}) && lower>=-$signed({17'd0,travel_limit});
wire signed [32:0] candidate_target=$signed({current_target[31],current_target})+
 (negative_direction ? -$signed({step_delta[31],step_delta}) : $signed({step_delta[31],step_delta}));
wire step_ok=candidate_target<=$signed({17'd0,travel_limit}) && candidate_target>=-$signed({17'd0,travel_limit});

always @* begin
 step_delta=0;step_tilt=small_step_limit;
 if(step_pulse[0])step_delta=counts_per_rev/6+((counts_per_rev%6>3 || (counts_per_rev%6==3 && (counts_per_rev/6)%2==1)) ? 1:0);
 else if(step_pulse[1])step_delta=(counts_per_rev+1)/3;
 else if(step_pulse[2])begin step_delta=(counts_per_rev>>1)+((counts_per_rev[0] && counts_per_rev[1]) ? 1:0);step_tilt=large_step_limit;end
 else if(step_pulse[3])begin step_delta=counts_per_rev;step_tilt=large_step_limit;end
end
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  counts_per_rev<=1040;amplitude<=173;interval_ms<=2000;reciprocal_tilt<=20;
  center<=0;elapsed_ms<=0;next_negative<=0;reciprocal_active<=0;range_blocked<=0;
  pending_valid<=0;request_target<=0;request_tilt<=20;request_is_step<=0;
 end else begin
  if(config_load)begin
   counts_per_rev<=config_data[15:0];amplitude<=config_data[31:16];interval_ms<=config_data[63:32];reciprocal_tilt<=config_data[79:64];
   pending_valid<=0;

   if(config_data[63:0]!={interval_ms,amplitude,counts_per_rev})begin
    center<=current_target;elapsed_ms<=0;next_negative<=negative_direction;
    reciprocal_active<=0;range_blocked<=0;
   end
  end else if(!enable)begin
   pending_valid<=0;reciprocal_active<=0;range_blocked<=0;elapsed_ms<=0;center<=current_target;
  end else if(external_target_accept)begin
   pending_valid<=0;center<=external_target_value;elapsed_ms<=0;next_negative<=negative_direction;range_blocked<=0;reciprocal_active<=reciprocal_enable;
  end else begin
   if(!reciprocal_enable)begin
    reciprocal_active<=0;elapsed_ms<=0;range_blocked<=0;
    if(pending_valid && !request_is_step)pending_valid<=0;
   end else if(!reciprocal_active && !range_blocked)begin
    center<=current_target;elapsed_ms<=0;next_negative<=negative_direction;reciprocal_active<=1;
   end
   if(reciprocal_active && reciprocal_enable && !endpoints_ok)begin
    range_blocked<=1;reciprocal_active<=0;
    if(!request_is_step)pending_valid<=0;
   end
   if(pending_valid && request_ready && (request_is_step || reciprocal_enable))begin
    pending_valid<=0;elapsed_ms<=0;
    if(request_is_step)begin center<=request_target;next_negative<=negative_direction;range_blocked<=0;reciprocal_active<=reciprocal_enable;end
    else next_negative<=!next_negative;
   end
   if(!pending_valid && |step_pulse)begin
    if(step_ok && step_delta!=0)begin
     request_target<=candidate_target[31:0];request_tilt<=step_tilt;pending_valid<=1;request_is_step<=1;
    end else range_blocked<=1;
   end else if(!pending_valid && reciprocal_active && reciprocal_enable && endpoints_ok && ms_tick)begin
    if(elapsed_ms+1>=interval_ms)begin
     request_target<=next_negative ? lower[31:0] : upper[31:0];request_tilt<=reciprocal_tilt;
     pending_valid<=1;request_is_step<=0;
    end else elapsed_ms<=elapsed_ms+1'b1;
   end
  end
 end
end
endmodule
`default_nettype wire
