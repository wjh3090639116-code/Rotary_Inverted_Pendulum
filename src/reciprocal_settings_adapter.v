// 按键与往复参数适配：将角度摆幅换算为编码器计数并提交参数更新。
// 角度显示与板端目标共用同一每转计数定义，避免两端单位不一致。
`default_nettype none

(* syn_dspstyle = "logic" *)
module reciprocal_settings_adapter #(parameter integer COUNTS_PER_REV=1040)(
 input wire clk,rst_n,enabled,up,down,
 input wire [15:0] amplitude_counts,small_limit,large_limit,
 output reg [31:0] amplitude_degrees,
 output reg key_valid,output reg [15:0] key_counts,key_limit
)                                       ;
reg [15:0] seen,key_baseline;
reg display_start,key_start,key_pending;
reg signed [31:0] display_numerator,key_numerator;
wire signed [31:0] display_result,key_result;
wire display_done,key_done,display_busy,key_busy;
reg [15:0] held_key_degrees;
wire display_current=seen==amplitude_counts && !display_busy && !display_start;
wire up_ok=up && !down && amplitude_degrees<360;
wire down_ok=down && !up && amplitude_degrees>10;
wire [31:0] next_degrees=up_ok ? (amplitude_degrees+10>360 ? 360:amplitude_degrees+10) :
 (amplitude_degrees<20 ? 10:amplitude_degrees-10);
tft_divide display_div(.clk(clk),.rst_n(rst_n),.start(display_start),
 .numerator(display_numerator),.denominator(COUNTS_PER_REV),.result(display_result),.done(display_done),.busy(display_busy));
tft_divide key_div(.clk(clk),.rst_n(rst_n),.start(key_start),
 .numerator(key_numerator),.denominator(32'd360),.result(key_result),.done(key_done),.busy(key_busy));
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  seen<=16'hffff;amplitude_degrees<=60;display_start<=0;display_numerator<=0;
  key_start<=0;key_numerator<=0;key_baseline<=0;held_key_degrees<=0;key_pending<=0;
  key_valid<=0;key_counts<=0;key_limit<=20;
 end else begin
  display_start<=0;key_start<=0;key_valid<=0;
  if(!display_busy && !display_start && seen!=amplitude_counts)begin
   seen<=amplitude_counts;display_numerator<={16'd0,amplitude_counts}*360+COUNTS_PER_REV/2;display_start<=1;
  end
  if(display_done && seen==amplitude_counts)amplitude_degrees<=display_result;
  if(enabled && display_current && !display_done && !key_busy && !key_start && !key_pending && (up_ok || down_ok))begin
   key_baseline<=amplitude_counts;held_key_degrees<=next_degrees[15:0];
   key_numerator<=next_degrees*COUNTS_PER_REV+180;key_start<=1;key_pending<=1;
  end
  if(key_done)begin
   key_pending<=0;
   if(enabled && key_pending && key_baseline==amplitude_counts && key_result>=1 && key_result<=32767)begin
    key_counts<=key_result[15:0];key_limit<=held_key_degrees<90 ? small_limit:large_limit;key_valid<=1;
   end
  end
 end
end
endmodule
`default_nettype wire
