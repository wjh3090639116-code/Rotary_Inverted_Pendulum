// 触摸坐标与操作译码：将触摸坐标映射至按钮区域，并保持单次触摸只触发一个操作。
// 使用顺序除法完成坐标换算，结合最近区域与间隔容差选择有效按钮。
`default_nettype none

module tft_divide(input wire clk,rst_n,start,input wire signed [31:0] numerator,denominator,
 output reg signed [31:0] result,output reg done,output wire busy);
 reg [31:0] quotient,divisor;reg [32:0] remainder;reg [5:0] count;reg negative;
 wire [32:0] trial={remainder[31:0],quotient[31]};
 assign busy=count!=0;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin quotient<=0;divisor<=1;remainder<=0;count<=0;negative<=0;result<=0;done<=0;end
 else begin
  done<=0;
  if(start && !busy)begin
   quotient<=numerator[31] ? -numerator : numerator;
   divisor<=denominator[31] ? -denominator : denominator;
   remainder<=0;negative<=numerator[31]^denominator[31];count<=32;
  end else if(busy)begin
   if(trial>={1'b0,divisor})begin remainder<=trial-{1'b0,divisor};quotient<={quotient[30:0],1'b1};end
   else begin remainder<=trial;quotient<={quotient[30:0],1'b0};end
   count<=count-1'b1;
   if(count==1)begin
    result<=negative ? -$signed({quotient[30:0],trial>={1'b0,divisor}}) : $signed({quotient[30:0],trial>={1'b0,divisor}});
    done<=1;
   end
  end
 end
endmodule

(* syn_dspstyle = "logic" *)
module tft_ui #(parameter integer CLK_HZ=50000000,
 parameter integer XFAC=663,XOFFSET=-13,YFAC=-914,YOFFSET=342,
 parameter integer SWAP_XY=0)(
 input wire clk,rst_n,raw_valid,down,input wire [11:0] raw_x,raw_y,
 input wire [3:0] notice,input wire notice_valid,
 input wire [7:0] status,input wire boot_busy,
 output reg action_valid,output reg [3:0] action,
 output reg [1:0] cal_step,output wire calibrated,output reg [3:0] pressed,
 output reg [3:0] message)                                       ;

 assign calibrated=1'b1;
 reg [31:0] release_count,message_count;reg armed,latched;
 reg [3:0] map_state;reg div_start;
 reg signed [31:0] numerator,denominator;wire signed [31:0] div_result;wire div_done,div_busy;
 reg signed [31:0] pixel_x,pixel_y;reg [11:0] sample_x,sample_y;
 reg [3:0] candidate;reg [3:0] held_candidate;reg [2:0] hits;
 tft_divide u_map(.clk(clk),.rst_n(rst_n),.start(div_start),.numerator(numerator),.denominator(denominator),.result(div_result),.done(div_done),.busy(div_busy));
 `include "src/tft_layout.vh"

 function [12:0] distance_to_button;
  input signed [11:0] px,py;input [3:0] id;
  reg signed [11:0] left_edge,right_edge,top_edge,bottom_edge;
  reg [12:0] dx,dy;
  begin
   left_edge=$signed({3'b0,screen_left(id)});right_edge=$signed({3'b0,screen_right(id)});
   top_edge=$signed({4'b0,screen_top(id)});bottom_edge=$signed({4'b0,screen_bottom(id)});
   dx=px<left_edge ? left_edge-px : px>right_edge ? px-right_edge : 0;
   dy=py<top_edge ? top_edge-py : py>bottom_edge ? py-bottom_edge : 0;
   distance_to_button=dx+dy;
  end
 endfunction

 localparam integer YABS=YFAC<0 ? -YFAC : YFAC;
 reg [23:0] y_product_low,y_product_high,y_product;
 always @(posedge clk)begin
  y_product_low<=(SWAP_XY ? sample_x : sample_y)*(YABS & 255);
  y_product_high<=(SWAP_XY ? sample_x : sample_y)*(YABS >> 8);
  y_product<=y_product_low+(y_product_high<<8);
 end
 reg [3:0] scan_index,scan_winner;
 reg [12:0] scan_distance,scan_best;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  action_valid<=0;action<=0;cal_step<=3;pressed<=0;message<=0;message_count<=0;
  release_count<=0;armed<=0;latched<=0;
  map_state<=0;div_start<=0;numerator<=0;denominator<=1;pixel_x<=0;pixel_y<=0;sample_x<=0;sample_y<=0;
  hits<=0;held_candidate<=0;candidate<=0;scan_index<=0;scan_winner<=0;scan_distance<=0;scan_best<=5;
 end else begin
  action_valid<=0;div_start<=0;
  if(!down)pressed<=0;
  if(message_count!=0)message_count<=message_count-1'b1;else message<=0;
  if(!down)begin
   hits<=0;held_candidate<=0;
   if(release_count<CLK_HZ/25)release_count<=release_count+1'b1;
   else begin armed<=1;latched<=0;end
  end else release_count<=0;
  if(raw_valid && down && !boot_busy && map_state==0)begin
   sample_x<=raw_x;sample_y<=raw_y;
   numerator<=$signed({1'b0,SWAP_XY ? raw_y : raw_x})*XFAC;
   denominator<=10000;div_start<=1;map_state<=1;
  end
  if(notice_valid)begin message<=(action==0 && notice==6) ? 4'd0 : notice;message_count<=CLK_HZ*4;end
  case(map_state)
   1:if(div_done)begin
    pixel_x<=div_result+XOFFSET;
    numerator<=YFAC<0 ? -$signed({8'b0,y_product}) : $signed({8'b0,y_product});
    denominator<=10000;div_start<=1;map_state<=2;
   end
   2:if(div_done)begin pixel_y<=32'sd239-pixel_x;pixel_x<=div_result+YOFFSET;map_state<=3;end
   3:begin
    if(pixel_x>=143 && pixel_x<=204 && pixel_y>=164 && pixel_y<=188)map_state<=8;
    else begin scan_index<=1;scan_winner<=0;scan_best<=5;map_state<=4;end
   end
   8:begin
    map_state<=0;
    if(down && armed && !latched && !boot_busy)begin
     if(held_candidate!=0 || hits==0)begin held_candidate<=0;hits<=1;end
     else begin latched<=1;armed<=0;pressed<=0;action<=0;action_valid<=1;end
    end
   end
   4:begin scan_distance<=distance_to_button(pixel_x[11:0],pixel_y[11:0],scan_index);map_state<=5;end
   5:begin
    if(scan_distance<scan_best || (scan_distance==scan_best && scan_distance<=4 && held_candidate==scan_index))begin
     scan_best<=scan_distance;scan_winner<=scan_index;
    end
    if(scan_index==9)map_state<=7;else begin scan_index<=scan_index+1'b1;map_state<=4;end
   end
   7:begin candidate<=scan_winner;map_state<=6;end
   6:begin
    map_state<=0;

    if(down && !boot_busy && !latched)pressed<=candidate;
    if(down && armed && !latched && !boot_busy)begin
     if(candidate==0)begin hits<=0;held_candidate<=0;end
     else if(candidate!=held_candidate)begin held_candidate<=candidate;hits<=1;end
     else begin
      latched<=1;armed<=0;pressed<=candidate;action<=candidate;action_valid<=1;

     end
    end
   end
   default:begin end
  endcase
 end
endmodule
`default_nettype wire
