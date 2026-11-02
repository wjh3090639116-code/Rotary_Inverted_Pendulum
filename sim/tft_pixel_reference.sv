// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`default_nettype none

module tft_pixel_reference #(parameter integer COUNTS_PER_REV=1040)(
 input wire clk,rst_n,frame_start,draw_start,input wire [27:0] animation_levels,input wire [7:0] x,input wire [8:0] y,
 input wire signed [31:0] position,target,input wire [11:0] zero,input wire [7:0] status,
 input wire touch_down,input wire [1:0] cal_step,input wire [2:0] pressed,input wire [3:0] message,
 output reg [15:0] pixel)                                       ;
 reg [239:0] label_rom[0:319],cal_rom[0:319];
 reg [79:0] status_rom[0:255];
 reg [17:0] digit_rom[0:383];
 initial begin
  $readmemh("src/tft_labels.hex",label_rom);$readmemh("src/tft_cal_labels.hex",cal_rom);
  $readmemh("src/tft_status.hex",status_rom);$readmemh("src/tft_digits.hex",digit_rom);
 end
 reg [239:0] labels,cal_labels;reg [79:0] status_line;
 reg [1:0] cal;reg [2:0] press;reg [3:0] msg;reg [7:0] flags;
 reg signed [31:0] angle,goal;
 reg [15:0] magnitude,goal_magnitude;reg negative,goal_negative;
 reg [3:0] digit0,digit1,digit2,digit3,digit4,goal0,goal1,goal2,goal3,goal4;
 reg [2:0] significant,column;reg [15:0] display_value;reg display_negative;
 reg [3:0] status_index;reg [11:0] zero_snapshot;
 reg [3:0] zero0,zero1,zero2,zero3;reg [1:0] value_field;
 reg [7:0] local_x;reg [8:0] local_y;
 reg [3:0] glyph;reg [4:0] gx,gy;reg digit_zone;
 reg [17:0] digit_line;
 reg [2:0] button;reg [27:0] animation_snapshot;reg touch_snapshot;
 reg [3:0] animation_level;reg [8:0] brightness;
 reg [12:0] inverse_scale;reg [7:0] box_x0,box_width,new_width,draw_left;
 reg [8:0] box_y0,box_height,new_height,draw_top;
 reg inside_draw;reg [7:0] label_x,dx;reg [8:0] label_y,dy;
 reg [20:0] label_mul_x,label_mul_y;
 reg [15:0] background;
 localparam [15:0] BG=16'h0863,CARD=16'h10e5,WHITE=16'he75d,MUTED=16'h8cb3,
   BLUE=16'h1baf,GREEN=16'h1bad,RED=16'hc9c8,BORDER=16'h2948,AMBER=16'hfd26;
 function [15:0] brighten;
  input [15:0] color;input [8:0] gain;
  reg [14:0] r,g,b;reg [4:0] rr,bb;reg [5:0] gg;
  begin
   r=color[15:11]*gain+128;g=color[10:5]*gain+128;b=color[4:0]*gain+128;
   rr=(r>>8)>31 ? 31 : r[12:8];gg=(g>>8)>63 ? 63 : g[13:8];bb=(b>>8)>31 ? 31 : b[12:8];
   brighten={rr,gg,bb};
  end
 endfunction

 reg [1:0] angle_stage;
 reg signed [31:0] position_hold,target_hold,position_product,target_product;
 reg signed [31:0] angle_numerator,goal_numerator;
 reg angle_start;
 wire signed [31:0] angle_result,goal_result;wire angle_done,goal_done;
 wire angle_busy,goal_busy;
 tft_divide angle_div(.clk(clk),.rst_n(rst_n),.start(angle_start),.numerator(angle_numerator),.denominator(COUNTS_PER_REV),.result(angle_result),.done(angle_done),.busy(angle_busy));
 tft_divide goal_div(.clk(clk),.rst_n(rst_n),.start(angle_start),.numerator(goal_numerator),.denominator(COUNTS_PER_REV),.result(goal_result),.done(goal_done),.busy(goal_busy));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin angle_stage<=0;angle_start<=0;position_hold<=0;target_hold<=0;position_product<=0;target_product<=0;angle_numerator<=0;goal_numerator<=0;end
 else begin
  angle_start<=0;
  if(frame_start)begin position_hold<=position;target_hold<=target;angle_stage<=1;end
  else case(angle_stage)
   1:begin position_product<=position_hold*32'sd360;target_product<=target_hold*32'sd360;angle_stage<=2;end
   2:begin
    angle_numerator<=position_product+(position_hold<0 ? -COUNTS_PER_REV/2 : COUNTS_PER_REV/2);
    goal_numerator<=target_product+(target_hold<0 ? -COUNTS_PER_REV/2 : COUNTS_PER_REV/2);
    angle_start<=1;angle_stage<=0;
   end
   default:begin end
  endcase
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin animation_snapshot<=0;touch_snapshot<=0;zero_snapshot<=0;zero0<=0;zero1<=0;zero2<=0;zero3<=0;cal<=3;press<=0;msg<=0;flags<=0;angle<=0;goal<=0;magnitude<=0;goal_magnitude<=0;negative<=0;goal_negative<=0;
 digit4<=0;goal4<=0;digit0<=0;digit1<=0;digit2<=0;digit3<=0;goal0<=0;goal1<=0;goal2<=0;goal3<=0;
 end else begin
  if(frame_start)begin
   zero_snapshot<=zero;cal<=cal_step;press<=pressed;

  end
  if(angle_done)angle<=angle_result;
  if(goal_done)goal<=goal_result;
  if(draw_start)begin animation_snapshot<=animation_levels;touch_snapshot<=touch_down;msg<=message;flags<=status;end

  zero0<=zero_snapshot%10;zero1<=(zero_snapshot/10)%10;
  zero2<=(zero_snapshot/100)%10;zero3<=(zero_snapshot/1000)%10;
  negative<=angle<0;goal_negative<=goal<0;
  magnitude<=angle<0 ? -angle : angle;goal_magnitude<=goal<0 ? -goal : goal;
  digit4<=(magnitude/10000)%10;goal4<=(goal_magnitude/10000)%10;
  digit0<=magnitude%10;digit1<=(magnitude/10)%10;digit2<=(magnitude/100)%10;digit3<=(magnitude/1000)%10;
  goal0<=goal_magnitude%10;goal1<=(goal_magnitude/10)%10;goal2<=(goal_magnitude/100)%10;goal3<=(goal_magnitude/1000)%10;
 end
 always @* begin
  status_index=0;
  if(!flags[0])status_index=1;
  else if(flags[3] && msg!=9)status_index=5;
  else if(flags[2])status_index=4;
  else if(flags[4])status_index=3;
  else if(flags[1])status_index=2;
  if(touch_snapshot && msg==0)status_index=13;
  if(msg==1)status_index=10;
  if(msg==2)status_index=1;
  if(msg==3)status_index=6;
  if(msg==4)status_index=7;
  if(msg==7)status_index=8;
  if(msg==8)status_index=9;
  if(msg==9)status_index=14;
  if(msg==5)status_index=11;
  if(msg==6)status_index=12;

  if(flags[3] && msg!=9)status_index=5;
  button=0;
  if(x>=12 && x<228)begin
   if(y>=140 && y<176)begin if(x<118)button=1;else if(x>=122)button=2;end
   else if(y>=184 && y<218)button=3;
   else if(y>=226 && y<260)button=4;
   else if(y>=268 && y<308)begin if(x<118)button=5;else if(x>=122)button=6;end
   else if(y>=96 && y<128 && x>=148)button=7;
  end
  box_x0=0;box_y0=0;box_width=0;box_height=0;
  case(button)
   1:begin box_x0=12;box_y0=140;box_width=106;box_height=36;end
   2:begin box_x0=122;box_y0=140;box_width=106;box_height=36;end
   3:begin box_x0=12;box_y0=184;box_width=216;box_height=34;end
   4:begin box_x0=12;box_y0=226;box_width=216;box_height=34;end
   5:begin box_x0=12;box_y0=268;box_width=106;box_height=40;end
   6:begin box_x0=122;box_y0=268;box_width=106;box_height=40;end
   7:begin box_x0=148;box_y0=96;box_width=80;box_height=32;end
   default:begin end
  endcase
  animation_level=0;
  if(button!=0)animation_level=animation_snapshot[(button-1)*4+:4];
  inverse_scale=4096;brightness=256;
  case(animation_level)
   0:begin inverse_scale=4096;brightness=256;end
   1:begin inverse_scale=4174;brightness=266;end
   2:begin inverse_scale=4256;brightness=275;end
   3:begin inverse_scale=4340;brightness=285;end
   4:begin inverse_scale=4428;brightness=294;end
   5:begin inverse_scale=4520;brightness=304;end
   6:begin inverse_scale=4615;brightness=314;end
   7:begin inverse_scale=4715;brightness=323;end
   8:begin inverse_scale=4819;brightness=333;end
   default:begin inverse_scale=4819;brightness=333;end
  endcase
  new_width=box_width;new_height=box_height;
  case(animation_level)
   1:begin new_width=(button==3 || button==4) ? 212 : button==7 ? 78 : 104;new_height=(button==3 || button==4) ? 33 : (button==5 || button==6) ? 39 : button==7 ? 31 : 35;end
   2:begin new_width=(button==3 || button==4) ? 208 : button==7 ? 77 : 102;new_height=(button==3 || button==4) ? 33 : (button==5 || button==6) ? 38 : button==7 ? 31 : 35;end
   3:begin new_width=(button==3 || button==4) ? 204 : button==7 ? 76 : 100;new_height=(button==3 || button==4) ? 32 : (button==5 || button==6) ? 38 : button==7 ? 30 : 34;end
   4:begin new_width=(button==3 || button==4) ? 200 : button==7 ? 74 : 98;new_height=(button==3 || button==4) ? 31 : (button==5 || button==6) ? 37 : button==7 ? 30 : 33;end
   5:begin new_width=(button==3 || button==4) ? 196 : button==7 ? 72 : 96;new_height=(button==3 || button==4) ? 31 : (button==5 || button==6) ? 36 : button==7 ? 29 : 33;end
   6:begin new_width=(button==3 || button==4) ? 192 : button==7 ? 71 : 94;new_height=(button==3 || button==4) ? 30 : (button==5 || button==6) ? 36 : button==7 ? 28 : 32;end
   7:begin new_width=(button==3 || button==4) ? 188 : button==7 ? 70 : 92;new_height=(button==3 || button==4) ? 30 : (button==5 || button==6) ? 35 : button==7 ? 28 : 31;end
   8:begin new_width=(button==3 || button==4) ? 184 : button==7 ? 68 : 90;new_height=(button==3 || button==4) ? 29 : (button==5 || button==6) ? 34 : button==7 ? 27 : 31;end
   default:begin end
  endcase
  draw_left=box_x0+((box_width-new_width)>>1);
  draw_top=box_y0+((box_height-new_height)>>1);
  inside_draw=button!=0 && x>=draw_left && x<draw_left+new_width && y>=draw_top && y<draw_top+new_height;
  label_x=x;label_y=y;dx=0;dy=0;label_mul_x=0;label_mul_y=0;
  if(inside_draw)begin
   dx=x-draw_left;dy=y-draw_top;label_mul_x=dx*inverse_scale;label_mul_y=dy*inverse_scale;
   label_x=box_x0+(label_mul_x>>12);label_y=box_y0+(label_mul_y>>12);
  end
  digit_zone=0;glyph=0;gx=0;gy=0;local_x=0;local_y=0;
  value_field=0;display_value=magnitude;display_negative=negative;
  if(y>=65 && y<97 && x>=32 && x<140)begin
   digit_zone=1;local_x=x-32;gy=y-65;gx=local_x%18;
  end else if(y>=103 && y<119 && x>=57 && x<111)begin
   digit_zone=1;local_x=(x-57)*2;gy=(y-103)*2;gx=local_x%18;
   value_field=1;display_value=goal_magnitude;display_negative=goal_negative;
  end
  if(y>=46 && y<62 && x>=166 && x<220)begin
   digit_zone=1;local_x=(x-166)*2;gy=(y-46)*2;gx=local_x%18;
   value_field=2;display_value={4'b0,zero_snapshot};display_negative=0;
  end
  significant=display_value>=10000 ? 5 : display_value>=1000 ? 4 : display_value>=100 ? 3 : display_value>=10 ? 2 : 1;
  column=local_x/18;
  if(column<5-significant)digit_zone=0;
  else if(column==5-significant)glyph=display_negative ? 11 : 10;
  else case(column)
   1:glyph=value_field==2 ? 4'd0 : value_field==1 ? goal4 : digit4;
   2:glyph=value_field==2 ? zero3 : value_field==1 ? goal3 : digit3;
   3:glyph=value_field==2 ? zero2 : value_field==1 ? goal2 : digit2;
   4:glyph=value_field==2 ? zero1 : value_field==1 ? goal1 : digit1;
   5:glyph=value_field==2 ? zero0 : value_field==1 ? goal0 : digit0;
   default:digit_zone=0;
  endcase

 end
 always @(posedge clk)begin
  labels<=label_rom[label_y];cal_labels<=cal_rom[y];
  status_line<=status_rom[{status_index,4'b0}+(y-10)];
  digit_line<=digit_rom[{glyph,5'b0}+gy];
 end
 reg [7:0] cx;reg [8:0] cy;
 always @* begin
  background=BG;
  if(x>=12 && x<228 && y>=38 && y<130)background=CARD;

  if(msg==6 && x>=55 && x<142 && y>=101 && y<122)background=16'h2249;
  if(msg==3 && x>=164 && x<222 && y>=44 && y<64)background=16'h2249;
  if(y==32)background=BORDER;
  if(button!=0 && inside_draw)begin
   background=button==2 ? RED : button==1 ? GREEN : BLUE;
   if((button>=5 && button<=7 && !flags[2]) || (button!=2 && !flags[0]))background=BORDER;
   background=brighten(background,brightness);
  end
  pixel=background;
  if(labels[239-label_x] && (button==0 || inside_draw))pixel=button==0 ? WHITE : brighten(WHITE,brightness);
  if(digit_zone && digit_line[17-gx])pixel=16'h6f3a;
  if(y>=10 && y<26 && x>=148 && x<228)begin
   pixel=BG;
   if(status_line[227-x])pixel=flags[3] ? RED : (msg==7 || msg==8 || msg==9 || !flags[0]) ? AMBER : 16'h6f3a;
  end

 end
endmodule
`default_nettype wire
