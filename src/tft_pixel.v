// 界面像素生成：依据区域快照生成数字、文字、按钮与状态颜色。
// 实际角度来自编码器，用户目标按整数角度显示，显示路径不合成测量数据。
`default_nettype none

module tft_pixel #(parameter integer CLK_HZ=50000000,COUNTS_PER_REV=1040)(
 input wire clk,rst_n,frame_start,draw_start,input wire [35:0] animation_levels,
 input wire [8:0] x,input wire [7:0] y,input wire signed [31:0] position,target,
 input wire [11:0] adc,zero,input wire [8:0] step_degrees,amplitude_degrees,
 input wire [7:0] status,input wire touch_down,input wire [1:0] cal_step,
 input wire [3:0] pressed,message,output reg [15:0] pixel,output wire goal_highlight);
 `include "src/tft_layout.vh"
 reg [319:0] labels_rom[0:239];reg [17:0] digits_rom[0:2047];
 reg [79:0] status_rom[0:255];reg [61:0] toggle_rom[0:47];
 initial begin
  $readmemh("src/tft_labels.hex",labels_rom);$readmemh("src/tft_v4_digits.hex",digits_rom);
  $readmemh("src/tft_status.hex",status_rom);$readmemh("src/tft_v4_toggle.hex",toggle_rom);
 end
 function [15:0] rgb;input [23:0] c;begin rgb={c[23:19],c[15:10],c[7:3]};end endfunction
 localparam [15:0] BG=rgb(24'h14181d),HEADER=rgb(24'h1a1e24),CARD=rgb(24'h242a31),
  GROUP=rgb(24'h20252c),BORDER=rgb(24'h424a55),WHITE=rgb(24'he8e9eb),MUTED=rgb(24'ha9b0ba),
  BUTTON=rgb(24'h2e353e),AMBER=rgb(24'hf0c96f),GREEN=rgb(24'h284337),RED=rgb(24'h74313b),CYAN=rgb(24'h384751);
 reg [7:0] flags;reg [3:0] msg;reg highlight_snapshot;
 reg [35:0] levels;reg [11:0] adc_snapshot,zero_snapshot;reg [8:0] step_snapshot,amp_snapshot;
 reg signed [31:0] angle,goal,position_hold,target_hold;
 reg position_overflow,target_overflow;reg [1:0] conversion_stage;
 reg signed [31:0] position_product,target_product,angle_numerator,goal_numerator;
 reg divide_start;wire signed [31:0] angle_result,goal_result;wire angle_done,goal_done,ab,gb;
 reg signed [31:0] previous_target;reg [31:0] highlight_count;
 assign goal_highlight=highlight_count!=0;
 tft_divide ad(.clk(clk),.rst_n(rst_n),.start(divide_start),.numerator(angle_numerator),.denominator(COUNTS_PER_REV),.result(angle_result),.done(angle_done),.busy(ab));
 tft_divide gd(.clk(clk),.rst_n(rst_n),.start(divide_start),.numerator(goal_numerator),.denominator(COUNTS_PER_REV),.result(goal_result),.done(goal_done),.busy(gb));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin previous_target<=0;highlight_count<=0;end
 else begin previous_target<=target;if(target!=previous_target)highlight_count<=CLK_HZ/2;else if(highlight_count!=0)highlight_count<=highlight_count-1'b1;end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  flags<=0;msg<=0;highlight_snapshot<=0;levels<=0;adc_snapshot<=0;zero_snapshot<=0;step_snapshot<=60;amp_snapshot<=30;
  angle<=0;goal<=0;position_hold<=0;target_hold<=0;position_overflow<=0;target_overflow<=0;
  conversion_stage<=0;divide_start<=0;position_product<=0;target_product<=0;angle_numerator<=0;goal_numerator<=0;
 end else begin
  divide_start<=0;
  if(draw_start)begin flags<=status;msg<=message;levels<=animation_levels;highlight_snapshot<=goal_highlight;end
  if(frame_start)begin
   adc_snapshot<=adc;zero_snapshot<=zero;step_snapshot<=step_degrees;amp_snapshot<=amplitude_degrees;
   position_overflow<=position>500000 || position< -500000;target_overflow<=target>500000 || target< -500000;
   position_hold<=(position>500000 || position< -500000) ? 32'sd0 : position;
   target_hold<=(target>500000 || target< -500000) ? 32'sd0 : target;conversion_stage<=1;
  end else case(conversion_stage)
   1:begin position_product<=position_hold*32'sd3600;target_product<=target_hold*32'sd360;conversion_stage<=2;end
   2:begin
    angle_numerator<=position_product+(position_hold<0 ? -COUNTS_PER_REV/2 : COUNTS_PER_REV/2);
    goal_numerator<=target_product+(target_hold<0 ? -COUNTS_PER_REV/2 : COUNTS_PER_REV/2);
    divide_start<=1;conversion_stage<=0;
   end
   default:begin end
  endcase
  if(angle_done)angle<=angle_result;if(goal_done)goal<=goal_result;
 end
 reg [3:0] a0,a1,a2,a3,a4,g0,g1,g2,g3,g4,c0,c1,c2,c3,z0,z1,z2,z3,s0,s1,s2,m0,m1,m2;
 reg [16:0] av,gv;reg an,gn,ao,go;reg [2:0] ai,gi;
 always @(posedge clk)begin
  av<=angle<0 ? -angle : angle;gv<=goal<0 ? -goal : goal;an<=angle<0;gn<=goal<0;
  ao<=position_overflow || angle>99999 || angle< -99999;go<=target_overflow || goal>9999 || goal< -9999;
  ai<=av>=10000 ? 4 : av>=1000 ? 3 : av>=100 ? 2 : 1;gi<=gv>=1000 ? 4 : gv>=100 ? 3 : gv>=10 ? 2 : 1;
  a0<=av%10;a1<=(av/10)%10;a2<=(av/100)%10;a3<=(av/1000)%10;a4<=(av/10000)%10;
  g0<=gv%10;g1<=(gv/10)%10;g2<=(gv/100)%10;g3<=(gv/1000)%10;g4<=(gv/10000)%10;
  c0<=adc_snapshot%10;c1<=(adc_snapshot/10)%10;c2<=(adc_snapshot/100)%10;c3<=adc_snapshot/1000;
  z0<=zero_snapshot%10;z1<=(zero_snapshot/10)%10;z2<=(zero_snapshot/100)%10;z3<=zero_snapshot/1000;
  s0<=step_snapshot%10;s1<=(step_snapshot/10)%10;s2<=step_snapshot/100;m0<=amp_snapshot%10;m1<=(amp_snapshot/10)%10;m2<=amp_snapshot/100;
 end
 reg [3:0] button;integer i;reg [15:0] bg,ink;reg [3:0] status_index;
 reg [2:0] field;reg [1:0] size;reg [8:0] local_x;reg [5:0] gy;reg [4:0] gx;reg [3:0] column,glyph;
 reg [2:0] integers;reg signed_flag,minus_flag,decimal_flag,degree_flag,overflow_flag;
 reg [3:0] n0,n1,n2,n3,n4,digit_column;reg digit_zone;
 reg [15:0] pixel_bg,pixel_ink;reg [4:0] font_x;reg [10:0] font_address;
 reg [17:0] font_row;reg [319:0] label_row;reg [79:0] status_row;reg [61:0] toggle_row;
 reg [8:0] draw_x;reg [7:0] draw_y;reg numeric_valid,draw_status,draw_toggle;
 always @* begin
  button=0;for(i=1;i<=9;i=i+1)if(x>=screen_left(i) && x<=screen_right(i) && y>=screen_top(i) && y<=screen_bottom(i))button=i;
  bg=BG;ink=WHITE;if(y<=22)bg=HEADER;
  if(x>=7 && x<=178 && y>=44 && y<=94)begin bg=CARD;if(x==7 || x==178 || y==44 || y==94)bg=rgb(24'h4a535e);end
  if(x>=184 && x<=313 && y>=44 && y<=94)begin
   bg=highlight_snapshot ? rgb(24'h2c2922) : CARD;
   if(x==184 || x==313 || y==44 || y==94)bg=highlight_snapshot ? rgb(24'hbd974b) : rgb(24'h4a535e);
  end
  if((x>=7 && x<=157 || x>=163 && x<=313) && y>=100 && y<=152)begin
   bg=GROUP;if(x==7 || x==157 || x==163 || x==313 || y==100 || y==152)bg=BORDER;
  end
  if(x>=7 && x<=210 && y>=158 && y<=195)begin bg=rgb(24'h20272c);if(x==7 || x==210 || y==158 || y==195)bg=rgb(24'h495964);end
  if(button!=0)begin
   bg=button==2 ? RED : button==1 ? GREEN : BUTTON;
   if((button>=5 && !flags[2]) || (button!=2 && !flags[0]))begin bg=GROUP;ink=MUTED;end
   if(x==screen_left(button) || x==screen_right(button) || y==screen_top(button) || y==screen_bottom(button))bg=rgb(24'h505a67);
   if(levels[(button-1)*4+:4]!=0)bg=button==2 ? rgb(24'h9c4350) : button==1 ? rgb(24'h3c6953) : rgb(24'h465568);
  end
  if(x>=143 && x<=204 && y>=164 && y<=188)begin
   bg=!flags[2] ? GROUP : flags[6] ? rgb(24'h3c6953) : CYAN;
   if(x==143 || x==204 || y==164 || y==188)bg=rgb(24'h697d8b);
  end
  status_index=0;if(!flags[0])status_index=1;else if(flags[3])status_index=5;else if(flags[2])status_index=4;else if(flags[4])status_index=3;else if(flags[1])status_index=2;
  if(msg==1)status_index=10;if(msg==3)status_index=6;if(msg==4)status_index=7;if(msg==5)status_index=11;
  if(msg==6)status_index=12;if(msg==7)status_index=8;if(msg==8)status_index=9;if(msg==9)status_index=14;if(flags[3])status_index=5;
  field=0;size=0;local_x=0;gy=0;digit_zone=0;
  if(y>=59 && y<91 && x>=15 && x<177)begin field=1;size=0;local_x=x-15;gy=y-59;digit_zone=1;end
  if(y>=62 && y<90 && x>=192 && x<312)begin field=2;size=1;local_x=x-192;gy=y-62;digit_zone=1;end
  if(y>=24 && y<44 && x>=39 && x<84)begin field=3;size=3;local_x=x-39;gy=y-24;digit_zone=1;end
  if(y>=25 && y<43 && x>=270 && x<310)begin field=4;size=2;local_x=x-270;gy=y-25;digit_zone=1;end
  if(y>=100 && y<120 && x>=43 && x<88)begin field=5;size=3;local_x=x-43;gy=y-100;digit_zone=1;end
  if(y>=161 && y<179 && x>=101 && x<141)begin field=6;size=2;local_x=x-101;gy=y-161;digit_zone=1;end
  if((button==5 || button==6) && y>=118 && y<138 && x>=screen_left(button)+5 && x<screen_right(button)-2)begin field=7;size=3;local_x=x-screen_left(button)-5;gy=y-118;digit_zone=1;end
  column=0;gx=0;
  case(size)0:begin column=local_x/18;gx=local_x%18;end 1:begin column=local_x/14;gx=local_x%14;end 2:begin column=local_x/8;gx=local_x%8;end 3:begin column=local_x/9;gx=local_x%9;end endcase
  integers=1;signed_flag=0;minus_flag=0;decimal_flag=0;degree_flag=0;overflow_flag=0;n0=0;n1=0;n2=0;n3=0;n4=0;
  case(field)
   1:begin integers=ai;signed_flag=1;minus_flag=an;decimal_flag=1;degree_flag=1;overflow_flag=ao;n0=a0;n1=a1;n2=a2;n3=a3;n4=a4;end
   2:begin integers=gi;signed_flag=1;minus_flag=gn;decimal_flag=0;degree_flag=1;overflow_flag=go;n0=g0;n1=g1;n2=g2;n3=g3;n4=g4;end
   3:begin integers=adc_snapshot>=1000 ? 4 : adc_snapshot>=100 ? 3 : adc_snapshot>=10 ? 2 : 1;n0=c0;n1=c1;n2=c2;n3=c3;end
   4:begin integers=zero_snapshot>=1000 ? 4 : zero_snapshot>=100 ? 3 : zero_snapshot>=10 ? 2 : 1;n0=z0;n1=z1;n2=z2;n3=z3;end
   5,7:begin integers=step_snapshot>=100 ? 3 : step_snapshot>=10 ? 2 : 1;degree_flag=1;signed_flag=field==7;minus_flag=button==5;n0=s0;n1=s1;n2=s2;end
   6:begin integers=amp_snapshot>=100 ? 3 : amp_snapshot>=10 ? 2 : 1;degree_flag=1;overflow_flag=amp_snapshot>=511;n0=m0;n1=m1;n2=m2;end
   default:digit_zone=0;
  endcase
  glyph=15;digit_column=column-(signed_flag ? 1 : 0);
  if(signed_flag && column==0)glyph=minus_flag ? 11 : 10;
  else if(digit_column<integers)case(integers-1-digit_column+(decimal_flag ? 1 : 0))0:glyph=n0;1:glyph=n1;2:glyph=n2;3:glyph=n3;4:glyph=n4;default:glyph=15;endcase
  else if(decimal_flag && digit_column==integers)glyph=12;
  else if(decimal_flag && digit_column==integers+1)glyph=n0;
  else if(degree_flag && digit_column==integers+(decimal_flag ? 2 : 0))glyph=13;
  else digit_zone=0;
  if(overflow_flag)begin glyph=column<5 ? 11 : 15;digit_zone=column<5;end
  font_address={size,glyph,gy[4:0]};
 end
 always @(posedge clk)begin
  label_row<=labels_rom[y];font_row<=digits_rom[font_address];status_row<=status_rom[{status_index,4'b0}+((y-4)&15)];
  toggle_row<=toggle_rom[(flags[6] ? 24 : 0)+((y>=164 && y<=187) ? y-164 : 0)];
  pixel_bg<=bg;pixel_ink<=ink;draw_x<=x;draw_y<=y;numeric_valid<=digit_zone;font_x<=gx;
  draw_status<=y>=4 && y<20 && x>=232 && x<312;draw_toggle<=x>=143 && x<=204 && y>=164 && y<188;
 end
 always @(posedge clk)begin
  pixel<=pixel_bg;if(label_row[319-draw_x])pixel<=pixel_ink;
  if(numeric_valid && font_row[17-font_x])pixel<=(draw_y>=62 && draw_y<90 && draw_x>=192 && highlight_snapshot) ? AMBER : WHITE;
  if(draw_status)pixel<=status_row[311-draw_x] ? (flags[3] ? rgb(24'hf08088) : (msg==7 || msg==8 || msg==9 || !flags[0]) ? AMBER : rgb(24'hbbebcf)) : HEADER;
  if(draw_toggle && toggle_row[204-draw_x])pixel<=WHITE;
  if(draw_x>=236 && draw_x<=240 && draw_y>=9 && draw_y<=13)pixel<=flags[3] ? rgb(24'hf08088) : flags[2] ? rgb(24'h73dfa4) : MUTED;
 end
endmodule
`default_nettype wire
