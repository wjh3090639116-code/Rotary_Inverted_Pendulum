// 显示刷新调度：跟踪按钮、数据、状态和设置区域的变化，只刷新需要更新的区域。
// 每个区域启动时锁存显示快照，避免传输过程中混入其他时刻的数据。
`default_nettype none
module tft_region_scheduler #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,input wire [35:0] levels,input wire [3:0] contact_button,
 input wire [31:0] position,target,input wire [11:0] adc,zero,input wire [8:0] step_degrees,amplitude_degrees,
 input wire [7:0] status,input wire [3:0] message,input wire touch_down,goal_highlight,
 input wire window_ready,window_started,window_done,output reg window_valid,
 output reg [8:0] x0,x1,output reg [7:0] y0,y1,output wire draw_start,data_start);
 `include "src/tft_layout.vh"
 localparam INITIAL=0,BUTTON=1,STATUS=2,CARD=3,SETTINGS=4;
 reg [8:0] dirty;reg [35:0] previous_levels;reg [7:0] previous_status;
 reg [3:0] previous_contact,urgent,chosen_button;reg initial_active,card_active,settings_active;
 reg [7:0] initial_y,card_y,settings_y,accepted_y;reg [2:0] choice,accepted_kind;reg [31:0] cooldown;
 reg [88:0] data_snapshot;wire [88:0] data_now={position,target,adc,zero,goal_highlight};
 reg [18:0] settings_snapshot;wire [18:0] settings_now={step_degrees,amplitude_degrees,status[6]};
 reg [12:0] status_snapshot;wire [12:0] status_now={status,message,touch_down};wire accept=window_valid && window_ready;
 assign draw_start=window_started;
 assign data_start=window_started && ((accepted_kind==INITIAL && accepted_y==0) || (accepted_kind==CARD && accepted_y==24) || (accepted_kind==SETTINGS && accepted_y==96));
 integer i,j;
 always @* begin
  chosen_button=0;for(j=8;j>=0;j=j-1)if(dirty[j])chosen_button=j+1;
  if(contact_button!=0 && dirty[contact_button-1])chosen_button=contact_button;
  choice=INITIAL;window_valid=1;x0=0;x1=319;y0=initial_y;y1=initial_y+3;
  if(urgent!=0 && dirty[urgent-1])begin choice=BUTTON;chosen_button=urgent;end
  else if(status_now!=status_snapshot)choice=STATUS;
  else if(chosen_button!=0)choice=BUTTON;
  else if(initial_active)choice=INITIAL;
  else if(settings_active || settings_now!=settings_snapshot)choice=SETTINGS;
  else if(card_active || (data_now!=data_snapshot && cooldown==0))choice=CARD;
  else window_valid=0;
  if(choice==BUTTON)begin x0=screen_left(chosen_button);x1=screen_right(chosen_button);y0=screen_top(chosen_button);y1=screen_bottom(chosen_button);end
  else if(choice==STATUS)begin x0=0;x1=319;y0=0;y1=23;end
  else if(choice==CARD)begin y0=card_active ? card_y : 8'd24;y1=y0+3;end
  else if(choice==SETTINGS)begin y0=settings_active ? settings_y : 8'd96;y1=y0+3;end
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  dirty<=0;previous_levels<=0;previous_status<=0;previous_contact<=0;urgent<=0;initial_active<=1;card_active<=0;settings_active<=0;
  initial_y<=0;card_y<=24;settings_y<=96;data_snapshot<=0;settings_snapshot<=0;status_snapshot<=0;cooldown<=0;accepted_kind<=INITIAL;accepted_y<=0;
 end else begin
  previous_levels<=levels;previous_status<=status;previous_contact<=contact_button;if(cooldown!=0)cooldown<=cooldown-1'b1;
  if(contact_button!=0 && contact_button!=previous_contact)urgent<=contact_button;
  else if(accept && choice==BUTTON && chosen_button==urgent)urgent<=0;
  for(i=0;i<9;i=i+1)begin
   if(levels[i*4+:4]!=previous_levels[i*4+:4] || status!=previous_status)dirty[i]<=1;
   else if(accept && choice==BUTTON && chosen_button==i+1)dirty[i]<=0;
  end
  if(accept)begin
   accepted_kind<=choice;accepted_y<=y0;if(choice==STATUS)status_snapshot<=status_now;
   if(choice==INITIAL)begin
    if(initial_y==0)begin data_snapshot<=data_now;settings_snapshot<=settings_now;end
    if(initial_y>=236)begin initial_active<=0;initial_y<=0;end else initial_y<=initial_y+4;
   end
   if(choice==CARD)begin
    if(!card_active)begin card_active<=1;card_y<=28;data_snapshot<=data_now;cooldown<=CLK_HZ/20;end
    else if(card_y>=92)begin card_active<=0;card_y<=24;end else card_y<=card_y+4;
   end
   if(choice==SETTINGS)begin
    if(!settings_active)begin settings_active<=1;settings_y<=100;settings_snapshot<=settings_now;end
    else if(settings_y>=192)begin settings_active<=0;settings_y<=96;end else settings_y<=settings_y+4;
   end
  end
 end
endmodule
`default_nettype wire
