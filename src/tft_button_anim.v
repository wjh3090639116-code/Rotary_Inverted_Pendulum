// 按钮视觉状态机：同一时刻仅保留一个按压反馈，新的按键优先替换旧反馈。
// 动画等级由固定节拍推进，像素模块据此调整按钮尺寸与亮度。
`default_nettype none

module tft_button_anim #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,input wire [3:0] contact_button,output wire [35:0] levels);
 reg [31:0] ms_count;wire ms_tick=ms_count==CLK_HZ/1000-1;
 reg [3:0] previous_contact;reg [9:0] age[0:8];reg [2:0] substep[0:8];
 reg [3:0] level[0:8];reg [8:0] active;integer i;
 genvar k;generate for(k=0;k<9;k=k+1)begin:pack_levels
  assign levels[k*4+:4]=level[k];
 end endgenerate
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin ms_count<=0;previous_contact<=0;active<=0;
  for(i=0;i<9;i=i+1)begin age[i]<=0;substep[i]<=0;level[i]<=0;end
 end else begin
  previous_contact<=contact_button;
  if(ms_tick)ms_count<=0;else ms_count<=ms_count+1'b1;
  for(i=0;i<9;i=i+1)begin
   if(contact_button!=0 && previous_contact!=contact_button)begin
    age[i]<=0;substep[i]<=0;
    if(contact_button==i+1)begin active[i]<=1;if(level[i]==0)level[i]<=1;end
    else begin active[i]<=0;level[i]<=0;end
   end else if(ms_tick && active[i])begin
    if(age[i]<500)age[i]<=age[i]+1'b1;
    if(substep[i]==7)begin
     substep[i]<=0;
     if(contact_button==i+1 || age[i]<500)begin
      if(level[i]<8)level[i]<=level[i]+1'b1;
     end else if(level[i]!=0)level[i]<=level[i]-1'b1;
     else active[i]<=0;
    end else substep[i]<=substep[i]+1'b1;
   end
  end
 end
endmodule
`default_nettype wire
