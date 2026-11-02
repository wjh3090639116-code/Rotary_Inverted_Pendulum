// 实体按键消抖：稳定按下产生一次脉冲，持续按住不重复触发。
// 释放状态确认后允许下一次有效按压。
`default_nettype none

module reciprocal_key #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,key_n,output wire press);
 localparam integer LIMIT=CLK_HZ/50;
 reg [1:0] sync;reg stable,previous,armed;reg [31:0] count;
 assign press=armed && previous && !stable;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin sync<=0;stable<=0;previous<=0;armed<=0;count<=0;end
 else begin
  sync<={sync[0],key_n};previous<=stable;
  if(stable)armed<=1;
  if(sync[1]==stable)count<=0;
  else if(count==LIMIT-1)begin stable<=sync[1];count<=0;end
  else count<=count+1'b1;
 end
endmodule

`default_nettype wire
