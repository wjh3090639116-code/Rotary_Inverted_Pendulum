// 电阻触摸采样：按控制器命令时序读取两轴坐标。
// 转换期间维持接触状态，仅在空闲且稳定释放时清除按下状态。
`default_nettype none

module tft_touch #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,irq_n,miso,
 output reg sck,mosi,cs_n,output reg [11:0] raw_x,raw_y,
 output reg sample_valid,output reg down);
 localparam integer HALF=CLK_HZ/2000000>=3 ? CLK_HZ/2000000 : 3;
 reg [1:0] irq_sync,do_sync;

 reg [31:0] release_count;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin down<=0;release_count<=0;end
 else if(!irq_sync[1])begin down<=1;release_count<=0;end
 else if(state==0 && cs_n)begin
  if(release_count<CLK_HZ/500)release_count<=release_count+1'b1;
  else down<=0;
 end else release_count<=0;
 reg [2:0] state;reg [31:0] count;reg [5:0] bit_index;
 reg axis;reg [15:0] shift;reg [11:0] x_hold;
 wire [7:0] command=axis ? 8'h90 : 8'hd0;
 wire [15:0] received={shift[14:0],do_sync[1]};
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin irq_sync<=3;do_sync<=0;end
 else begin irq_sync<={irq_sync[0],irq_n};do_sync<={do_sync[0],miso};end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin state<=0;count<=0;sck<=0;mosi<=0;cs_n<=1;raw_x<=0;raw_y<=0;sample_valid<=0;bit_index<=0;axis<=0;shift<=0;x_hold<=0;end
 else begin
  sample_valid<=0;
  if(count!=0)count<=count-1'b1;
  else case(state)
   0:begin sck<=0;cs_n<=1;if(!irq_sync[1])begin axis<=0;state<=1;end end
   1:begin cs_n<=0;sck<=0;bit_index<=0;mosi<=command[7];shift<=0;count<=HALF-1;state<=2;end
   2:begin
    count<=HALF-1;
    if(!sck)begin
     sck<=1;if(bit_index>=9)shift<=received;
    end else begin
     sck<=0;
     if(bit_index==24)begin
      cs_n<=1;
      if(!axis)begin x_hold<=shift[15:4];axis<=1;count<=HALF*2;state<=1;end
      else begin
       if(down)begin raw_x<=x_hold;raw_y<=shift[15:4];sample_valid<=1;end
       state<=0;count<=CLK_HZ/200;
      end
     end else begin
      bit_index<=bit_index+1'b1;
      if(bit_index<7)mosi<=command[6-bit_index];else mosi<=0;
      if(bit_index==7)count<=CLK_HZ*6/1000000;
     end
    end
   end
   default:state<=0;
  endcase
 end
endmodule
`default_nettype wire
