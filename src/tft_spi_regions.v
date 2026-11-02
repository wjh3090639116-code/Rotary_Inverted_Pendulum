// 显示区域传输：设置液晶地址窗口后连续发送一个矩形区域的像素。
// 窗口传输期间使用同一帧快照，完成握手后才切换至下一区域。
`default_nettype none

module tft_spi_regions #(parameter integer CLK_HZ=50000000,INIT_BYTES=92,SPI_DIV=3)(
 input wire clk,rst_n,input wire [15:0] pixel,
 input wire window_valid,input wire [8:0] window_x0,window_x1,
 input wire [7:0] window_y0,window_y1,
 output wire window_ready,output reg window_started,window_done,
 output reg [8:0] x,output reg [7:0] y,
 output wire sck,mosi,dc,output reg reset_n,output wire cs_n);
 reg [9:0] init_rom[0:INIT_BYTES-1];initial $readmemh("src/tft_init.hex",init_rom);
 localparam RESET=0,RECOVER=1,INIT=2,SLEEP=3,INIT_TAIL=4,IDLE=5,WINDOW=6,SETTLE=7,HIGH=8,LOW=9,TAIL=10;
 reg [3:0] state,command_index;reg [31:0] delay_count;reg [7:0] index;
 reg [3:0] settle;reg [8:0] left,right;reg [7:0] top,bottom;reg [15:0] pixel_hold;
 wire ready;wire [9:0] token=init_rom[index];reg [7:0] address_byte;
 always @* case(command_index)
  0:address_byte=8'h2a;1:address_byte={7'b0,left[8]};2:address_byte=left[7:0];
  3:address_byte={7'b0,right[8]};4:address_byte=right[7:0];5:address_byte=8'h2b;
  6:address_byte=0;7:address_byte=top;
  8:address_byte=0;9:address_byte=bottom;
  default:address_byte=8'h2c;
 endcase
 wire send=(state==INIT && !token[9]) || state==WINDOW || state==HIGH || state==LOW;
 wire [7:0] data=state==INIT ? token[7:0] : state==WINDOW ? address_byte : state==HIGH ? pixel[15:8] : pixel_hold[7:0];
 wire dc_bit=state==INIT ? token[8] : state==WINDOW ? !(command_index==0 || command_index==5 || command_index==10) : 1'b1;
 assign window_ready=state==IDLE;assign cs_n=!reset_n;
 tft_spi_byte #(.DIV(SPI_DIV)) u_spi(.clk(clk),.rst_n(rst_n && reset_n),.valid(send),.dc_in(dc_bit),.data(data),.ready(ready),.sck(sck),.mosi(mosi),.dc(dc));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin state<=RESET;delay_count<=CLK_HZ/50;reset_n<=0;index<=0;x<=0;y<=0;window_started<=0;window_done<=0;command_index<=0;settle<=0;left<=0;right<=0;top<=0;bottom<=0;pixel_hold<=0;end
 else begin
  window_started<=0;window_done<=0;
  case(state)
   RESET:if(delay_count!=0)delay_count<=delay_count-1'b1;else begin reset_n<=1;delay_count<=(CLK_HZ/1000)*120;state<=RECOVER;end
   RECOVER:if(delay_count!=0)delay_count<=delay_count-1'b1;else state<=INIT;
   INIT:if(token[9])begin if(ready)begin delay_count<=(CLK_HZ/1000)*120;state<=SLEEP;end end
        else if(ready)begin if(index==INIT_BYTES-1)state<=INIT_TAIL;else index<=index+1'b1;end
   SLEEP:if(delay_count!=0)delay_count<=delay_count-1'b1;else begin index<=index+1'b1;state<=INIT;end
   INIT_TAIL:if(ready)state<=IDLE;
   IDLE:if(window_valid)begin
    left<=window_x0;right<=window_x1;top<=window_y0;bottom<=window_y1;
    x<=window_x0;y<=window_y0;command_index<=0;state<=WINDOW;window_started<=1;
   end
   WINDOW:if(ready)begin
    if(command_index==10)begin settle<=0;state<=SETTLE;end
    else command_index<=command_index+1'b1;
   end
   SETTLE:if(settle==7)state<=HIGH;else settle<=settle+1'b1;
   HIGH:if(ready)begin pixel_hold<=pixel;state<=LOW;end
   LOW:if(ready)begin
    if(x==right)begin x<=left;if(y==bottom)state<=TAIL;else begin y<=y+1'b1;settle<=0;state<=SETTLE;end end
    else begin x<=x+1'b1;settle<=0;state<=SETTLE;end
   end
   TAIL:if(ready)begin window_done<=1;state<=IDLE;end
   default:state<=RESET;
  endcase
 end
endmodule
`default_nettype wire
