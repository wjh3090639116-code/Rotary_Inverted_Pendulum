// 液晶串行发送：串行时钟空闲为低，最高有效位先发送。
// 数据与命令选择随传输请求锁存，发送期间通过忙信号阻止重复接收。
`default_nettype none

module tft_spi_byte #(parameter integer DIV=4)(
 input wire clk,rst_n,valid,dc_in,input wire [7:0] data,
 output wire ready,output reg sck,mosi,dc);
 reg busy;reg [7:0] shift;reg [2:0] bit_index;reg [7:0] count;
 assign ready=!busy;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin busy<=0;sck<=0;mosi<=0;dc<=1;shift<=0;bit_index<=0;count<=0;end
 else if(!busy)begin
  sck<=0;
  if(valid)begin busy<=1;shift<=data;mosi<=data[7];dc<=dc_in;bit_index<=7;count<=DIV-1;end
 end else if(count!=0)count<=count-1'b1;
 else begin
  count<=DIV-1;
  if(!sck)sck<=1;
  else begin
   sck<=0;
   if(bit_index==0)busy<=0;
   else begin shift<={shift[6:0],1'b0};mosi<=shift[6];bit_index<=bit_index-1'b1;end
  end
 end
endmodule

module tft_spi_display_legacy #(parameter integer CLK_HZ=50000000,INIT_BYTES=92)(
 input wire clk,rst_n,input wire [15:0] pixel,
 output reg [8:0] x,output reg [7:0] y,output reg frame_start,
 output wire sck,mosi,dc,output reg reset_n,output wire cs_n);
 reg [9:0] init_rom[0:INIT_BYTES-1];
 initial $readmemh("src/tft_init.hex",init_rom);
 localparam RESET=0,RECOVER=1,INIT=2,SLEEP=3,FRAME=4,SETTLE=5,HIGH=6,LOW=7;
 reg [3:0] state;reg [31:0] delay_count;reg [7:0] index;reg [1:0] settle;
 reg [15:0] pixel_hold;
 wire ready;wire [9:0] token=init_rom[index];
 wire send=(state==INIT && !token[9]) || state==FRAME || state==HIGH || state==LOW;
 wire [7:0] data=state==INIT ? token[7:0] : state==FRAME ? 8'h2c : state==HIGH ? pixel[15:8] : pixel_hold[7:0];
 wire dc_bit=state==INIT ? token[8] : state!=FRAME;
 assign cs_n=!reset_n;
 tft_spi_byte #(.DIV(CLK_HZ/12500000>0 ? CLK_HZ/12500000 : 1)) u_spi(
 .clk(clk),.rst_n(rst_n && reset_n),.valid(send),.data(data),.dc_in(dc_bit),
 .ready(ready),.sck(sck),.mosi(mosi),.dc(dc));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin state<=RESET;delay_count<=CLK_HZ/50;reset_n<=0;index<=0;x<=0;y<=0;frame_start<=0;settle<=0;pixel_hold<=0;end
 else begin
  frame_start<=0;
  case(state)
   RESET: if(delay_count!=0)delay_count<=delay_count-1'b1;else begin reset_n<=1;delay_count<=(CLK_HZ/1000)*120;state<=RECOVER;end
   RECOVER:if(delay_count!=0)delay_count<=delay_count-1'b1;else state<=INIT;
   INIT:if(token[9])begin if(ready)begin delay_count<=(CLK_HZ/1000)*120;state<=SLEEP;end end
        else if(ready)begin if(index==INIT_BYTES-1)begin state<=FRAME;frame_start<=1;end else index<=index+1'b1;end
   SLEEP:if(delay_count!=0)delay_count<=delay_count-1'b1;else begin index<=index+1'b1;state<=INIT;end
   FRAME:if(ready)begin state<=SETTLE;settle<=0;end
   SETTLE:if(settle==2)state<=HIGH;else settle<=settle+1'b1;
   HIGH:if(ready)begin pixel_hold<=pixel;state<=LOW;end
   LOW:if(ready)begin
     if(x==319)begin x<=0;if(y==239)begin y<=0;state<=FRAME;frame_start<=1;end else begin y<=y+1'b1;state<=SETTLE;settle<=0;end end
     else begin x<=x+1'b1;state<=SETTLE;settle<=0;end
   end
   default:state<=RESET;
  endcase
 end
endmodule
`default_nettype wire

`default_nettype none
module tft_spi_display #(parameter integer CLK_HZ=50000000,INIT_BYTES=92,
 SPI_DIV=(CLK_HZ/12500000>0 ? CLK_HZ/12500000 : 1),REGION_MODE=0)(
 input wire clk,rst_n,input wire [15:0] pixel,
 output wire [8:0] x,output wire [7:0] y,output wire frame_start,
 output wire sck,mosi,dc,reset_n,cs_n,
 input wire window_valid,input wire [8:0] window_x0,window_x1,
 input wire [7:0] window_y0,window_y1,
 output wire window_ready,window_started,window_done);
 generate if(REGION_MODE)begin:regions
  tft_spi_regions #(.CLK_HZ(CLK_HZ),.INIT_BYTES(INIT_BYTES),.SPI_DIV(SPI_DIV)) engine(
   .clk(clk),.rst_n(rst_n),.pixel(pixel),.x(x),.y(y),.window_valid(window_valid),
   .window_x0(window_x0),.window_x1(window_x1),.window_y0(window_y0),.window_y1(window_y1),
   .window_ready(window_ready),.window_started(window_started),.window_done(window_done),
   .sck(sck),.mosi(mosi),.dc(dc),.reset_n(reset_n),.cs_n(cs_n));
  assign frame_start=window_started;
 end else begin:legacy
  tft_spi_display_legacy #(.CLK_HZ(CLK_HZ),.INIT_BYTES(INIT_BYTES),.SPI_DIV(SPI_DIV)) engine(
   .clk(clk),.rst_n(rst_n),.pixel(pixel),.x(x),.y(y),.frame_start(frame_start),
   .sck(sck),.mosi(mosi),.dc(dc),.reset_n(reset_n),.cs_n(cs_n));
  assign window_ready=0;assign window_started=frame_start;assign window_done=0;
 end endgenerate
endmodule
`default_nettype wire
