// 上电参数装载：从只读存储初始化数据依次发送配置命令。
// 初始化内容不包含启动运行命令，参数装载本身不会使电机进入运行状态。
`default_nettype none

module tft_profile_boot #(parameter integer CLK_HZ=50000000,BAUD=115200)(
 input wire clk,rst_n,output reg busy,output wire serial_out);
 reg [7:0] rom[0:191];
 initial $readmemh("src/tft_profile.hex",rom);
 reg [7:0] index;reg [31:0] guard;
 wire ready;wire sending=index<BYTE_COUNT;
 localparam integer BYTE_COUNT=129;
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_tx(.clk(clk),.rst_n(rst_n),
 .in_data(rom[index]),.in_valid(busy && guard==0 && sending),.in_ready(ready),.txd(serial_out));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin busy<=1;index<=0;guard<=CLK_HZ/100;end
 else if(busy)begin
  if(guard!=0)guard<=guard-1'b1;
  else if(sending)begin if(ready)index<=index+1'b1;end
  else if(ready)begin busy<=0;end
 end
endmodule
`default_nettype wire
