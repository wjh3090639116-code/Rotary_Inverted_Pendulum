// 角度采集接口：仅将转换结果的高１２位送入角度处理链路。
// 低４位不参与滤波和闭环计算；采集故障与有效状态独立输出。
`default_nettype none

module ads8688_angle_if #(
 parameter integer CLK_HZ=50000000,SPI_HZ=1000000,CHANNEL=0,
 parameter integer BOOT_CYCLES=CLK_HZ/50,WARMUP_CYCLES=CLK_HZ/50
)(input wire clk,rst_n,
 output wire adc_cs_n,adc_sclk,adc_sdi,input wire adc_sdo,
 output reg [15:0] raw16,output reg [11:0] sample_code,
 output reg sample_strobe,link_valid,output wire [7:0] error_code);
localparam integer STALE_CYCLES=CLK_HZ/250;
wire [15:0] conversion;wire fresh,ready;
reg [2:0] good_count;reg [31:0] age;
ads8688_checked_if #(.CLK_HZ(CLK_HZ),.SPI_HZ(SPI_HZ),.CHANNEL(CHANNEL),
 .BOOT_CYCLES(BOOT_CYCLES),.WARMUP_CYCLES(WARMUP_CYCLES)) adc(
 .clk(clk),.rst_n(rst_n),.adc_cs_n(adc_cs_n),.adc_sclk(adc_sclk),
 .adc_sdi(adc_sdi),.adc_sdo(adc_sdo),.sample_data(conversion),
 .sample_valid(fresh),.ready(ready),.error_code(error_code));
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  raw16<=0;sample_code<=0;sample_strobe<=0;link_valid<=0;good_count<=0;age<=STALE_CYCLES;
 end else begin
  sample_strobe<=0;
  if(age<STALE_CYCLES)age<=age+1'b1;
  if(!ready || error_code!=0 || age>=STALE_CYCLES)begin link_valid<=0;good_count<=0;end
  if(fresh && ready && error_code==0)begin
   raw16<=conversion;sample_code<=conversion[15:4];
   sample_strobe<=1;age<=0;
   if(good_count<4)good_count<=good_count+1'b1;
   if(good_count>=3)link_valid<=1;
  end
 end
end
endmodule
`default_nettype wire
