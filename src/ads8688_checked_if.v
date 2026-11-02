// 采样有效性管理：检查数据格式与量程配置，完成校验后恢复手动通道采样。
// 寄存器访问会改变转换状态，恢复帧的转换值不作为有效样本发布。
`default_nettype none

module ads8688_checked_if #(
 parameter integer CLK_HZ=50000000,SPI_HZ=1000000,SAMPLE_HZ=1000,CHANNEL=0,
 parameter [7:0] RANGE_CODE=8'h06,
 parameter integer BOOT_CYCLES=CLK_HZ/50,WARMUP_CYCLES=CLK_HZ/50
)(input wire clk,rst_n,
 output wire adc_cs_n,adc_sclk,adc_sdi,input wire adc_sdo,
 output reg [15:0] sample_data,output reg sample_valid,ready,
 output reg [7:0] error_code);
localparam integer PERIOD=CLK_HZ/SAMPLE_HZ;
localparam integer PW=PERIOD<2 ? 1 : $clog2(PERIOD);
localparam BOOT=0,ISSUE=1,WAIT_SPI=2,WARM=3,RUN=4;
localparam [15:0] RANGE_WRITE=((5+CHANNEL)<<9)|16'h0100|RANGE_CODE;
localparam [15:0] RANGE_READ=((5+CHANNEL)<<9),MANUAL=16'hC000|(CHANNEL<<10);
reg [2:0] state,step;
reg [31:0] wait_count;
reg [PW-1:0] period_count;
reg spi_start,runtime_check;
reg [15:0] command,pending_data;
wire busy,done;wire [31:0] received;
always @* case(step)
 0:command=16'h8500;1:command=16'h0700;2:command=16'h0600;
 3:command=RANGE_WRITE;4:command=RANGE_READ;5,7:command=MANUAL;
 default:command=16'h0000;
endcase
ads8688_spi32 #(.CLK_HZ(CLK_HZ),.SPI_HZ(SPI_HZ)) spi(
 .clk(clk),.rst_n(rst_n),.start(spi_start),.tx_data({command,16'd0}),
 .busy(busy),.done(done),.rx_data(received),.cs_n(adc_cs_n),
 .sclk(adc_sclk),.sdi(adc_sdi),.sdo(adc_sdo));
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  state<=BOOT;step<=0;wait_count<=0;period_count<=0;spi_start<=0;
  runtime_check<=0;pending_data<=0;sample_data<=0;sample_valid<=0;ready<=0;error_code<=0;
 end else begin
  spi_start<=0;sample_valid<=0;
  if(ready)period_count<=period_count==PERIOD-1 ? 0 : period_count+1'b1;
  if(ready && period_count==0 && state!=RUN)begin
   ready<=0;error_code<=3;wait_count<=0;state<=BOOT;runtime_check<=0;
  end else case(state)
   BOOT:begin
    ready<=0;runtime_check<=0;
    if(wait_count==BOOT_CYCLES-1)begin wait_count<=0;step<=0;state<=ISSUE;end
    else wait_count<=wait_count+1'b1;
   end
   ISSUE:if(!busy)begin spi_start<=1;state<=WAIT_SPI;end
   WAIT_SPI:if(done)begin
    if((step==2 && received[15:8]!=0)||(step==4 && received[15:8]!=RANGE_CODE))begin
     ready<=0;error_code<=step==2 ? 1 : 2;wait_count<=0;runtime_check<=0;state<=BOOT;
    end else if(runtime_check)case(step)
     6:begin pending_data<=received[15:0];step<=2;state<=ISSUE;end
     2:begin step<=4;state<=ISSUE;end
     4:begin step<=7;state<=ISSUE;end

     7:begin sample_data<=pending_data;sample_valid<=1;state<=RUN;end
     default:begin ready<=0;error_code<=3;wait_count<=0;state<=BOOT;end
    endcase
    else if(step==5)begin wait_count<=0;state<=WARM;end
    else begin step<=step+1'b1;state<=ISSUE;end
   end
   WARM:if(wait_count==WARMUP_CYCLES-1)begin
    wait_count<=0;period_count<=0;ready<=1;error_code<=0;state<=RUN;
   end else wait_count<=wait_count+1'b1;
   RUN:if(period_count==0)begin runtime_check<=1;step<=6;state<=ISSUE;end
   default:begin ready<=0;wait_count<=0;state<=BOOT;end
  endcase
 end
end
endmodule
`default_nettype wire
