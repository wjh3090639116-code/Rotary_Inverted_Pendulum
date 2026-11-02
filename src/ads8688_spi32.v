// 模数转换器串行时序：每帧传输３２位，时钟空闲为低电平。
// 输入数据在下降沿前保持稳定；接收帧低１６位保存转换结果。
`default_nettype none

module ads8688_spi32 #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SPI_HZ = 1000000
) (
    input wire clk, rst_n, start,
    input wire [31:0] tx_data,
    output reg busy, done,
    output reg [31:0] rx_data,
    output reg cs_n, sclk, sdi,
    input wire sdo
);
    localparam integer HALF_DIV = CLK_HZ / (2 * SPI_HZ);
    localparam integer CW = (HALF_DIV < 2) ? 1 : $clog2(HALF_DIV);
    localparam IDLE=0, SHIFT=1, HOLD=2, GAP=3;
    reg [1:0] state;
    reg [CW-1:0] count;
    reg [5:0] bit_index;
    reg [31:0] tx_saved, rx_shift;

    (* syn_preserve=1 *) reg sdo_meta, sdo_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sdo_meta<=0; sdo_sync<=0; end
        else begin sdo_meta<=sdo; sdo_sync<=sdo_meta; end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; count<=0; bit_index<=0; tx_saved<=0; rx_shift<=0;
            rx_data<=0; busy<=0; done<=0; cs_n<=1; sclk<=0; sdi<=0;
        end else begin
            done<=0;
            case (state)
                IDLE: if (start) begin
                    busy<=1; cs_n<=0; sclk<=0; sdi<=0;
                    tx_saved<=tx_data; rx_shift<=0; bit_index<=0;
                    count<=0; state<=SHIFT;
                end
                SHIFT: if (count == HALF_DIV-1) begin
                    count<=0;
                    if (!sclk) begin
                        sclk<=1;
                        sdi<=tx_saved[31]; tx_saved<={tx_saved[30:0],1'b0};
                    end else begin
                        sclk<=0; rx_shift<={rx_shift[30:0],sdo_sync};
                        if (bit_index==31) begin
                            rx_data<={rx_shift[30:0],sdo_sync}; state<=HOLD;
                        end else bit_index<=bit_index+1'b1;
                    end
                end else count<=count+1'b1;
                HOLD: if (count==HALF_DIV-1) begin
                    cs_n<=1; sdi<=0; done<=1; count<=0; state<=GAP;
                end else count<=count+1'b1;
                GAP: if (count==HALF_DIV-1) begin
                    count<=0; busy<=0; state<=IDLE;
                end else count<=count+1'b1;
                default: begin state<=IDLE; cs_n<=1; sclk<=0; busy<=0; end
            endcase
        end
    end
endmodule
`default_nettype wire
