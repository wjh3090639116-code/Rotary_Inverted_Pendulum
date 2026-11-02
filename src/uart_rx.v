// 异步串口接收：采用８位数据、无校验、１位停止位格式。
// 输入经过两级同步，在各数据位中间采样；字节有效标志只保持一个时钟周期。
`default_nettype none

module uart_rx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire rxd,
    output reg [7:0] byte_data,
    output reg byte_valid
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer TIMER_WIDTH = CLKS_PER_BIT > 1 ? $clog2(CLKS_PER_BIT) : 1;
    localparam [1:0] IDLE=2'd0, START=2'd1, DATA=2'd2, STOP=2'd3;
    reg [1:0] sync_bits;
    reg [1:0] state;
    reg [TIMER_WIDTH-1:0] timer;
    reg [2:0] bit_index;
    reg [7:0] shift_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_bits <= 2'b11;
            state <= IDLE;
            timer <= 32'd0;
            bit_index <= 3'd0;
            shift_data <= 8'd0;
            byte_data <= 8'd0;
            byte_valid <= 1'b0;
        end else begin
            sync_bits <= {sync_bits[0], rxd};
            byte_valid <= 1'b0;
            case (state)
                IDLE: if (!sync_bits[1]) begin
                    timer <= CLKS_PER_BIT / 2 - 1;
                    state <= START;
                end
                START: if (timer == 0) begin
                    if (!sync_bits[1]) begin
                        timer <= CLKS_PER_BIT - 1;
                        bit_index <= 3'd0;
                        state <= DATA;
                    end else state <= IDLE;
                end else timer <= timer - 1'b1;
                DATA: if (timer == 0) begin
                    shift_data[bit_index] <= sync_bits[1];
                    timer <= CLKS_PER_BIT - 1;
                    if (bit_index == 3'd7)
                        state <= STOP;
                    else
                        bit_index <= bit_index + 1'b1;
                end else timer <= timer - 1'b1;
                STOP: if (timer == 0) begin
                    if (sync_bits[1]) begin
                        byte_data <= shift_data;
                        byte_valid <= 1'b1;
                    end
                    state <= IDLE;
                end else timer <= timer - 1'b1;
                default: state <= IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
