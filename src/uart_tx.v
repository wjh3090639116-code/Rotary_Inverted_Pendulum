// 异步串口发送：采用８位数据、无校验、１位停止位格式。
// 有效与就绪同时成立时接收一个字节，并按起始位、数据位、停止位顺序发送。
`default_nettype none

module uart_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] in_data,
    input  wire       in_valid,
    output wire       in_ready,
    output reg        txd
);

    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer TIMER_WIDTH = CLKS_PER_BIT > 1 ? $clog2(CLKS_PER_BIT) : 1;
    localparam [1:0] IDLE  = 2'd0,
                     START = 2'd1,
                     DATA  = 2'd2,
                     STOP  = 2'd3;
    reg [1:0]  state;
    reg [TIMER_WIDTH-1:0] bit_count;
    reg [2:0]  bit_index;
    reg [7:0]  shift_data;

    assign in_ready = (state == IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_count <= 0;
            bit_index <= 0;
            shift_data <= 0;
            txd <= 1'b1;
        end else begin
            case (state)
                IDLE: begin
                    txd <= 1'b1;
                    if (in_valid) begin
                        shift_data <= in_data;
                        bit_count <= 0;
                        bit_index <= 0;
                        txd <= 1'b0;
                        state <= START;
                    end
                end
                START: begin
                    if (bit_count == CLKS_PER_BIT - 1) begin
                        bit_count <= 0;
                        txd <= shift_data[0];
                        state <= DATA;
                    end else bit_count <= bit_count + 1'b1;
                end
                DATA: begin
                    if (bit_count == CLKS_PER_BIT - 1) begin
                        bit_count <= 0;
                        if (bit_index == 3'd7) begin
                            txd <= 1'b1;
                            state <= STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            txd <= shift_data[bit_index + 1'b1];
                        end
                    end else bit_count <= bit_count + 1'b1;
                end
                STOP: begin
                    if (bit_count == CLKS_PER_BIT - 1) begin
                        bit_count <= 0;
                        state <= IDLE;
                    end else bit_count <= bit_count + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
