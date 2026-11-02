// 串口组帧发送：依次输出帧头、类型、长度、载荷与异或校验。
// 数据帧在帧握手时锁存，每次字节握手消费一个字节并累计校验。
`default_nettype none

module uart_packet_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer MAX_PAYLOAD_BYTES = 8
) (
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire                             frame_valid,
    output wire                             frame_ready,
    input  wire [7:0]                       frame_type,
    input  wire [7:0]                       frame_length,
    input  wire [8*MAX_PAYLOAD_BYTES-1:0]   frame_payload,
    output wire                             uart_txd
);
    reg busy;
    reg [7:0] byte_index;
    reg [7:0] saved_type;
    reg [7:0] saved_length;
    reg [7:0] saved_checksum;
    reg [8*MAX_PAYLOAD_BYTES-1:0] saved_payload;
    reg [7:0] tx_byte;
    wire tx_ready;

    reg preparing;
    assign frame_ready = !busy && !preparing && (frame_length <= MAX_PAYLOAD_BYTES);

    always @* begin
        case (byte_index)
            8'd0: tx_byte = 8'hA5;
            8'd1: tx_byte = 8'h5A;
            8'd2: tx_byte = saved_type;
            8'd3: tx_byte = saved_length;
            default: begin
                if (byte_index < saved_length + 8'd4)
                    tx_byte = saved_payload[7:0];
                else
                    tx_byte = saved_checksum;
            end
        endcase
    end

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart_tx (
        .clk(clk), .rst_n(rst_n),
        .in_data(tx_byte), .in_valid(busy), .in_ready(tx_ready),
        .txd(uart_txd)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0;
            preparing <= 1'b0;
            byte_index <= 0;
            saved_type <= 0;
            saved_length <= 0;
            saved_payload <= 0;
            saved_checksum <= 0;
        end else begin
            if (frame_valid && frame_ready) begin
                saved_type <= frame_type;
                saved_length <= frame_length;
                saved_checksum <= frame_type ^ frame_length;
                saved_payload <= frame_payload;
                byte_index <= 0;
                preparing <= 1'b1;
            end else if (preparing) begin
                preparing <= 1'b0;
                busy <= 1'b1;
            end else if (busy && tx_ready) begin

                if (byte_index >= 4 && byte_index < saved_length + 8'd4) begin
                    saved_checksum <= saved_checksum ^ saved_payload[7:0];
                    saved_payload <= saved_payload >> 8;
                end
                if (byte_index == saved_length + 8'd4)
                    busy <= 1'b0;
                else
                    byte_index <= byte_index + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
