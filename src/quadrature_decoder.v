// 正交编码器译码：同步两相输入，依据相邻状态转移累计带符号位置。
// 非法跳变单独计数，不作为正常位置增量。
`default_nettype none

module quadrature_decoder(
    input wire clk, input wire rst_n,
    input wire enc_a, input wire enc_b,
    output reg signed [31:0] position,
    output reg invalid_transition
);

    reg [1:0] a_sync, b_sync;
    reg [1:0] prev;
    wire [1:0] curr = {a_sync[1],b_sync[1]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 0; b_sync <= 0; prev <= 0;
            position <= 0; invalid_transition <= 0;
        end else begin
            a_sync <= {a_sync[0],enc_a};
            b_sync <= {b_sync[0],enc_b};

            invalid_transition <= 0;
            if (curr != prev) begin

                case ({prev,curr})
                    4'b0001,4'b0111,4'b1110,4'b1000: position <= position + 1'sb1;
                    4'b0010,4'b1011,4'b1101,4'b0100: position <= position - 1'sb1;
                    default: invalid_transition <= 1;
                endcase
                prev <= curr;
            end
        end
    end
endmodule
`default_nettype wire
