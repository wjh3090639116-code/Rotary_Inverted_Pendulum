// 串口命令解析：查找帧头、检查长度并计算异或校验，完整有效帧才产生更新脉冲。
// 参数检查与执行条件由顶层处理；载荷按低字节在前的顺序解释。
`default_nettype none

module uart_config_rx #(
    parameter integer CLK_HZ=50000000, BAUD=115200
)(
    input wire clk, rst_n, uart_rxd,
    output reg [239:0] payload,
    output reg valid,
    output reg [159:0] swing_payload,
    output reg [7:0] swing_length,
    output reg swing_valid,
    output reg [15:0] period_payload,
    output reg period_valid,
    output reg [15:0] outer_period_payload,
    output reg outer_period_valid,
    output reg [31:0] wrap_payload,
    output reg wrap_valid,
    output reg [15:0] tilt_payload,
    output reg tilt_valid,
    output reg [15:0] deadzone_payload,
    output reg deadzone_valid,
    output reg [95:0] live_pid_payload,
    output reg live_pid_valid,
    output reg [63:0] live_pd_payload,
    output reg live_pd_valid,
    output reg [15:0] pwm_limit_payload,
    output reg pwm_limit_valid,
    output reg [23:0] tuning_payload,
    output reg tuning_valid,
    output reg [7:0] response_payload,
    output reg response_valid,
    output reg [63:0] trim_payload,
    output reg trim_valid,monitor_valid,monitor_enable,
    output reg [127:0] board_payload,
    output reg board_valid,board_extended,board_triplet,
    output reg reciprocal_valid,reciprocal_enable,
    output reg target_has_tilt,
    output reg [15:0] target_tilt_payload,
    output reg query_valid,
    output reg signed [31:0] target_payload,
    output reg target_valid,
    output reg run_command,
    output reg run_manual,
    output reg run_detach,
    output reg run_valid
);
    wire [7:0] rx_data;
    wire rx_valid;
    uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_rx (
        .clk(clk),.rst_n(rst_n),.rxd(uart_rxd),
        .byte_data(rx_data),.byte_valid(rx_valid)
    );
    localparam [2:0] HEAD1=0, HEAD2=1, TYPE=2, LENGTH=3, DATA=4, CHECK=5;
    reg [2:0] state;
    reg [7:0] frame_type, frame_length, checksum;
    reg [4:0] index;
    reg [239:0] receiving;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=HEAD1; frame_type<=0; frame_length<=0;
            index<=0; checksum<=0; receiving<=0;
            payload<=0; valid<=0; swing_payload<=0; swing_length<=16; swing_valid<=0;
            period_payload<=0; period_valid<=0; outer_period_payload<=0; outer_period_valid<=0;
            wrap_payload<=0; wrap_valid<=0; tilt_payload<=0; tilt_valid<=0;
            deadzone_payload<=0; deadzone_valid<=0;live_pid_payload<=0;live_pid_valid<=0;live_pd_payload<=0;live_pd_valid<=0;
            pwm_limit_payload<=0;pwm_limit_valid<=0;tuning_payload<=0;tuning_valid<=0;response_payload<=0;response_valid<=0;trim_payload<=0;trim_valid<=0;monitor_valid<=0;monitor_enable<=0;board_payload<=0;board_valid<=0;board_extended<=0;board_triplet<=0;reciprocal_valid<=0;reciprocal_enable<=0;target_has_tilt<=0;target_tilt_payload<=0;run_command<=0; run_manual<=0;run_detach<=0; run_valid<=0; query_valid<=0; target_payload<=0; target_valid<=0;
        end else begin
            pwm_limit_valid<=0;tuning_valid<=0;response_valid<=0;trim_valid<=0;monitor_valid<=0;board_valid<=0;reciprocal_valid<=0;valid<=0; swing_valid<=0; period_valid<=0; outer_period_valid<=0; wrap_valid<=0; tilt_valid<=0;
            run_valid<=0; query_valid<=0; target_valid<=0; deadzone_valid<=0;live_pid_valid<=0;live_pd_valid<=0;
            if (rx_valid) begin
                case (state)
                    HEAD1: if (rx_data==8'hA5) state<=HEAD2;
                    HEAD2: begin
                        if (rx_data==8'h5A) state<=TYPE;
                        else if (rx_data!=8'hA5) state<=HEAD1;
                    end
                    TYPE: begin
                        if (rx_data==8'h48 || rx_data==8'h46 || rx_data==8'h44 || rx_data==8'h37 || rx_data==8'h3a || rx_data==8'h40 || rx_data==8'h42 || rx_data==8'h32 || rx_data==8'h1b || rx_data==8'h1f || rx_data==8'h12 ||
                            rx_data==8'h13 || rx_data==8'h14 ||
                            rx_data==8'h26 || rx_data==8'h28 || rx_data==8'h2a || rx_data==8'h2c || rx_data==8'h2e || rx_data==8'h16 || rx_data==8'h17) begin
                            frame_type<=rx_data; checksum<=rx_data; state<=LENGTH;
                        end else if (rx_data==8'hA5) state<=HEAD2;
                        else state<=HEAD1;
                    end
                    LENGTH: begin
                        if ((frame_type==8'h48 && rx_data==8'd2) || (frame_type==8'h46 && rx_data==8'd3) || (frame_type==8'h44 && rx_data==8'd1) || (frame_type==8'h37 && rx_data==8'd8) || (frame_type==8'h3a && rx_data==8'd1) || (frame_type==8'h40 && (rx_data==8'd10 || rx_data==8'd14 || rx_data==8'd16)) || (frame_type==8'h42 && rx_data==8'd1) || (frame_type==8'h32 && rx_data==8'd30) ||
                            (frame_type==8'h1b && (rx_data==8'd4 || rx_data==8'd6)) ||
                            (frame_type==8'h1f && rx_data==8'd2) ||
                            (frame_type==8'h12 && rx_data==8'd1) ||
                            (frame_type==8'h13 && rx_data==8'd2) ||
                            (frame_type==8'h14 && rx_data==8'd4) ||
                            (frame_type==8'h26 && rx_data==8'd16) ||
                            (frame_type==8'h28 && rx_data==8'd20) ||
                            (frame_type==8'h16 && rx_data==8'd2) ||
                            (frame_type==8'h2a && rx_data==8'd2) ||
                            (frame_type==8'h2c && rx_data==8'd12) ||
                            (frame_type==8'h2e && rx_data==8'd8) ||
                            (frame_type==8'h17 && rx_data==8'd0)) begin
                            frame_length<=rx_data; checksum<=checksum^rx_data;
                            index<=0; receiving<=0;
                            state<=rx_data==0 ? CHECK : DATA;
                        end else if (rx_data==8'hA5) state<=HEAD2;
                        else state<=HEAD1;
                    end
                    DATA: begin
                        receiving[8*index +: 8]<=rx_data;
                        checksum<=checksum^rx_data;
                        if (index==frame_length-1'b1) state<=CHECK;
                        else index<=index+1'b1;
                    end
                    CHECK: begin
                        if (rx_data==checksum) begin
                            case (frame_type)
                                8'h48:begin pwm_limit_payload<=receiving[15:0];pwm_limit_valid<=1;end
                                8'h46:begin tuning_payload<=receiving[23:0];tuning_valid<=1;end
                                8'h44:begin response_payload<=receiving[7:0];response_valid<=1;end
                                8'h37:begin trim_payload<=receiving[63:0];trim_valid<=1;end
                                8'h3a:if(receiving[7:0]<=1)begin monitor_enable<=receiving[0];monitor_valid<=1;end
                                8'h40:begin board_payload<=receiving[127:0];board_valid<=1;board_extended<=frame_length>=14;board_triplet<=frame_length==16;end
                                8'h42:if(receiving[7:0]<=1)begin reciprocal_enable<=receiving[0];reciprocal_valid<=1;end
                                8'h32: begin payload<=receiving; valid<=1; end
                                8'h1f: begin outer_period_payload<=receiving[15:0]; outer_period_valid<=1; end
                                8'h1b: begin target_payload<=receiving[31:0]; target_valid<=1;target_has_tilt<=frame_length==6;target_tilt_payload<=receiving[47:32]; end
                                8'h26,8'h28: begin swing_payload<=receiving[159:0]; swing_length<=frame_length; swing_valid<=1; end
                                8'h16: begin tilt_payload<=receiving[15:0]; tilt_valid<=1; end
                                8'h2a: begin deadzone_payload<=receiving[15:0]; deadzone_valid<=1; end
                                8'h2c: begin live_pid_payload<=receiving[95:0]; live_pid_valid<=1; end
                                8'h2e: begin live_pd_payload<=receiving[63:0]; live_pd_valid<=1; end
                                8'h17: query_valid<=1;
                                8'h12: if (receiving[7:0]<=8'd3) begin
                                    run_command<=receiving[7:0]!=0;run_manual<=receiving[7:0]==2;run_detach<=receiving[7:0]==3;run_valid<=1;
                                end
                                8'h13: begin
                                    period_payload<=receiving[15:0]; period_valid<=1;
                                end
                                8'h14: begin
                                    wrap_payload<=receiving[31:0]; wrap_valid<=1;
                                end
                                default: ;
                            endcase
                        end
                        state<=HEAD1;
                    end
                    default: state<=HEAD1;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
