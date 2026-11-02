// 电机驱动接口：控制量符号决定方向，绝对值决定脉宽调制占空比。
// 在脉宽调制周期边界锁存指令，失能时立即关闭待机使能及驱动输出。
`default_nettype none

module tb6612_pwm #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_HZ = 20000,
    parameter integer DUTY_MAX = 1000
)(
    input wire clk, input wire rst_n, input wire enable,
    input wire signed [15:0] command,
    output wire in1, output wire in2, output wire pwm, output wire stby
);
    localparam integer PERIOD = CLK_HZ/PWM_HZ;
    reg [15:0] count;
    reg enable_d;

    reg signed [15:0] latched;
    wire [15:0] mag = latched[15] ? -latched : latched;
    wire [31:0] threshold = (mag > DUTY_MAX ? DUTY_MAX : mag) * PERIOD / DUTY_MAX;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin count <= 0; latched <= 0; enable_d <= 0; end
        else begin
            enable_d <= enable;
            if (!enable_d) begin count <= 0; latched <= 0; end
            else if (count == PERIOD-1) begin count <= 0; latched <= command; end
            else count <= count + 1'b1;
        end
    end

    assign stby = rst_n && enable && enable_d;
    assign in1 = stby && (latched > 0);
    assign in2 = stby && (latched < 0);
    assign pwm = stby && (count < threshold) && (latched != 0);
endmodule
`default_nettype wire
