// 旋臂位置外环：根据位置误差和速度反馈生成摆杆目标倾角，不直接输出电机占空比。
// 增益采用１６位小数，速度采用每１０毫秒计数的８位小数表示；结果受倾角上限约束。
`default_nettype none

module position_pd_native(
 input wire clk,rst_n,enable,tick,
 input wire signed [17:0] position_error,
 input wire signed [31:0] velocity_q8,
 input wire signed [23:0] kp_q16,kd_q16,
 input wire [11:0] tilt_limit,
 output reg signed [12:0] command,
 output reg valid);
reg [2:0] stage;
reg signed [17:0] sampled_position;
reg signed [31:0] sampled_velocity;
reg signed [23:0] sampled_kp,sampled_kd;
reg [11:0] sampled_limit;
(* syn_keep = 1 *) reg signed [41:0] p_product;
reg signed [55:0] d_product;
reg signed [57:0] sum_q24;
wire signed [57:0] rounded=(sum_q24+(sum_q24[57] ? 58'sd8388607 : 58'sd8388608))>>>24;
wire signed [57:0] bound=$signed({46'd0,sampled_limit});
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin stage<=0;sampled_position<=0;sampled_velocity<=0;sampled_kp<=0;sampled_kd<=0;sampled_limit<=0;p_product<=0;d_product<=0;sum_q24<=0;command<=0;valid<=0;end
 else if(!enable)begin stage<=0;command<=0;valid<=0;end
 else begin valid<=0;case(stage)
 0:if(tick)begin sampled_position<=position_error;sampled_velocity<=velocity_q8;sampled_kp<=kp_q16;sampled_kd<=kd_q16;sampled_limit<=tilt_limit;stage<=1;end
 1:begin p_product<=$signed(sampled_position)*$signed(sampled_kp);d_product<=$signed(sampled_velocity)*$signed(sampled_kd);stage<=2;end
 2:begin sum_q24<=$signed({{8{p_product[41]}},p_product,8'd0})-$signed(d_product);stage<=3;end
 3:begin command<=rounded>bound ? $signed({1'b0,sampled_limit}) : rounded< -bound ? -$signed({1'b0,sampled_limit}) : rounded[12:0];valid<=1;stage<=0;end
 default:stage<=0;
 endcase end
end
endmodule
`default_nettype wire
