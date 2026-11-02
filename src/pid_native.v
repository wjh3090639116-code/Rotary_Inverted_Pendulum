// 摆杆直立内环：误差和测量值均使用高１２位角度码，控制周期由毫秒节拍触发。
// 比例、微分增益采用８位小数，积分增益采用１２位小数；分阶段完成乘加和输出限幅。
// 微分项根据测量变化计算并滤波；积分仅在未饱和或有助于退出饱和时更新。
`default_nettype none

module pid_native #(parameter integer I_LIMIT=2000)(
 input wire clk,rst_n,tick,enable,
 input wire signed [12:0] error_in,measurement,
 input wire signed [23:0] kp_q8,ki_q12,kd_q8,
 input wire [15:0] output_limit,
 output reg signed [15:0] command);
reg signed [23:0] integral,candidate_integral;
reg signed [12:0] previous_measurement,sampled_measurement,sampled_error;
reg signed [23:0] sampled_kp,sampled_ki,sampled_kd;
reg signed [22:0] d_filtered_q8,candidate_d_q8;
reg signed [55:0] p_term,i_term,d_term,pi_q16,sum_q16;
reg [2:0] stage;
// 积分候选值先作独立限幅，微分项根据测量变化计算以减轻目标突变影响。
wire signed [24:0] integral_sum=$signed(integral)+$signed(sampled_error);
wire signed [23:0] integral_next=integral_sum>I_LIMIT ? I_LIMIT : integral_sum< -I_LIMIT ? -I_LIMIT : integral_sum[23:0];
wire signed [13:0] delta=$signed(previous_measurement)-$signed(sampled_measurement);
wire signed [23:0] d_difference=$signed({delta,8'd0})-$signed(d_filtered_q8);
wire signed [23:0] d_increment=d_difference[23] ? -(((-d_difference)+24'sd3)>>>2) : (d_difference+24'sd3)>>>2;
wire signed [23:0] d_next=$signed(d_filtered_q8)+d_increment;
wire signed [55:0] raw=(sum_q16+(sum_q16[55] ? 56'sd65535 : 56'sd0))>>>16;
wire signed [55:0] bound=$signed({40'd0,output_limit});
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin integral<=0;candidate_integral<=0;previous_measurement<=0;sampled_measurement<=0;sampled_error<=0;sampled_kp<=0;sampled_ki<=0;sampled_kd<=0;d_filtered_q8<=0;candidate_d_q8<=0;p_term<=0;i_term<=0;d_term<=0;pi_q16<=0;sum_q16<=0;stage<=0;command<=0;end
 else if(!enable)begin integral<=0;previous_measurement<=measurement;d_filtered_q8<=0;stage<=0;command<=0;end
 else case(stage)
 0:if(tick)begin sampled_error<=error_in;sampled_measurement<=measurement;sampled_kp<=kp_q8;sampled_ki<=ki_q12;sampled_kd<=kd_q8;stage<=1;end
 1:begin candidate_integral<=sampled_ki==0 ? 24'sd0 : integral_next;candidate_d_q8<=d_next[22:0];previous_measurement<=sampled_measurement;d_filtered_q8<=d_next[22:0];stage<=2;end
 2:begin p_term<=$signed(sampled_kp)*$signed(sampled_error);i_term<=$signed(sampled_ki)*$signed(candidate_integral);d_term<=$signed(sampled_kd)*$signed(candidate_d_q8);stage<=3;end
 3:begin pi_q16<=(p_term<<<8)+(i_term<<<4);stage<=4;end
 4:begin sum_q16<=pi_q16+d_term;stage<=5;end
 // 末级完成输出饱和及抗积分饱和判定，再返回等待下一控制节拍。
 5:begin
   command<=raw>bound ? $signed(output_limit) : raw< -bound ? -$signed(output_limit) : raw[15:0];
   if(sampled_ki==0 || (raw<=bound && raw>=-bound) || (raw>bound && (sampled_ki[23]^sampled_error[12])) || (raw< -bound && !(sampled_ki[23]^sampled_error[12])))integral<=candidate_integral;
   stage<=0;
 end
 default:stage<=0;
 endcase
end
endmodule
`default_nettype wire
