// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`default_nettype none
module forward_turn_reference_baseline #(parameter integer COUNTS_PER_REV=1040)(
 input wire clk,rst_n,ms_tick,enable,start,cancel,
 input wire signed [31:0] goal,actual,
 output wire signed [31:0] reference,output reg ramping);
 reg signed [24:0] reference_q8;
 reg [4:0] elapsed_ms;
 wire signed [24:0] goal_q8={goal[16:0],8'd0};
 wire signed [24:0] remaining=goal_q8-reference_q8;
 localparam [24:0] D90=(90*COUNTS_PER_REV*256)/360,D30=(30*COUNTS_PER_REV*256)/360,D10=(10*COUNTS_PER_REV*256)/360;
 localparam [24:0] V120=(120*COUNTS_PER_REV*256)/18000,V80=(80*COUNTS_PER_REV*256)/18000,
                         V40=(40*COUNTS_PER_REV*256)/18000,V20=(20*COUNTS_PER_REV*256)/18000;
 localparam signed [25:0] LEAD=(60*COUNTS_PER_REV*256)/360;
 wire [24:0] speed_step=remaining>D90 ? V120 : remaining>D30 ? V80 : remaining>D10 ? V40 : V20;
 wire signed [25:0] lead_room=$signed({actual[16],actual[16:0],8'd0})+LEAD-$signed({reference_q8[24],reference_q8});
 wire [24:0] allowed_step=lead_room<=0 ? 25'd0 : lead_room<$signed({1'b0,speed_step}) ? lead_room[24:0] : speed_step;
 wire actual_fits=actual[31:17]=={15{actual[16]}};
 assign reference=ramping ? ($signed(reference_q8)>>>8) : goal;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin reference_q8<=0;elapsed_ms<=0;ramping<=0;end
 else if(!enable || cancel)begin reference_q8<=goal_q8;elapsed_ms<=0;ramping<=0;end
 else if(start)begin
   if(!ramping)reference_q8<=goal_q8;
   elapsed_ms<=0;ramping<=1;
 end else if(ramping && ms_tick)begin
   if(elapsed_ms==19)begin
     elapsed_ms<=0;
     if(remaining<=0 || (actual_fits && remaining<=$signed(allowed_step)))begin reference_q8<=goal_q8;ramping<=0;end
     else if(actual_fits)reference_q8<=reference_q8+$signed(allowed_step);
   end else elapsed_ms<=elapsed_ms+1'b1;
 end
endmodule
`default_nettype wire
