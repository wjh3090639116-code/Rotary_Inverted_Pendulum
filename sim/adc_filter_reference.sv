// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`default_nettype none

module adc_filter_reference #(
    parameter integer CLK_HZ=50000000,
    parameter integer MIN_CUTOFF_HZ=200,
    parameter integer BETA_MILLI=50,
    parameter integer DERIVATIVE_CUTOFF_HZ=50
)(
    input wire clk, rst_n, restart, sample_strobe,
    input wire [11:0] sample_code, full_span, half_span,
    output reg [11:0] filtered_code, raw_code,
    output reg output_strobe, output_valid
);
localparam integer D_ALPHA=(64'd65536*6283*DERIVATIVE_CUTOFF_HZ)/
    (1000000+6283*DERIVATIVE_CUTOFF_HZ);
localparam [3:0] IDLE=0, DIFFERENCE=1, D_MULTIPLY=2, D_UPDATE=3,
    BETA_MULTIPLY=4, BETA_DIVIDE=5, CUTOFF=6, ALPHA_START=7,
    ALPHA_DIVIDE=8, X_MULTIPLY=9, X_UPDATE=10, PUBLISH=11, D_ABSOLUTE=12;
reg [3:0] state;
reg primed;
reg [11:0] input_latched, span_latched;
reg signed [24:0] x_q8, x_delta;
reg signed [26:0] derivative_q8, derivative_difference;
reg [26:0] derivative_magnitude_registered;
reg signed [44:0] derivative_product;
reg signed [42:0] x_product;
reg [47:0] quotient;
reg [32:0] remainder;
reg [31:0] divisor;
reg [5:0] bits_left;
reg [15:0] cutoff_hz;
reg [16:0] alpha_q16;
reg [7:0] publication_delay;
reg [31:0] sample_age;
reg restart_registered;
localparam integer GAP_CYCLES=(CLK_HZ/1000)*3/2;
wire [16:0] cutoff_candidate=MIN_CUTOFF_HZ+quotient[15:0];
wire [24:0] rounded_x=$unsigned(x_q8)+25'd128;
wire signed [16:0] step=$signed({1'b0,sample_code})-$signed({1'b0,filtered_code});
wire seam=(step>$signed({1'b0,half_span}) || step<-$signed({1'b0,half_span}));
wire signed [44:0] d_increment=derivative_product[44] ?
    -(((-derivative_product)+45'sd32768)>>>16) : (derivative_product+45'sd32768)>>>16;
wire signed [45:0] derivative_next=$signed(derivative_q8)+d_increment;
wire [26:0] derivative_magnitude=derivative_q8[26] ? -derivative_q8 : derivative_q8;
wire [32:0] trial={remainder[31:0],quotient[47]};
wire take_bit=trial>={1'b0,divisor};
wire signed [42:0] x_increment=x_product[42] ?
    -(((-x_product)+43'sd32768)>>>16) : (x_product+43'sd32768)>>>16;
wire signed [43:0] next_x=$signed(x_q8)+x_increment;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=IDLE; primed<=0; input_latched<=0; span_latched<=2640;
        x_q8<=0; x_delta<=0; derivative_q8<=0; derivative_difference<=0;
        derivative_magnitude_registered<=0;
        derivative_product<=0; x_product<=0; quotient<=0; remainder<=0;
        divisor<=1; bits_left<=0; cutoff_hz<=MIN_CUTOFF_HZ; alpha_q16<=0;
        publication_delay<=0;
        sample_age<=GAP_CYCLES;
        restart_registered<=1;
        filtered_code<=0; raw_code<=0; output_strobe<=0; output_valid<=0;
    end else begin
        output_strobe<=0;
        restart_registered<=restart;
        if(sample_strobe) sample_age<=0;
        else if(sample_age<GAP_CYCLES) sample_age<=sample_age+1'b1;
        if(state!=IDLE && publication_delay<110) publication_delay<=publication_delay+1'b1;

        if(restart_registered) begin primed<=0; output_valid<=0; end
        case(state)
        IDLE: if(sample_strobe) begin
            publication_delay<=0;
            input_latched<=sample_code; span_latched<=full_span==0 ? 12'd2640 : full_span;
            if(!primed || restart_registered || seam || sample_age>=GAP_CYCLES) begin
                x_q8<=$signed({1'b0,sample_code,8'd0}); derivative_q8<=0;
                state<=PUBLISH;
            end else state<=DIFFERENCE;
        end
        DIFFERENCE: begin

            x_delta<=$signed({1'b0,input_latched,8'd0})-$signed(x_q8);
            derivative_difference<=($signed({1'b0,input_latched,8'd0})-$signed(x_q8))-$signed(derivative_q8);
            state<=D_MULTIPLY;
        end
        D_MULTIPLY: begin derivative_product<=$signed(derivative_difference)*D_ALPHA; state<=D_UPDATE; end
        D_UPDATE: begin derivative_q8<=derivative_next[26:0]; state<=D_ABSOLUTE; end
        D_ABSOLUTE: begin derivative_magnitude_registered<=derivative_magnitude; state<=BETA_MULTIPLY; end
        BETA_MULTIPLY: begin

            quotient<=derivative_magnitude_registered*48'd360*BETA_MILLI;
            divisor<={12'd0,span_latched,8'd0}; remainder<=0; bits_left<=48; state<=BETA_DIVIDE;
        end
        BETA_DIVIDE: begin
            quotient<={quotient[46:0],take_bit}; remainder<=take_bit ? trial-{1'b0,divisor} : trial;
            if(bits_left==1) state<=CUTOFF; else bits_left<=bits_left-1'b1;
        end
        CUTOFF: begin

            cutoff_hz<=quotient>5000-MIN_CUTOFF_HZ ? 16'd5000 : cutoff_candidate[15:0];
            state<=ALPHA_START;
        end
        ALPHA_START: begin

            quotient<=cutoff_hz*48'd411762688; divisor<=1000000+cutoff_hz*32'd6283;
            remainder<=0; bits_left<=48; state<=ALPHA_DIVIDE;
        end
        ALPHA_DIVIDE: begin
            quotient<={quotient[46:0],take_bit}; remainder<=take_bit ? trial-{1'b0,divisor} : trial;
            if(bits_left==1) state<=X_MULTIPLY; else bits_left<=bits_left-1'b1;
        end
        X_MULTIPLY: begin alpha_q16<=quotient[16:0]; x_product<=$signed(x_delta)*$signed({1'b0,quotient[16:0]}); state<=X_UPDATE; end
        X_UPDATE: begin x_q8<=next_x[24:0]; state<=PUBLISH; end
        PUBLISH: if(publication_delay>=110) begin
            raw_code<=input_latched;
            filtered_code<=rounded_x[19:8];
            output_strobe<=1; output_valid<=!restart_registered; primed<=!restart_registered; state<=IDLE;
        end
        default: begin state<=IDLE; primed<=0; output_valid<=0; end
        endcase
    end
end
endmodule
`default_nettype wire
