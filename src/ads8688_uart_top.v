// 系统顶层：协调采集、起摆、双环控制、目标管理、触屏和串口。
// 停机、故障与参数接收决定驱动使能；目标命令经过运行状态和行程检查后生效。
`default_nettype none

module ads8688_uart_top #(
    parameter integer CLK_HZ=50000000, BAUD=115200,
    parameter integer PWM_HZ=20000, ENABLE_SCREEN=1, COUNTS_PER_REV=1040,
    parameter integer ADC_CHANNEL=0, ADC_SPI_HZ=1000000,
    parameter integer ADC_BOOT_CYCLES=CLK_HZ/50, ADC_WARMUP_CYCLES=CLK_HZ/50
)(
    input wire clk, rst_n, run_btn_n, step_down_btn_n, amplitude_up_btn_n, amplitude_down_btn_n,
    output wire adc_cs_n, adc_sclk, adc_sdi,
    input wire adc_sdo,
    input wire encoder_a, encoder_b, uart_rxd,
    output wire tft_sck,tft_mosi,tft_dc,tft_rst_n,tft_cs_n,
    input wire touch_irq_n,touch_miso,
    output wire touch_sck,touch_mosi,touch_cs_n,
    output wire uart_txd, motor_in1, motor_in2, motor_pwm, motor_stby
);
    localparam integer LOCAL_UI=ENABLE_SCREEN;
    wire ui_valid;wire [3:0] ui_action;wire [7:0] ui_status;
    wire [8:0] ui_step_degrees,ui_amplitude_degrees;
    reg free_mode;
    wire [11:0] ui_adc,ui_zero;reg [3:0] ui_notice;reg ui_notice_valid;
    wire signed [31:0] ui_position,ui_target;
    wire profile_busy,profile_serial;
    wire control_rx=profile_busy ? profile_serial : uart_rxd;
// 显示与上电配置子系统：初始化只写配置，运行操作通过公共仲裁入口提交。
    generate if(ENABLE_SCREEN)begin:screen
      tft_profile_boot #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_boot(
       .clk(clk),.rst_n(rst_n),.busy(profile_busy),.serial_out(profile_serial));
      tft_panel #(.CLK_HZ(CLK_HZ),.COUNTS_PER_REV(COUNTS_PER_REV)) u_panel(
       .clk(clk),.rst_n(rst_n),.position(ui_position),.target(ui_target),
       .notice(ui_notice),.notice_valid(ui_notice_valid),.status(ui_status),
       .adc(ui_adc),.zero(ui_zero),.step_degrees(ui_step_degrees),.amplitude_degrees(ui_amplitude_degrees),.boot_busy(profile_busy),.action_valid(ui_valid),.action(ui_action),
       .tft_sck(tft_sck),.tft_mosi(tft_mosi),.tft_dc(tft_dc),.tft_rst_n(tft_rst_n),.tft_cs_n(tft_cs_n),
       .touch_irq_n(touch_irq_n),.touch_miso(touch_miso),.touch_sck(touch_sck),.touch_mosi(touch_mosi),.touch_cs_n(touch_cs_n));
    end else begin:no_screen
      assign profile_busy=0;assign profile_serial=1;assign ui_valid=0;assign ui_action=0;
      assign tft_sck=0;assign tft_mosi=0;assign tft_dc=0;assign tft_rst_n=rst_n;assign tft_cs_n=1;
      assign touch_sck=0;assign touch_mosi=0;assign touch_cs_n=1;
    end endgenerate
    localparam integer CONTROL_DIV=CLK_HZ/1000;
    localparam integer DEFAULT_PERIOD_US=1000;
    localparam integer CYCLES_PER_US=CLK_HZ/1000000;
    localparam integer ADC_REPORT_DIV=CLK_HZ/50;
    localparam integer MOTOR_REPORT_DIV=CLK_HZ/200;
    localparam integer BUTTON_CYCLES=CLK_HZ/50;
    reg [19:0] control_count, control_limit;
    reg [31:0] adc_report_count, motor_report_count;
    reg [31:0] control_div;
    reg tick;
    reg adc_due, motor_due, config_due, swing_due, period_due, outer_period_due, wrap_due, tilt_due, target_due, deadzone_due, live_pid_due, live_pd_due;
    reg [11:0] wrap_low, wrap_high;
    reg [11:0] wrap_span, wrap_half_span;

    wire config_accept,wrap_accept;
    wire [11:0] adc_code, adc_unfiltered, adc_filter_raw;
    wire adc_link_valid, adc_bus_sample, filter_valid;
    wire adc_sample_pulse;
    wire adc_valid=adc_link_valid && filter_valid;
    wire [15:0] adc_raw16;
    wire [7:0] adc_error;
// 角度反馈链路：原始转换码经有效性管理和滤波后进入摆杆控制。
    ads8688_angle_if #(.CLK_HZ(CLK_HZ),.SPI_HZ(ADC_SPI_HZ),.CHANNEL(ADC_CHANNEL),
        .BOOT_CYCLES(ADC_BOOT_CYCLES),.WARMUP_CYCLES(ADC_WARMUP_CYCLES)) u_adc (
        .clk(clk),.rst_n(rst_n),.adc_cs_n(adc_cs_n),.adc_sclk(adc_sclk),
        .adc_sdi(adc_sdi),.adc_sdo(adc_sdo),.raw16(adc_raw16),
        .sample_code(adc_unfiltered),.sample_strobe(adc_bus_sample),
        .link_valid(adc_link_valid),.error_code(adc_error));
    adc_one_euro_filter #(.CLK_HZ(CLK_HZ)) u_adc_filter (
        .clk(clk),.rst_n(rst_n),.restart(!adc_link_valid || config_accept || wrap_accept),
        .sample_strobe(adc_bus_sample),.sample_code(adc_unfiltered),
        .full_span(wrap_span),.half_span(wrap_half_span),
        .filtered_code(adc_code),.raw_code(adc_filter_raw),
        .output_strobe(adc_sample_pulse),.output_valid(filter_valid));
    wire signed [31:0] encoder_position;
    wire encoder_invalid;
// 编码器位置反馈与非法跳变统计。
    quadrature_decoder u_encoder (
        .clk(clk),.rst_n(rst_n),.enc_a(encoder_a),.enc_b(encoder_b),
        .position(encoder_position),.invalid_transition(encoder_invalid)
    );
    reg [15:0] invalid_count;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) invalid_count<=0;
        else if (encoder_invalid) invalid_count<=invalid_count+1'b1;

    reg [1:0] button_sync;
    reg button_stable, button_previous, button_armed;
    reg [31:0] button_count;
    wire button_press=button_previous && !button_stable;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_sync<=0; button_stable<=0; button_previous<=0;
            button_armed<=0; button_count<=0;
        end else begin
            button_sync<={button_sync[0],run_btn_n};
            button_previous<=button_stable;
            if (button_sync[1]==button_stable) button_count<=0;
            else if (button_count==BUTTON_CYCLES-1) begin
                button_stable<=button_sync[1]; button_count<=0;
            end else button_count<=button_count+1'b1;
            if (button_stable) button_armed<=1;
        end
    end

    reg [1:0] step_down_sync;
    reg step_down_stable,step_down_previous,step_down_armed;
    reg [31:0] step_down_count;
    wire step_down_press=step_down_previous && !step_down_stable;
    reg [5:0] ui_step_units;
    assign ui_step_degrees=ui_step_units*9'd10;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin
        step_down_sync<=0;step_down_stable<=0;step_down_previous<=0;
        step_down_armed<=0;step_down_count<=0;ui_step_units<=6;
      end else begin
        step_down_sync<={step_down_sync[0],step_down_btn_n};
        step_down_previous<=step_down_stable;
        if(step_down_sync[1]==step_down_stable)step_down_count<=0;
        else if(step_down_count==BUTTON_CYCLES-1)begin
          step_down_stable<=step_down_sync[1];step_down_count<=0;
        end else step_down_count<=step_down_count+1'b1;
        if(step_down_stable)step_down_armed<=1;

        if(LOCAL_UI && !profile_busy)begin
          case({button_press && button_armed,step_down_press && step_down_armed})
            2'b10:if(ui_step_units<36)ui_step_units<=ui_step_units+1'b1;
            2'b01:if(ui_step_units>1)ui_step_units<=ui_step_units-1'b1;
            default:begin end
          endcase
        end
      end

    wire [239:0] config_rx, config_uart;
    wire local_event=LOCAL_UI && ui_valid;
    wire local_stop=local_event && ui_action==2;
    wire local_start=local_event && (ui_action==1 || ui_action==3);
    wire local_zero=local_event && ui_action==4 && !run_request && !config_rx_valid && !config_checked_valid && config_loaded && adc_valid && adc_code>=16 && adc_code<=4079;
    reg local_zero_hold;
    reg [239:0] local_zero_payload;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin local_zero_hold<=0;local_zero_payload<=0;end
      else begin local_zero_hold<=local_zero;if(local_zero)local_zero_payload<={config_echo[239:16],4'b0,adc_code};end
    assign config_rx=local_zero_hold ? local_zero_payload : local_zero ? {config_echo[239:16],4'b0,adc_code} : config_uart;
    assign ui_position=arm_position;
    assign ui_target=arm_target_full;
    assign ui_adc=adc_code;
    assign ui_zero=zero_code;
    assign ui_status={1'b0,free_mode,remote_run,state==WAIT_UPRIGHT,state==FAULT,state==ACTIVE,run_request,config_loaded && adc_valid};
    wire signed [31:0] arm_target_rx, arm_target_uart;
    wire signed [31:0] reciprocal_target;
    wire reciprocal_valid,reciprocal_amplitude_ok,reciprocal_canceled;
    wire manual_target_valid;
    reg target_from_reciprocal,target_from_center_return;
    reg center_return_pending,free_mode_previous;
    wire signed [31:0] panel_center;
    wire reciprocal_allowed;
    wire arm_target_uart_valid;
    wire local_target_action=ui_action>=5 && ui_action<=9;
    wire local_target=local_event && local_target_action && !run_rx_valid && !arm_target_uart_valid;

    reg signed [6:0] ui_residual,ui_fraction_hold;
    reg ui_target_pending;
    reg [3:0] ui_action_hold;
    function [17:0] step_whole;input [5:0] units;begin case(units)
      6'd1:step_whole=(COUNTS_PER_REV*1)/36;
      6'd2:step_whole=(COUNTS_PER_REV*2)/36;
      6'd3:step_whole=(COUNTS_PER_REV*3)/36;
      6'd4:step_whole=(COUNTS_PER_REV*4)/36;
      6'd5:step_whole=(COUNTS_PER_REV*5)/36;
      6'd6:step_whole=(COUNTS_PER_REV*6)/36;
      6'd7:step_whole=(COUNTS_PER_REV*7)/36;
      6'd8:step_whole=(COUNTS_PER_REV*8)/36;
      6'd9:step_whole=(COUNTS_PER_REV*9)/36;
      6'd10:step_whole=(COUNTS_PER_REV*10)/36;
      6'd11:step_whole=(COUNTS_PER_REV*11)/36;
      6'd12:step_whole=(COUNTS_PER_REV*12)/36;
      6'd13:step_whole=(COUNTS_PER_REV*13)/36;
      6'd14:step_whole=(COUNTS_PER_REV*14)/36;
      6'd15:step_whole=(COUNTS_PER_REV*15)/36;
      6'd16:step_whole=(COUNTS_PER_REV*16)/36;
      6'd17:step_whole=(COUNTS_PER_REV*17)/36;
      6'd18:step_whole=(COUNTS_PER_REV*18)/36;
      6'd19:step_whole=(COUNTS_PER_REV*19)/36;
      6'd20:step_whole=(COUNTS_PER_REV*20)/36;
      6'd21:step_whole=(COUNTS_PER_REV*21)/36;
      6'd22:step_whole=(COUNTS_PER_REV*22)/36;
      6'd23:step_whole=(COUNTS_PER_REV*23)/36;
      6'd24:step_whole=(COUNTS_PER_REV*24)/36;
      6'd25:step_whole=(COUNTS_PER_REV*25)/36;
      6'd26:step_whole=(COUNTS_PER_REV*26)/36;
      6'd27:step_whole=(COUNTS_PER_REV*27)/36;
      6'd28:step_whole=(COUNTS_PER_REV*28)/36;
      6'd29:step_whole=(COUNTS_PER_REV*29)/36;
      6'd30:step_whole=(COUNTS_PER_REV*30)/36;
      6'd31:step_whole=(COUNTS_PER_REV*31)/36;
      6'd32:step_whole=(COUNTS_PER_REV*32)/36;
      6'd33:step_whole=(COUNTS_PER_REV*33)/36;
      6'd34:step_whole=(COUNTS_PER_REV*34)/36;
      6'd35:step_whole=(COUNTS_PER_REV*35)/36;
      6'd36:step_whole=(COUNTS_PER_REV*36)/36;
      default:step_whole=0;endcase end endfunction
    function [17:0] step_remainder;input [5:0] units;begin case(units)
      6'd1:step_remainder=(COUNTS_PER_REV*1)%36;
      6'd2:step_remainder=(COUNTS_PER_REV*2)%36;
      6'd3:step_remainder=(COUNTS_PER_REV*3)%36;
      6'd4:step_remainder=(COUNTS_PER_REV*4)%36;
      6'd5:step_remainder=(COUNTS_PER_REV*5)%36;
      6'd6:step_remainder=(COUNTS_PER_REV*6)%36;
      6'd7:step_remainder=(COUNTS_PER_REV*7)%36;
      6'd8:step_remainder=(COUNTS_PER_REV*8)%36;
      6'd9:step_remainder=(COUNTS_PER_REV*9)%36;
      6'd10:step_remainder=(COUNTS_PER_REV*10)%36;
      6'd11:step_remainder=(COUNTS_PER_REV*11)%36;
      6'd12:step_remainder=(COUNTS_PER_REV*12)%36;
      6'd13:step_remainder=(COUNTS_PER_REV*13)%36;
      6'd14:step_remainder=(COUNTS_PER_REV*14)%36;
      6'd15:step_remainder=(COUNTS_PER_REV*15)%36;
      6'd16:step_remainder=(COUNTS_PER_REV*16)%36;
      6'd17:step_remainder=(COUNTS_PER_REV*17)%36;
      6'd18:step_remainder=(COUNTS_PER_REV*18)%36;
      6'd19:step_remainder=(COUNTS_PER_REV*19)%36;
      6'd20:step_remainder=(COUNTS_PER_REV*20)%36;
      6'd21:step_remainder=(COUNTS_PER_REV*21)%36;
      6'd22:step_remainder=(COUNTS_PER_REV*22)%36;
      6'd23:step_remainder=(COUNTS_PER_REV*23)%36;
      6'd24:step_remainder=(COUNTS_PER_REV*24)%36;
      6'd25:step_remainder=(COUNTS_PER_REV*25)%36;
      6'd26:step_remainder=(COUNTS_PER_REV*26)%36;
      6'd27:step_remainder=(COUNTS_PER_REV*27)%36;
      6'd28:step_remainder=(COUNTS_PER_REV*28)%36;
      6'd29:step_remainder=(COUNTS_PER_REV*29)%36;
      6'd30:step_remainder=(COUNTS_PER_REV*30)%36;
      6'd31:step_remainder=(COUNTS_PER_REV*31)%36;
      6'd32:step_remainder=(COUNTS_PER_REV*32)%36;
      6'd33:step_remainder=(COUNTS_PER_REV*33)%36;
      6'd34:step_remainder=(COUNTS_PER_REV*34)%36;
      6'd35:step_remainder=(COUNTS_PER_REV*35)%36;
      6'd36:step_remainder=(COUNTS_PER_REV*36)%36;
      default:step_remainder=0;endcase end endfunction
    reg signed [17:0] ui_step_whole;
    reg signed [6:0] ui_step_remainder;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin ui_step_whole<=COUNTS_PER_REV/6;ui_step_remainder<=(COUNTS_PER_REV%6)*6;end
      else begin ui_step_whole<=step_whole(ui_step_units);ui_step_remainder<=step_remainder(ui_step_units);end
    wire ui_turn=ui_action==8 || ui_action==9;
    wire ui_positive=ui_action==6 || ui_action==9;
    wire signed [17:0] ui_whole=ui_turn ? COUNTS_PER_REV : ui_step_whole;
    wire signed [6:0] ui_remainder=ui_turn ? 7'sd0 : ui_step_remainder;
    wire signed [17:0] ui_base=$signed(arm_target_position)+(ui_positive ? ui_whole : -ui_whole);
    wire signed [6:0] ui_fraction=$signed(ui_residual)+(ui_positive ? ui_remainder : -ui_remainder);
    wire signed [2:0] ui_adjust=(ui_fraction>18 || (ui_fraction==18 && ui_base>=0)) ? 3'sd1 :
        (ui_fraction < -18 || (ui_fraction== -18 && ui_base<=0)) ? -3'sd1 : 3'sd0;
    wire signed [17:0] ui_target_rounded=ui_base+ui_adjust;
    wire signed [6:0] ui_fraction_next=ui_fraction-ui_adjust*36;
    assign arm_target_rx=local_target ? (ui_action==7 ? 32'sd0 : ui_target_rounded) : arm_target_uart_valid ? arm_target_uart : center_return_pending ? panel_center : reciprocal_target;
    assign manual_target_valid=arm_target_uart_valid || local_target;
    assign arm_target_rx_valid=manual_target_valid || reciprocal_valid;
    wire arm_target_rx_valid;
    wire config_rx_valid, swing_rx_valid, period_rx_valid, wrap_rx_valid, tilt_rx_valid;
    wire run_rx_valid, run_rx_command, run_rx_manual, query_rx_valid;
    wire run_uart_valid,run_uart_command,run_uart_manual,run_uart_detach;
    wire handoff_request=run_uart_valid && run_uart_detach && !local_stop;

    wire ui_start_ok=config_loaded && adc_valid && !adc_bad && !run_request && state==OFF;
    wire ui_run=local_stop || (local_start && ui_start_ok);
    wire uart_stop=run_uart_valid && !run_uart_command;
    wire run_from_ui=ui_run && !uart_stop;
    assign run_rx_valid=(run_uart_valid && !run_uart_detach) || ui_run;
    assign run_rx_command=!(local_stop || uart_stop) &&
        (local_start || (run_uart_valid && run_uart_command));
    assign run_rx_manual=run_from_ui ? ui_action==3 : run_uart_manual;
    wire [159:0] swing_rx; wire [7:0] swing_rx_length;
    wire [15:0] period_rx;
    wire [15:0] outer_period_rx; wire outer_period_rx_valid;
    wire [31:0] wrap_rx;
    wire [63:0] live_pd_rx;wire live_pd_rx_valid;
    wire [95:0] live_pid_rx;wire live_pid_rx_valid;
    wire [15:0] pwm_limit_rx;wire pwm_limit_rx_valid;
    reg [9:0] pwm_limit_code;
    wire [15:0] pwm_output_limit={6'd0,pwm_limit_code};
    reg pwm_limit_due;reg [7:0] pwm_limit_result;wire send_pwm_limit;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin pwm_limit_code<=900;pwm_limit_due<=0;pwm_limit_result<=0;end
      else begin
        if(send_pwm_limit)pwm_limit_due<=0;
        if(pwm_limit_rx_valid)begin
          pwm_limit_due<=1;pwm_limit_result<=pwm_limit_rx>900 ? 8'd1 : 8'd0;
          if(pwm_limit_rx<=900)pwm_limit_code<=pwm_limit_rx[9:0];
        end else if(query_rx_valid)begin pwm_limit_due<=1;pwm_limit_result<=0;end
      end
    wire [23:0] tuning_rx;wire tuning_rx_valid;
    reg [1:0] correction_strength;reg [15:0] correction_rate_q8;
    reg tuning_due;reg [7:0] tuning_result;wire send_tuning;
    wire tuning_range_ok=tuning_rx[7:0]<=2 && tuning_rx[23:8]>=7 && tuning_rx[23:8]<=147;
    wire tuning_state_ok=!run_request || pid_enable;
    wire tuning_accept=tuning_rx_valid && tuning_range_ok && tuning_state_ok;
    wire tuning_change=tuning_accept && (tuning_rx[1:0]!=correction_strength || tuning_rx[23:8]!=correction_rate_q8);
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin correction_strength<=0;correction_rate_q8<=44;tuning_due<=0;tuning_result<=0;end
      else begin
        if(send_tuning)tuning_due<=0;
        if(tuning_rx_valid)begin
          tuning_due<=1;tuning_result<=!tuning_range_ok ? 8'd1 : !tuning_state_ok ? 8'd2 : 8'd0;
          if(tuning_accept)begin correction_strength<=tuning_rx[1:0];correction_rate_q8<=tuning_rx[23:8];end
        end else if(query_rx_valid)begin tuning_due<=1;tuning_result<=0;end
      end
    wire [7:0] response_rx;wire response_rx_valid;
    reg response_fast;reg response_due;reg [7:0] response_result;
    wire response_accept=response_rx_valid && response_rx<=1 && (!run_request || pid_enable);
    wire response_change=response_accept && response_rx[0]!=response_fast;
    wire send_response;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin response_fast<=1;response_due<=0;response_result<=0;end
      else begin
        if(send_response)response_due<=0;
        if(response_rx_valid)begin
          response_due<=1;response_result<=response_rx>1 ? 8'd1 : (!run_request || pid_enable) ? 8'd0 : 8'd2;
          if(response_accept)response_fast<=response_rx[0];
        end else if(query_rx_valid)begin response_due<=1;response_result<=0;end
      end
    wire [63:0] trim_rx;
    wire trim_rx_valid,monitor_rx_valid,monitor_rx_enable;
    wire [127:0] board_rx;
    wire board_rx_valid,board_rx_extended,board_rx_triplet,reciprocal_rx_valid,reciprocal_rx_enable;
    wire arm_target_uart_has_tilt;
    wire [15:0] arm_target_uart_tilt;
    wire signed [31:0] internal_target,trimmed_target;
    wire motion_ramping,motion_reference_start,motion_reference_cancel;
    reg half_turn_negative;
    wire [7:0] trim_status;
    reg [63:0] trim_config;
    reg trim_loaded,monitor_enabled;
    reg [7:0] trim_result;
    wire trim_range_ok=trim_rx[31:0]<=32'd8388352 && trim_rx[47:32]>=1 && trim_rx[47:32]<=32767 && trim_rx[63:48]<=1;
    wire trim_state_ok=!run_request || (pid_enable && trim_rx[63:32]==trim_config[63:32]);
    wire trim_accept=trim_rx_valid && trim_range_ok && trim_state_ok;
    wire [15:0] tilt_rx; wire [15:0] deadzone_rx; wire deadzone_rx_valid;
// 配置命令解析：完整帧通过校验后，分发至对应参数接收逻辑。
    uart_config_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_config_rx (
        .clk(clk),.rst_n(rst_n),.uart_rxd(control_rx),
        .payload(config_uart),.valid(config_rx_valid),
        .swing_payload(swing_rx),.swing_valid(swing_rx_valid),.swing_length(swing_rx_length),
        .period_payload(period_rx),.period_valid(period_rx_valid),
        .outer_period_payload(outer_period_rx),.outer_period_valid(outer_period_rx_valid),
        .wrap_payload(wrap_rx),.wrap_valid(wrap_rx_valid),
        .tilt_payload(tilt_rx),.tilt_valid(tilt_rx_valid),
        .deadzone_payload(deadzone_rx),.deadzone_valid(deadzone_rx_valid),
        .live_pid_payload(live_pid_rx),.live_pid_valid(live_pid_rx_valid),
        .live_pd_payload(live_pd_rx),.live_pd_valid(live_pd_rx_valid),
        .pwm_limit_payload(pwm_limit_rx),.pwm_limit_valid(pwm_limit_rx_valid),
        .tuning_payload(tuning_rx),.tuning_valid(tuning_rx_valid),
        .response_payload(response_rx),.response_valid(response_rx_valid),
        .trim_payload(trim_rx),.trim_valid(trim_rx_valid),.monitor_valid(monitor_rx_valid),.monitor_enable(monitor_rx_enable),
        .board_payload(board_rx),.board_valid(board_rx_valid),.board_extended(board_rx_extended),.board_triplet(board_rx_triplet),
        .reciprocal_valid(reciprocal_rx_valid),.reciprocal_enable(reciprocal_rx_enable),
        .target_has_tilt(arm_target_uart_has_tilt),.target_tilt_payload(arm_target_uart_tilt),
        .query_valid(query_rx_valid),
        .target_payload(arm_target_uart),.target_valid(arm_target_uart_valid),
        .run_command(run_uart_command),.run_manual(run_uart_manual),.run_detach(run_uart_detach),.run_valid(run_uart_valid)
    );
    wire [15:0] new_zero=config_rx[15:0], new_capture=config_rx[47:32];
    wire [15:0] new_fall=config_rx[63:48], new_travel=config_rx[79:64];
    wire [7:0] new_angle_sign=config_rx[23:16], new_encoder_sign=config_rx[31:24];
    wire config_ok=new_zero>=16 && new_zero<=4079 &&
        (new_angle_sign==8'h01 || new_angle_sign==8'hff) &&
        (new_encoder_sign==8'h01 || new_encoder_sign==8'hff) &&
        new_capture>=1 && new_capture<=1024 && new_fall>new_capture && new_fall<=2048 &&
        new_travel>=1 && new_travel<=32767 &&
        $signed(config_rx[111:80])<=131072 && $signed(config_rx[111:80])>=-131072 &&
        $signed(config_rx[143:112])<=1048576 && $signed(config_rx[143:112])>=-1048576 &&
        $signed(config_rx[175:144])<=2097152 && $signed(config_rx[175:144])>=-2097152 &&
        $signed(config_rx[207:176])<=1048576 && $signed(config_rx[207:176])>=-1048576 &&
        $signed(config_rx[239:208])<=4194304 && $signed(config_rx[239:208])>=-4194304;
    reg config_checked_valid,config_checked_ok;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin config_checked_valid<=0;config_checked_ok<=0;end
        else begin config_checked_valid<=config_rx_valid || local_zero;config_checked_ok<=config_ok;end
    end
    assign config_accept=config_checked_valid && config_checked_ok;
    reg run_request;

    wire signed [15:0] new_pump=swing_rx[15:0];
    wire swing_ok=new_pump!=0 && new_pump>=-900 && new_pump<=900 &&
        swing_rx[31:16]>=1 && swing_rx[31:16]<=1000 &&
        swing_rx[47:32]<=4095 && swing_rx[63:48]<=4095 &&
        swing_rx[79:64]<=4095 &&
        swing_rx[95:80]>=5 && swing_rx[95:80]<=64 &&
        swing_rx[111:96]>=80 && swing_rx[111:96]<=480 &&
        swing_rx[111:96]>swing_rx[95:80] &&
        swing_rx[127:112]>=5 && swing_rx[127:112]<=32767;
    wire signed [15:0] new_stage2_pwm=swing_rx[143:128];
    wire stage2_ok=new_stage2_pwm>=-900 && new_stage2_pwm<=900 && swing_rx[159:144]>=1 && swing_rx[159:144]<=1000;
    wire swing_accept=swing_rx_valid && swing_ok && !run_request &&
        (swing_rx_length==16 || (swing_rx_length==20 && stage2_ok));
    wire period_accept=period_rx_valid && period_rx==16'd1000 && !run_request;
    assign wrap_accept=wrap_rx_valid && !run_request &&
                     wrap_rx[15:0]<=16'd499 && wrap_rx[31:16]<=16'd4095 &&
                     wrap_rx[31:16]>wrap_rx[15:0] &&
                     (wrap_rx[31:16]-wrap_rx[15:0])>=16'd1000;

    wire tilt_accept=tilt_rx_valid && !run_request &&
                     tilt_rx>=16'd1 && tilt_rx<=16'd102;
    wire deadzone_accept=deadzone_rx_valid && !run_request && deadzone_rx<=16'd300;
    reg [15:0] pwm_deadzone;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)pwm_deadzone<=0;
        else if(deadzone_accept)pwm_deadzone<=deadzone_rx;

    wire [15:0] pid_output_limit=pwm_output_limit>pwm_deadzone ? pwm_output_limit-pwm_deadzone : 16'd0;
    reg [11:0] tilt_limit;
    reg [11:0] tilt_echo;
    reg triplet_limits_loaded;reg [1:0] turn_tilt_kind;
    wire [15:0] profile_tilt_limit,normal_effective_limit;
    reg normal_recovery_active,normal_recovery_armed;
    reg [2:0] disturbance_ms;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin tilt_limit<=12'd2; tilt_echo<=12'd2; end
        else if(triplet_limits_loaded)begin tilt_limit<=profile_tilt_limit[11:0];tilt_echo<=profile_tilt_limit[11:0];end
        else if(arm_target_accept && arm_target_with_tilt)begin tilt_limit<=arm_target_tilt;tilt_echo<=arm_target_tilt;end
        else if (tilt_accept) begin tilt_limit<=tilt_rx; tilt_echo<=tilt_rx; end
    reg [31:0] wrap_echo;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin

            wrap_low<=0; wrap_high<=12'd2640;
            wrap_span<=12'd2640; wrap_half_span<=12'd1320;
            wrap_echo<={16'd2640,16'd0};
        end else if (wrap_accept) begin
            wrap_low<=wrap_rx[15:0]; wrap_high<=wrap_rx[31:16];
            wrap_span<=wrap_rx[31:16]-wrap_rx[15:0];
            wrap_half_span<=(wrap_rx[31:16]-wrap_rx[15:0])>>1;
            wrap_echo<=wrap_rx;
        end
    end
    reg signed [15:0] swing_pump_pwm;
    reg [15:0] pulse_ms,swing_target_limit;
    reg [11:0] bottom_code,stage1_target,stage1_upper,bottom_window,rearm_distance;
    reg [15:0] control_period_us,period_echo;
    reg [159:0] swing_echo; reg [7:0] swing_echo_length;
    reg stage2_configured; reg signed [15:0] stage2_pwm;reg [15:0] stage2_ms;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin
        stage2_configured<=0;stage2_pwm<=200;stage2_ms<=20;swing_echo_length<=16;
        swing_pump_pwm<=250;pulse_ms<=20;bottom_code<=2560;stage1_target<=1808;
        stage1_upper<=1168;bottom_window<=32;rearm_distance<=160;swing_target_limit<=160;
        swing_echo<={16'd160,16'd160,16'd32,16'd1168,16'd1808,16'd2560,16'd20,16'd250};
      end else if(swing_accept)begin
        stage2_configured<=swing_rx_length==20;swing_echo_length<=swing_rx_length;
        stage2_pwm<=swing_rx[143:128];stage2_ms<=swing_rx[159:144];
        swing_pump_pwm<=new_pump;pulse_ms<=swing_rx[31:16];bottom_code<=swing_rx[47:32];
        stage1_target<=swing_rx[63:48];stage1_upper<=swing_rx[79:64];
        bottom_window<=swing_rx[95:80];rearm_distance<=swing_rx[111:96];
        swing_target_limit<=swing_rx[127:112];swing_echo<=swing_rx;
      end
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin control_period_us<=1000;control_div<=CONTROL_DIV;control_limit<=CONTROL_DIV-1;period_echo<=1000;end
    reg config_loaded, angle_reverse, encoder_reverse;
    reg [239:0] config_echo;
    reg [11:0] zero_code, capture_window, fall_limit;
    reg [15:0] travel_limit;
    reg signed [23:0] kp_q8,ki_q12,kd_q8,arm_kp_q16,arm_kd_q16;

    reg [95:0] live_pid_staged;
    reg live_pid_pending,live_pid_range_ok;
    reg [103:0] live_pid_echo;
    wire live_pid_rx_ok=$signed(live_pid_rx[31:0])>=-131072 && $signed(live_pid_rx[31:0])<=131072 &&
        $signed(live_pid_rx[63:32])>=-1048576 && $signed(live_pid_rx[63:32])<=1048576 &&
        $signed(live_pid_rx[95:64])>=-2097152 && $signed(live_pid_rx[95:64])<=2097152 && live_pid_rx!=0;
    wire pid_enable;reg outer_tick;
    reg [63:0] live_pd_staged;
    reg live_pd_pending,live_pd_range_ok;
    reg [71:0] live_pd_echo;
    wire live_pd_rx_ok=$signed(live_pd_rx[31:0])>=-1048576 && $signed(live_pd_rx[31:0])<=1048576 &&
        $signed(live_pd_rx[63:32])>=-4194304 && $signed(live_pd_rx[63:32])<=4194304;

    wire live_pd_response=live_pd_pending && !config_accept &&
        ((outer_tick && pid_enable && live_pd_range_ok) || (tick && (!pid_enable || !live_pd_range_ok)));
    wire live_pd_commit=live_pd_response && outer_tick && pid_enable && live_pd_range_ok;
    wire [63:0] live_pd_actual={{8{arm_kd_q16[23]}},arm_kd_q16,{8{arm_kp_q16[23]}},arm_kp_q16};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin live_pd_staged<=0;live_pd_pending<=0;live_pd_range_ok<=0;live_pd_echo<=0;end
        else if(config_accept)live_pd_pending<=0;
        else begin
            if(live_pd_response)begin
                live_pd_pending<=0;
                live_pd_echo<=live_pd_commit ? {live_pd_staged,8'd0} : {live_pd_actual,live_pd_range_ok ? 8'd1 : 8'd2};
            end else if(query_rx_valid)live_pd_echo<={live_pd_actual,8'd0};
            if(live_pd_rx_valid)begin live_pd_staged<=live_pd_rx;live_pd_pending<=1;live_pd_range_ok<=live_pd_rx_ok;end
        end
    end
    wire live_pid_response=tick && live_pid_pending && !config_accept;
    wire live_pid_commit=live_pid_response && live_pid_range_ok && pid_enable;

    reg free_pending;
    wire free_request=local_event && ui_action==0;
    assign reciprocal_allowed=pid_enable && !config_accept && !local_stop &&
        !(run_rx_valid && !run_rx_command) && arm_kp_q16!=0;
    wire free_request_ok=reciprocal_allowed && reciprocal_amplitude_ok && !free_pending &&
        !config_checked_valid && !config_rx_valid && !manual_target_valid;
    wire free_response=tick && free_pending && !config_accept;
    wire free_commit=free_response && reciprocal_allowed && reciprocal_amplitude_ok && !manual_target_valid;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)free_pending<=0;
      else if(config_accept || !reciprocal_allowed || manual_target_valid)free_pending<=0;
      else begin
        if(free_response)free_pending<=0;
        if(free_request && free_request_ok)free_pending<=1;
      end

    always @(posedge clk or negedge rst_n)
      if(!rst_n)free_mode<=0;
      else if(!reciprocal_allowed)free_mode<=0;
      else if(reciprocal_rx_valid)free_mode<=reciprocal_rx_enable;
      else if(free_commit)free_mode<=!free_mode;
    wire amplitude_up,amplitude_down;
// 实体按键与往复参数入口，按键改变参数而不直接输出驱动。
    reciprocal_key #(.CLK_HZ(CLK_HZ)) plus(.clk(clk),.rst_n(rst_n),.key_n(amplitude_up_btn_n),.press(amplitude_up));
    reciprocal_key #(.CLK_HZ(CLK_HZ)) minus(.clk(clk),.rst_n(rst_n),.key_n(amplitude_down_btn_n),.press(amplitude_down));
    reg [79:0] board_config;
    reg [15:0] small_step_limit,large_step_limit,reverse_turn_limit;
    reg board_limits_loaded;
    reg [7:0] board_result;
    wire board_config_ok=board_rx[15:0]==COUNTS_PER_REV && board_rx[31:16]>=1 && board_rx[31:16]<=32767 &&
      board_rx[63:32]>=100 && board_rx[63:32]<=600000 && board_rx[79:64]>=1 && board_rx[79:64]<=64 &&
      (!board_rx_extended || (board_rx[95:80]>=1 && board_rx[95:80]<=64 && board_rx[111:96]>=1 && board_rx[111:96]<=64)) &&
      (!board_rx_triplet || (board_rx[127:112]>=1 && board_rx[127:112]<=64));
    wire board_state_ok=!run_request || (pid_enable && board_rx[15:0]==board_config[15:0]);
    wire board_accept=board_rx_valid && board_config_ok && board_state_ok;
    wire [31:0] amplitude_degrees_full;
    assign ui_amplitude_degrees=amplitude_degrees_full>511 ? 9'd511 : amplitude_degrees_full[8:0];
    wire amplitude_key_ready;
    wire [15:0] key_counts,key_tilt;
    wire amplitude_key_load=amplitude_key_ready && !profile_busy && !board_rx_valid;
    reciprocal_settings_adapter #(.COUNTS_PER_REV(COUNTS_PER_REV)) u_settings_adapter(
      .clk(clk),.rst_n(rst_n),.enabled(!profile_busy && !board_rx_valid),.up(amplitude_up),.down(amplitude_down),
      .amplitude_counts(board_config[31:16]),.small_limit(small_step_limit),.large_limit(large_step_limit),
      .amplitude_degrees(amplitude_degrees_full),.key_valid(amplitude_key_ready),.key_counts(key_counts),.key_limit(key_tilt));
    wire [79:0] key_config={key_tilt,board_config[63:32],key_counts,board_config[15:0]};
    wire reciprocal_config_load=board_accept || amplitude_key_load;
    wire [79:0] reciprocal_config=board_accept ? board_rx[79:0] : key_config;
    wire panel_active,panel_blocked,panel_ready,panel_valid;
    wire [15:0] panel_tilt;
    wire board_reply_extended=board_limits_loaded || board_rx_extended;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin board_config<={16'd20,32'd2000,16'd173,16'd1040};board_result<=0;small_step_limit<=20;large_step_limit<=40;reverse_turn_limit<=40;board_limits_loaded<=0;triplet_limits_loaded<=0;end
      else if(board_rx_valid)begin
        board_result<=!board_config_ok ? 8'd1 : !board_state_ok ? 8'd2 : 8'd0;
        if(board_accept)begin
          board_config<=board_rx[79:0];
          if(board_rx_extended)begin small_step_limit<={9'd0,board_rx[86:80]};large_step_limit<={9'd0,board_rx[102:96]};reverse_turn_limit<=board_rx_triplet ? {9'd0,board_rx[118:112]} : {9'd0,board_rx[102:96]};board_limits_loaded<=1;triplet_limits_loaded<=board_rx_triplet;end
        end
      end else begin
        if(amplitude_key_load)board_config<=key_config;
        if(query_rx_valid)board_result<=0;
      end
    end
    assign panel_ready=target_operable && !arm_target_pending && !manual_target_valid && !reciprocal_config_load;
    assign reciprocal_valid=(center_return_pending || panel_valid) && panel_ready;
    assign reciprocal_amplitude_ok=board_config[31:16]!=0;

    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin center_return_pending<=0;free_mode_previous<=0;end
      else begin
        free_mode_previous<=free_mode;
        if(!reciprocal_allowed || config_accept || reciprocal_config_load || manual_target_valid || (arm_target_accept && !target_from_reciprocal))
          center_return_pending<=0;
        else if(arm_target_pending && target_from_center_return)center_return_pending<=0;
        else if(free_mode_previous && !free_mode)center_return_pending<=1;
      end

    assign reciprocal_canceled=1'b0;
    board_target_control u_board_targets(
      .clk(clk),.rst_n(rst_n),.enable(reciprocal_allowed),.ms_tick(tick),.step_pulse(4'd0),
      .negative_direction(1'b0),.reciprocal_enable(free_mode),
      .config_load(reciprocal_config_load),.config_data(reciprocal_config),.current_target(arm_target_full),
      .external_target_value(arm_target_command),.external_target_accept(arm_target_accept && !target_from_reciprocal),
      .travel_limit(travel_limit),.request_valid(panel_valid),.request_ready(panel_ready),
      .small_step_limit(small_step_limit),.large_step_limit(large_step_limit),
      .request_target(reciprocal_target),.request_tilt(panel_tilt),.reciprocal_active(panel_active),.range_blocked(panel_blocked),.reciprocal_center(panel_center));
    wire [95:0] live_pid_actual={{8{kd_q8[23]}},kd_q8,{8{ki_q12[23]}},ki_q12,{8{kp_q8[23]}},kp_q8};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin live_pid_staged<=0;live_pid_pending<=0;live_pid_range_ok<=0;live_pid_echo<=0;end
        else begin
            if(config_accept)live_pid_pending<=0;
            else begin
                if(live_pid_response)begin
                    live_pid_pending<=0;
                    live_pid_echo<=live_pid_commit ? {live_pid_staged,8'd0} :
                        {live_pid_actual,live_pid_range_ok ? 8'd1 : 8'd2};
                end else if(query_rx_valid)live_pid_echo<={live_pid_actual,8'd0};
                if(live_pid_rx_valid)begin
                    live_pid_staged<=live_pid_rx;live_pid_range_ok<=live_pid_rx_ok;live_pid_pending<=1;
                end
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            config_loaded<=0; config_echo<=0;
            zero_code<=0; capture_window<=0; fall_limit<=0; travel_limit<=0;
            angle_reverse<=0; encoder_reverse<=0;
            kp_q8<=0; ki_q12<=0; kd_q8<=0; arm_kp_q16<=0; arm_kd_q16<=0;
        end else if (config_accept) begin
            config_loaded<=1; config_echo<=config_rx;
            zero_code<=new_zero; capture_window<=new_capture;
            fall_limit<=new_fall; travel_limit<=new_travel;
            angle_reverse<=new_angle_sign==8'hff;
            encoder_reverse<=new_encoder_sign==8'hff;
            kp_q8<=config_rx[103:80]; ki_q12<=config_rx[135:112];
            kd_q8<=config_rx[167:144]; arm_kp_q16<=config_rx[199:176];
            arm_kd_q16<=config_rx[231:208];
        end else begin
            if(live_pid_commit)begin
                kp_q8<=live_pid_staged[23:0];ki_q12<=live_pid_staged[55:32];kd_q8<=live_pid_staged[87:64];
                config_echo[175:80]<=live_pid_staged;
            end
            if(live_pd_commit)begin
                arm_kp_q16<=live_pd_staged[23:0];arm_kd_q16<=live_pd_staged[55:32];
                config_echo[239:176]<=live_pd_staged;
            end
        end
    end

    reg signed [12:0] raw_angle, circular_angle;
    wire signed [12:0] wrapped_angle=
        raw_angle>$signed({1'b0,wrap_half_span}) ?
            raw_angle-$signed({1'b0,wrap_span}) :
        raw_angle<-$signed({1'b0,wrap_half_span}) ?
            raw_angle+$signed({1'b0,wrap_span}) : raw_angle;
    reg signed [12:0] angle;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin raw_angle<=0; circular_angle<=0; angle<=0; end
        else begin
            raw_angle<=$signed({1'b0,adc_code})-$signed({1'b0,zero_code});
            circular_angle<=wrapped_angle;
            angle<=angle_reverse ? -circular_angle : circular_angle;
        end
    end

    reg [2:0] adc_pulse_delay;
    reg [1:0] speed_pipe;
    reg speed_ready;
    reg angle_sample_seen;
    reg signed [12:0] previous_angle;
    reg signed [13:0] speed_step_raw, speed_step_wrap;
    reg signed [21:0] speed_filtered_q8;
    reg signed [12:0] pendulum_velocity;
    wire signed [13:0] speed_step_short=
        speed_step_raw>$signed({2'b0,wrap_half_span}) ?
            speed_step_raw-$signed({2'b0,wrap_span}) :
        speed_step_raw<-$signed({2'b0,wrap_half_span}) ?
            speed_step_raw+$signed({2'b0,wrap_span}) : speed_step_raw;

    wire signed [22:0] speed_delta_q8=
        $signed({speed_step_wrap,8'd0})-$signed(speed_filtered_q8);
    wire signed [22:0] speed_increment_q8=speed_delta_q8[22] ?
        -(((-speed_delta_q8)+27'sd3)>>>2) : (speed_delta_q8+27'sd3)>>>2;
    wire signed [22:0] speed_next_q8=$signed(speed_filtered_q8)+speed_increment_q8;
    wire signed [22:0] speed_rounded=speed_filtered_q8[21] ?
        -(((-$signed(speed_filtered_q8))+27'sd128)>>>8) :
        ($signed(speed_filtered_q8)+27'sd128)>>>8;
    wire signed [13:0] speed_filter_next=speed_rounded[13:0];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_pulse_delay<=0; speed_pipe<=0; speed_ready<=0; angle_sample_seen<=0;
            previous_angle<=0; speed_step_raw<=0; speed_step_wrap<=0;
            speed_filtered_q8<=0; pendulum_velocity<=0;
        end else if (config_accept || wrap_accept) begin
            adc_pulse_delay<=0; speed_pipe<=0; speed_ready<=0; angle_sample_seen<=0;
            previous_angle<=angle; speed_step_raw<=0; speed_step_wrap<=0;
            speed_filtered_q8<=0; pendulum_velocity<=0;
        end else begin
            adc_pulse_delay<={adc_pulse_delay[1:0],adc_sample_pulse};
            speed_pipe<={speed_pipe[0],adc_pulse_delay[2] && adc_valid && angle_sample_seen};
            speed_ready<=speed_pipe[1];
            if (adc_pulse_delay[2] && adc_valid) begin
                speed_step_raw<=$signed({angle[12],angle})-
                                $signed({previous_angle[12],previous_angle});
                previous_angle<=angle;
                angle_sample_seen<=1;
            end
            if (speed_pipe[0]) speed_step_wrap<=speed_step_short;
            if (speed_pipe[1]) speed_filtered_q8<=speed_next_q8[21:0];
            if (speed_ready) begin
                pendulum_velocity<=speed_filter_next>14'sd4095 ? 13'sd4095 :
                    speed_filter_next< -14'sd4095 ? -13'sd4095 : speed_filter_next[12:0];
            end
        end
    end
    wire [11:0] angle_magnitude=angle[12] ? -angle[11:0] : angle[11:0];
    wire [11:0] velocity_magnitude=pendulum_velocity[12] ?
        -pendulum_velocity : pendulum_velocity;
// 运行状态机：停止、等待直立、闭环运行、故障、自动起摆和阶段完成状态。
    localparam [2:0] OFF=0,WAIT_UPRIGHT=1,ACTIVE=2,FAULT=3,SWING=4,COMPLETE=5;
    reg [2:0] state;
    reg manual_mode;
    reg [1:0] manual_samples;
    reg manual_seen;
    reg [11:0] manual_previous_code;
    wire signed [12:0] manual_raw_step=$signed({1'b0,adc_code})-$signed({1'b0,manual_previous_code});
    wire signed [12:0] manual_step=manual_raw_step>$signed({1'b0,wrap_half_span}) ? manual_raw_step-$signed({1'b0,wrap_span}) :
        manual_raw_step<-$signed({1'b0,wrap_half_span}) ? manual_raw_step+$signed({1'b0,wrap_span}) : manual_raw_step;
    wire signed [12:0] manual_error=$signed({1'b0,adc_code})-$signed({1'b0,zero_code});
    wire manual_in_window=manual_error<=$signed({1'b0,capture_window}) && manual_error>=-$signed({1'b0,capture_window});
    wire manual_capture_ready=manual_samples>=2;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin manual_samples<=0;manual_seen<=0;manual_previous_code<=0;end
        else if(state!=WAIT_UPRIGHT || !run_request || config_accept || !adc_valid)begin
            manual_samples<=0;manual_seen<=0;
        end else if(adc_sample_pulse)begin
            manual_previous_code<=adc_code;manual_seen<=1;
            if(manual_seen && manual_in_window && manual_step<=128 && manual_step>=-128)begin
                if(manual_samples<2)manual_samples<=manual_samples+1'b1;
            end else manual_samples<=0;
        end
    end
    reg signed [31:0] encoder_center,previous_position,arm_position;
    reg signed [31:0] travel_upper,travel_lower;
    reg signed [16:0] arm_target_position;
    reg signed [15:0] arm_velocity;
    reg signed [25:0] arm_velocity_filtered_q8;
    wire signed [26:0] arm_delta_q8=
        $signed({arm_velocity,8'd0})-$signed(arm_velocity_filtered_q8);
    wire signed [26:0] arm_increment_q8=arm_delta_q8[26] ?
        -(((-arm_delta_q8)+27'sd3)>>>2) : (arm_delta_q8+27'sd3)>>>2;
    wire signed [26:0] arm_next_q8=$signed(arm_velocity_filtered_q8)+arm_increment_q8;
    wire signed [26:0] arm_rounded=arm_velocity_filtered_q8[25] ?
        -(((-$signed(arm_velocity_filtered_q8))+27'sd128)>>>8) :
        ($signed(arm_velocity_filtered_q8)+27'sd128)>>>8;
    wire signed [17:0] arm_velocity_filtered=arm_rounded[17:0];
    wire [16:0] arm_magnitude=arm_position[31] ?
        -arm_position[16:0] : arm_position[16:0];

    reg fallen;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) fallen<=0;
        else fallen<=angle>$signed({1'b0,fall_limit}) ||
                     angle<-$signed({1'b0,fall_limit});

    wire [7:0] sensor_fault_raw=!adc_valid ? 8'd1 : 8'd0;
    reg [7:0] sensor_fault_sampled, sensor_fault_latched;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sensor_fault_sampled<=0; sensor_fault_latched<=0; end
        else begin
            sensor_fault_sampled<=sensor_fault_raw;
            if (config_accept || !run_request)
                sensor_fault_latched<=0;
            else if (run_request && sensor_fault_latched==0 && sensor_fault_raw!=0)
                sensor_fault_latched<=sensor_fault_raw;
        end
    end
    wire [7:0] sensor_fault_reason=sensor_fault_latched!=0 ?
        sensor_fault_latched : sensor_fault_sampled;
    wire adc_bad=sensor_fault_reason!=0;

    wire signed [16:0] encoder_delta=
        encoder_position[16:0]-encoder_center[16:0];
    wire signed [16:0] signed_position=encoder_reverse ?
        -encoder_delta : encoder_delta;
    wire signed [31:0] position_now={{15{signed_position[16]}},signed_position};
    wire signed [31:0] velocity_now=position_now-previous_position;
    wire stage1_done,pulse_event,stage2_active;
    wire capture_event;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) arm_velocity_filtered_q8<=0;
        else if (config_accept || capture_event || (state!=ACTIVE && state!=SWING)) arm_velocity_filtered_q8<=0;
        else if (tick) arm_velocity_filtered_q8<=arm_next_q8[25:0];
    wire travel_bad=encoder_position>travel_upper ||
                    encoder_position<travel_lower;

    reg [7:0] fault_reason;
    reg remote_run;
    reg remote_lockout;
    reg [31:0] remote_watchdog_us;
    reg [31:0] swing_elapsed_us;
    wire signed [16:0] arm_position_narrow=arm_position[16:0];
    wire [15:0] soft_limit=swing_target_limit<travel_limit ? swing_target_limit : travel_limit;
    reg signed [16:0] soft_bound;
    reg near_right,near_left;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin soft_bound<=0;near_right<=0;near_left<=0;end
        else begin
            soft_bound<=$signed({1'b0,soft_limit});
            near_right<=arm_position_narrow>=soft_bound;
            near_left<=arm_position_narrow<=-soft_bound;
        end
    end
    assign capture_event=tick && run_request && !adc_bad && !travel_bad &&
        ((state==SWING && stage2_configured && stage1_done) || (state==WAIT_UPRIGHT && manual_capture_ready));
    wire [15:0] pulse_count;
    wire signed [15:0] pulse_command;
// 自动起摆与直立接管：起摆输出经统一驱动路径，接管时建立编码器相对原点。
    swing_stage1_pulse #(.CLK_HZ(CLK_HZ)) u_stage1(
        .clk(clk),.rst_n(rst_n),.enable(state==SWING && run_request && config_loaded && !config_accept),
        .sample_strobe(adc_sample_pulse),.adc_valid(adc_valid),.adc_code(adc_code),
        .wrap_span(wrap_span),.wrap_half_span(wrap_half_span),.bottom_code(bottom_code),
        .target_code(stage1_target),.upper_code(stage1_upper),.bottom_window(bottom_window),
        .rearm_distance(rearm_distance),.first_pwm(swing_pump_pwm),.pulse_ms(pulse_ms),
        .angle_reverse(angle_reverse),.command(pulse_command),.done(stage1_done),
        .two_stage_enable(stage2_configured),.stage2_pwm(stage2_pwm),.stage2_ms(stage2_ms),
        .upright_code(zero_code),.capture_window(capture_window),.stage2_active(stage2_active),
        .pulse_event(pulse_event),.pulse_count(pulse_count));
    wire signed [15:0] swing_command=near_right && pulse_command>0 ? 16'sd0 :
         near_left && pulse_command<0 ? 16'sd0 : pulse_command;
    wire swing_enable=state==SWING && !stage1_done;
    assign pid_enable=rst_n && state==ACTIVE && run_request &&
                    config_loaded && !adc_bad && !travel_bad && !fallen;
    wire swing_motor_enable=rst_n && swing_enable && run_request &&
                            config_loaded && !adc_bad && !travel_bad;
    wire motor_enable=pid_enable || swing_motor_enable;
    reg pid_enable_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) pid_enable_d<=0;
        else pid_enable_d<=pid_enable;

    wire outer_period_accept=outer_period_rx_valid && !run_request &&
        outer_period_rx>=1000 && outer_period_rx<=50000;
    reg [15:0] outer_period_us;
    reg [31:0] outer_count,outer_limit;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin outer_period_us<=10000;outer_limit<=CLK_HZ/100-1;outer_count<=0;outer_tick<=0;end
        else begin
            outer_tick<=0;
            if(outer_period_accept)begin outer_period_us<=outer_period_rx;outer_limit<=outer_period_rx*CYCLES_PER_US-1;end
            if(!pid_enable || config_accept || outer_period_accept)outer_count<=0;
            else if(outer_count==outer_limit)begin outer_count<=0;outer_tick<=1;end
            else outer_count<=outer_count+1'b1;
        end
    end
    wire signed [31:0] outer_velocity_q8,outer_window_counts;
    wire outer_speed_valid;
    reg signed [31:0] position_for_speed;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)position_for_speed<=0;
        else position_for_speed<=position_now;
    encoder_speed_estimator #(.CLK_HZ(CLK_HZ),.VARIABLE_PERIOD(1)) u_outer_speed(
        .clk(clk),.rst_n(rst_n),.enable(pid_enable && !config_accept),.tick(outer_tick),
        .position(position_for_speed),.period_us(outer_period_us),.velocity_q8(outer_velocity_q8),
        .window_counts(outer_window_counts),.valid(outer_speed_valid));
    wire signed [12:0] target_tilt;
    wire signed [17:0] target_trim_offset;
// 用户目标保持界面语义，内部目标叠加有界补偿后提供给位置外环。
    target_trim_supervisor #(.FAST_SUPPORTED(1),.TUNING_SUPPORTED(1)) u_target_trim(
      .clk(clk),.rst_n(rst_n),.enable(pid_enable && trim_loaded && trim_config[48]),.ms_tick(tick && !motion_ramping),
      .reset_history(config_accept || capture_event),.fast_mode(response_fast),.response_change(response_change || tuning_change),.correction_strength(correction_strength),.correction_rate_q8(correction_rate_q8),
      .target_change(arm_target_accept && arm_target_command!=arm_target_full),.requested(arm_target_full),.actual(arm_position),
      .tolerance_q8(trim_config[31:0]),.max_trim(trim_config[47:32]),.travel_limit(travel_limit),
      .counts_per_rev(16'd1040),.internal_target(trimmed_target),.offset(target_trim_offset),.status(trim_status));
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin trim_config<={16'd0,16'd87,32'd1479};trim_loaded<=0;trim_result<=0;monitor_enabled<=0;end
      else begin
        if(trim_rx_valid)begin
          trim_result<=!trim_range_ok ? 8'd1 : !trim_state_ok ? 8'd2 : 8'd0;
          if(trim_accept)begin trim_config<=trim_rx;trim_loaded<=1;end
        end else if(query_rx_valid)trim_result<=trim_loaded ? 8'd0 : 8'd2;
        if(monitor_rx_valid)monitor_enabled<=monitor_rx_enable;
      end
    end
// 半圈与整圈运动参考：按剩余距离分段推进，并约束参考领先真实位置的距离。
    turn_motion_reference #(.COUNTS_PER_REV(COUNTS_PER_REV)) u_motion_reference(
      .clk(clk),.rst_n(rst_n),.ms_tick(tick),.enable(pid_enable && triplet_limits_loaded),
      .start(motion_reference_start),.cancel(motion_reference_cancel),
      .goal(trimmed_target),.actual(arm_position),.reference(internal_target),.ramping(motion_ramping));
    wire signed [17:0] arm_position_error=$signed(internal_target[17:0])-$signed(arm_position_narrow);
    wire outer_result_valid;
// 串级控制：位置外环输出倾角参考，摆杆内环据角度误差计算最终驱动量。
    position_pd_native u_outer_pd(.clk(clk),.rst_n(rst_n),.enable(pid_enable && !config_accept),
        .tick(outer_speed_valid),.position_error(arm_position_error),.velocity_q8(outer_velocity_q8),
        .kp_q16(arm_kp_q16),.kd_q16(arm_kd_q16),.tilt_limit(tilt_limit),
        .command(target_tilt),.valid(outer_result_valid));
    wire signed [12:0] angle_error=$signed(target_tilt)-$signed(angle);
    wire signed [15:0] angle_command;
    pid_native u_pid (
        .clk(clk),.rst_n(rst_n),.tick(tick),.enable(pid_enable_d),
        .error_in(angle_error),.measurement(angle),
        .kp_q8(kp_q8),.ki_q12(ki_q12),.kd_q8(kd_q8),
        .output_limit(pid_output_limit),.command(angle_command)
    );
// 驱动仲裁：仅允许当前运行阶段提供控制量，之后统一应用死区与总输出限幅。
    wire signed [15:0] drive_command=state==SWING ? swing_command : state==ACTIVE ? angle_command : 16'sd0;

    wire signed [16:0] drive_extended=$signed({drive_command[15],drive_command});
    wire signed [16:0] deadzone_extended=$signed({1'b0,pwm_deadzone});
    wire signed [16:0] compensated_drive=drive_command>0 ? drive_extended+deadzone_extended :
        drive_command<0 ? drive_extended-deadzone_extended : 17'sd0;
    wire signed [16:0] total_pwm_bound=$signed({1'b0,pwm_output_limit});
    wire signed [15:0] guarded_drive=compensated_drive>total_pwm_bound ? $signed(pwm_output_limit) :
        compensated_drive< -total_pwm_bound ? -$signed(pwm_output_limit) : compensated_drive[15:0];

    reg signed [15:0] drive_registered;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)drive_registered<=0;
        else drive_registered<=guarded_drive;
    wire signed [15:0] applied_command=motor_enable ? drive_registered : 16'sd0;
    tb6612_pwm #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) u_motor (
        .clk(clk),.rst_n(rst_n),.enable(motor_enable),.command(drive_registered),
        .in1(motor_in1),.in2(motor_in2),.pwm(motor_pwm),.stby(motor_stby)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_count<=0; tick<=0; state<=OFF; fault_reason<=0;
            run_request<=0; remote_run<=0;manual_mode<=0;
            remote_lockout<=0;
            remote_watchdog_us<=0; swing_elapsed_us<=0;
            encoder_center<=0; previous_position<=0; arm_position<=0; arm_velocity<=0;
            travel_upper<=0; travel_lower<=0;
        end else begin
            if (period_accept) begin control_count<=0; tick<=0; end
            else if (control_count>=control_limit) begin
                control_count<=0; tick<=1;
            end else begin control_count<=control_count+1'b1; tick<=0; end
            if (config_accept) begin
                state<=OFF; fault_reason<=0;
                run_request<=0; remote_run<=0;manual_mode<=0;
                remote_lockout<=0;
                remote_watchdog_us<=0; swing_elapsed_us<=0;
            end else begin

                if (button_press && button_armed && !LOCAL_UI && !run_rx_valid) begin
                    run_request<=!run_request;
                    if(!run_request)manual_mode<=0;
                    if (remote_run) remote_lockout<=1;
                    remote_run<=0;
                    remote_watchdog_us<=0;
                end else if (run_rx_valid) begin
                    if (!run_rx_command) begin
                        run_request<=0; remote_run<=0; remote_lockout<=run_from_ui;
                    end else if ((run_from_ui || !remote_lockout) && config_loaded) begin
                        run_request<=1; remote_run<=!run_from_ui;
                        if(run_from_ui)remote_lockout<=1;
                        if(!run_request && state==OFF)manual_mode<=run_rx_manual;
                    end
                    remote_watchdog_us<=0;
                end else if(handoff_request)begin
                    remote_run<=0;remote_watchdog_us<=0;
                end else if (tick && remote_run) begin
                    if (remote_watchdog_us+control_period_us>=32'd1000000) begin
                        run_request<=0; remote_run<=0; remote_watchdog_us<=0;
                    end else remote_watchdog_us<=remote_watchdog_us+control_period_us;
                end
                if (tick) begin
                    arm_position<=position_now;
                    if (velocity_now>32767) arm_velocity<=32767;
                    else if (velocity_now< -32768) arm_velocity<=-32768;
                    else arm_velocity<=velocity_now[15:0];
                    previous_position<=position_now;
                    case (state)
                        OFF: begin
                            swing_elapsed_us<=0;
                            if (run_request && config_loaded) begin
                                fault_reason<=0;
                                encoder_center<=encoder_position;
                                travel_upper<=encoder_position+$signed({16'b0,travel_limit});
                                travel_lower<=encoder_position-$signed({16'b0,travel_limit});
                                previous_position<=0; arm_position<=0; arm_velocity<=0;
                                state<=manual_mode ? WAIT_UPRIGHT : SWING;
                            end
                        end
                        WAIT_UPRIGHT: begin
                            if(!run_request)state<=OFF;
                            else if(adc_bad)begin fault_reason<=sensor_fault_reason;state<=FAULT;end
                            else if(travel_bad)begin fault_reason<=8'd4;state<=FAULT;end
                            else if(manual_capture_ready)begin
                                state<=ACTIVE;
                                encoder_center<=encoder_position;
                                previous_position<=0;arm_position<=0;arm_velocity<=0;
                                travel_upper<=encoder_position+$signed({16'b0,travel_limit});
                                travel_lower<=encoder_position-$signed({16'b0,travel_limit});
                            end
                        end
                        ACTIVE: begin
                            if (!run_request) state<=OFF;
                            else if (adc_bad) begin
                                fault_reason<=sensor_fault_reason; state<=FAULT;
                            end else if (travel_bad) begin
                                fault_reason<=8'd4; state<=FAULT;
                            end else if (fallen) begin
                                fault_reason<=8'd6; state<=FAULT;
                            end
                        end
                        SWING: begin
                            if (!run_request) state<=OFF;
                            else if (adc_bad) begin
                                fault_reason<=sensor_fault_reason; state<=FAULT;
                            end else if (travel_bad) begin
                                fault_reason<=8'd4; state<=FAULT;
                            end else if (swing_elapsed_us+control_period_us>=32'd20000000) begin
                                fault_reason<=8'd5; state<=FAULT;
                            end
                            else if (stage1_done) begin
                                if(stage2_configured)begin
                                    state<=ACTIVE;

                                    encoder_center<=encoder_position;
                                    previous_position<=0;arm_position<=0;arm_velocity<=0;
                                    travel_upper<=encoder_position+$signed({16'b0,travel_limit});
                                    travel_lower<=encoder_position-$signed({16'b0,travel_limit});
                                end else begin
                                    state<=COMPLETE;run_request<=0;remote_run<=0;remote_lockout<=1;
                                end
                            end
                            else begin
                                swing_elapsed_us<=swing_elapsed_us+control_period_us;
                            end
                        end
                        COMPLETE: if(run_request)state<=OFF;
                        FAULT: if (!run_request) state<=OFF;
                        default: begin fault_reason<=8'd7; state<=FAULT; end
                    endcase
                end
            end
        end
    end

    reg signed [16:0] arm_target_negative_bound;
    reg signed [31:0] arm_target_command;
    reg arm_target_pending,target_in_travel;
    reg arm_target_with_tilt,target_tilt_valid;reg [15:0] arm_target_tilt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arm_target_negative_bound<=0; arm_target_command<=0;
            arm_target_pending<=0; target_in_travel<=0;target_from_reciprocal<=0;target_from_center_return<=0;arm_target_with_tilt<=0;target_tilt_valid<=0;arm_target_tilt<=0;
        end else begin
            arm_target_negative_bound<=-$signed({1'b0,travel_limit});
            arm_target_pending<=arm_target_rx_valid;target_from_reciprocal<=reciprocal_valid && !manual_target_valid;
            target_from_center_return<=reciprocal_valid && center_return_pending && !manual_target_valid;
            if (arm_target_rx_valid) begin
                arm_target_command<=arm_target_rx;
                arm_target_with_tilt<=local_target || !arm_target_uart_valid || arm_target_uart_has_tilt;
                arm_target_tilt<=local_target ? ((ui_turn || (ui_action==7 ? (arm_target_full>=520 || arm_target_full<=-520) : ui_step_degrees>=180)) ? large_step_limit : small_step_limit) :
                  arm_target_uart_valid ? (arm_target_uart_has_tilt ? arm_target_uart_tilt : {4'd0,tilt_limit}) : panel_tilt;
                target_tilt_valid<=!arm_target_uart_valid || !arm_target_uart_has_tilt || (arm_target_uart_tilt>=1 && arm_target_uart_tilt<=64);
                target_in_travel<=arm_target_rx[31:16]=={16{arm_target_rx[15]}} &&
                    $signed({arm_target_rx[15],arm_target_rx[15:0]})<=$signed({1'b0,travel_limit}) &&
                    $signed({arm_target_rx[15],arm_target_rx[15:0]})>=arm_target_negative_bound;
            end
        end
    end
    wire target_operable=pid_enable && !config_accept &&
                        !(run_rx_valid && !run_rx_command) && !local_stop;
    wire arm_target_accept=arm_target_pending && target_operable && target_in_travel && target_tilt_valid &&
        !(target_from_reciprocal && (manual_target_valid || reciprocal_config_load || (!free_mode && !target_from_center_return)));
    wire signed [31:0] arm_target_full={{15{arm_target_position[16]}},arm_target_position};
    wire target_reset=arm_target_position!=0 && (!run_request || state!=ACTIVE);

// 根据经过检查的目标增量选择普通、整圈或半圈倾角策略。
    wire signed [17:0] nominal_target_delta=$signed({arm_target_command[16],arm_target_command[16:0]})-$signed({arm_target_position[16],arm_target_position});
    wire [1:0] requested_turn_kind=target_from_reciprocal || (ui_target_pending && ui_action_hold==7) ? 2'd0 :
      nominal_target_delta==COUNTS_PER_REV ? 2'd1 : nominal_target_delta==-COUNTS_PER_REV ? 2'd2 :
      (nominal_target_delta==COUNTS_PER_REV/2 || nominal_target_delta==-COUNTS_PER_REV/2) ? 2'd3 : 2'd0;
    wire actual_fits17=arm_position[31:17]=={15{arm_position[16]}};
    wire signed [17:0] nominal_actual_error=$signed({arm_target_position[16],arm_target_position})-$signed({arm_position[16],arm_position[16:0]});
    wire [17:0] nominal_actual_abs=nominal_actual_error[17] ? -nominal_actual_error : nominal_actual_error;
    wire turn_target_reached=actual_fits17 && {nominal_actual_abs,8'd0}<=trim_config[31:0];
    assign normal_effective_limit=turn_tilt_kind==0 && normal_recovery_active && small_step_limit<40 ? 16'd40 : small_step_limit;
    assign profile_tilt_limit=turn_tilt_kind==1 ? large_step_limit : turn_tilt_kind==2 ? reverse_turn_limit : normal_effective_limit;
    assign motion_reference_start=arm_target_accept && arm_target_command!=arm_target_full && requested_turn_kind!=0;
    assign motion_reference_cancel=config_accept || (arm_target_accept && (target_from_reciprocal || (arm_target_command!=arm_target_full && requested_turn_kind==0)));
    wire [31:0] motion_velocity_abs=outer_velocity_q8[31] ? -outer_velocity_q8 : outer_velocity_q8;
    localparam [31:0] MOTION_FINISH_SPEED_Q8=(20*COUNTS_PER_REV*256)/36000;
    wire [12:0] recovery_angle_error_abs=angle_error[12] ? -angle_error : angle_error;
    wire recovery_quiet=recovery_angle_error_abs<=32 && turn_target_reached && motion_velocity_abs<=MOTION_FINISH_SPEED_Q8;
    reg [8:0] motion_settle_ms;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)motion_settle_ms<=0;
      else if(!pid_enable || motion_ramping || !turn_target_reached ||
              motion_velocity_abs>MOTION_FINISH_SPEED_Q8 || (turn_tilt_kind==0 && recovery_angle_error_abs>32) || (arm_target_accept && arm_target_command!=arm_target_full))motion_settle_ms<=0;
      else if(tick && motion_settle_ms<(turn_tilt_kind==0 ? 9'd500 : 9'd100))motion_settle_ms<=motion_settle_ms+1'b1;

// 摆杆恢复档：直立接管或持续扰动时扩大普通倾角上限，连续稳定后回到用户保持值。
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin normal_recovery_active<=0;normal_recovery_armed<=0;disturbance_ms<=0;end
      else if(!pid_enable || !triplet_limits_loaded || config_accept || free_mode || turn_tilt_kind!=0 ||
              (arm_target_accept && arm_target_command!=arm_target_full))begin
        normal_recovery_active<=0;normal_recovery_armed<=0;disturbance_ms<=0;
      end else if(!pid_enable_d)begin normal_recovery_active<=1;normal_recovery_armed<=0;disturbance_ms<=0;end
      else if(tick)begin
        if(!normal_recovery_active && normal_recovery_armed && recovery_angle_error_abs>=40)begin
          if(disturbance_ms>=4)begin normal_recovery_active<=1;normal_recovery_armed<=0;disturbance_ms<=0;end
          else disturbance_ms<=disturbance_ms+1'b1;
        end else disturbance_ms<=0;
        if(recovery_quiet && motion_settle_ms>=500)begin normal_recovery_active<=0;normal_recovery_armed<=1;end
      end
    wire [15:0] requested_profile_tilt=requested_turn_kind==1 ? large_step_limit : requested_turn_kind==2 ? reverse_turn_limit : small_step_limit;
    wire [15:0] accepted_target_tilt=triplet_limits_loaded ?
      (target_from_reciprocal || arm_target_command!=arm_target_full ? requested_profile_tilt : profile_tilt_limit) :
      (arm_target_with_tilt ? arm_target_tilt : {4'd0,tilt_limit});
    always @(posedge clk or negedge rst_n)
      if(!rst_n)turn_tilt_kind<=0;
      else if(!triplet_limits_loaded || !pid_enable || config_accept)turn_tilt_kind<=0;
      else if(arm_target_accept && target_from_reciprocal)turn_tilt_kind<=0;
      else if(arm_target_accept && arm_target_command!=arm_target_full)turn_tilt_kind<=requested_turn_kind;
      else if(turn_tilt_kind!=0 && tick && turn_target_reached && (!motion_ramping && motion_settle_ms>=100))turn_tilt_kind<=0;

    always @(posedge clk or negedge rst_n)
      if(!rst_n)half_turn_negative<=0;
      else if(arm_target_accept && arm_target_command!=arm_target_full)half_turn_negative<=requested_turn_kind==3 && nominal_target_delta<0;
      else if(!pid_enable || turn_tilt_kind!=3)half_turn_negative<=0;

    reg [55:0] arm_target_echo;reg arm_target_echo_extended;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin arm_target_position<=0; arm_target_echo<=0;arm_target_echo_extended<=0; end
        else begin
            if (config_accept || !run_request || state!=ACTIVE) arm_target_position<=0;
            else if (arm_target_accept) arm_target_position<=arm_target_command[16:0];

            if (config_accept || target_reset) begin arm_target_echo<=0;arm_target_echo_extended<=0;end
            else if (arm_target_pending) begin
                arm_target_echo_extended<=arm_target_with_tilt;
                arm_target_echo<=arm_target_accept ?
                {accepted_target_tilt,8'd0,arm_target_command} : {{4'd0,tilt_limit},!target_operable ? 8'd1 : !target_in_travel ? 8'd2 : 8'd3,arm_target_full};
            end
            else if (query_rx_valid) arm_target_echo<={{4'd0,tilt_limit},8'd0,arm_target_full};
        end
    end

    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin ui_residual<=0;ui_target_pending<=0;ui_fraction_hold<=0;ui_notice<=0;ui_notice_valid<=0;end
      else begin
        ui_notice_valid<=0;
        ui_target_pending<=local_target;
        if(local_target)ui_fraction_hold<=ui_action==7 ? 0 : ui_fraction_next;
        if(config_accept || !run_request || state!=ACTIVE)ui_residual<=0;
        else if(arm_target_accept)begin
          if(ui_target_pending)ui_residual<=ui_fraction_hold;
          else ui_residual<=0;
        end
        if(local_event && ui_action>0 && ui_action<5 && !local_zero)begin
          ui_notice_valid<=1;
          if(local_stop)ui_notice<=1;
          else if(ui_action==4)ui_notice<=local_zero ? 3 : !adc_valid ? 9 : 8;
          else if(local_start)ui_notice<=ui_start_ok && !uart_stop ? 5 : !adc_valid ? 9 : 8;
        end
        if(local_event && local_target_action && !local_target)begin ui_notice_valid<=1;ui_notice<=8;end
        if(free_request && !free_request_ok)begin ui_notice_valid<=1;ui_notice<=8;end
        if(free_response)begin ui_notice_valid<=1;ui_notice<=free_commit ? 6 : 8;end
        if(reciprocal_canceled)begin ui_notice_valid<=1;ui_notice<=6;end
        if(local_zero_hold && config_accept)begin ui_notice_valid<=1;ui_notice<=3;end
        if(ui_target_pending)begin
          ui_notice_valid<=1;
          ui_notice<=arm_target_accept ? 6 : !adc_valid ? 9 : target_operable ? 7 : 4;
        end
      end

    wire frame_ready;

    reg ui_due;
    reg [15:0] ui_sequence;
    reg [87:0] ui_reply;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)ui_action_hold<=0;else if(reciprocal_canceled)ui_action_hold<=0;else if(local_event)ui_action_hold<=ui_action;
    reg trim_due,board_due,monitor_due;
    reg [31:0] monitor_count;
    reg [231:0] monitor_snapshot;
    wire signed [31:0] monitor_velocity_q8,monitor_window;
    wire monitor_speed_valid;
    encoder_speed_estimator #(.CLK_HZ(CLK_HZ),.VARIABLE_PERIOD(1)) u_monitor_speed(
      .clk(clk),.rst_n(rst_n),.enable(monitor_enabled),.tick(monitor_count>=CLK_HZ/100-1),
      .position(position_now),.period_us(16'd10000),.velocity_q8(monitor_velocity_q8),.window_counts(monitor_window),.valid(monitor_speed_valid));
    wire [7:0] board_flags={(motion_ramping || (turn_tilt_kind==0 && normal_recovery_active)),turn_tilt_kind,target_operable,panel_blocked,panel_active,free_mode,(turn_tilt_kind==3 && half_turn_negative)};
    reg [7:0] previous_board_flags;
    wire [7:0] trim_monitor_status={1'b1,(trim_loaded && trim_config[48]),trim_status[5:0]};
    reg detach_due;reg [7:0] detach_result;
    wire choose_detach=detach_due;
    wire send_detach=frame_ready && choose_detach;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin detach_due<=0;detach_result<=0;end
      else if(run_uart_valid && run_uart_detach)begin
        detach_due<=1;detach_result<=local_stop || !run_request ? 8'd1 : state==FAULT ? 8'd2 : 8'd0;
      end else if(send_detach)detach_due<=0;
    wire choose_pwm_limit=pwm_limit_due && !detach_due;
    assign send_pwm_limit=frame_ready && choose_pwm_limit;
    wire choose_tuning=tuning_due && !detach_due && !pwm_limit_due;
    assign send_tuning=frame_ready && choose_tuning;
    wire choose_response=response_due && !detach_due && !tuning_due && !pwm_limit_due;
    assign send_response=frame_ready && choose_response;
    wire choose_trim=trim_due && !detach_due && !response_due && !tuning_due && !pwm_limit_due;
    wire choose_board=!trim_due && board_due && !detach_due && !response_due && !tuning_due && !pwm_limit_due;
    wire legacy_reply_due=config_due || live_pid_due || live_pd_due || deadzone_due || swing_due || period_due || outer_period_due || wrap_due || tilt_due || target_due || ui_due;
    wire choose_monitor=monitor_due && !trim_due && !board_due && !legacy_reply_due && !detach_due && !response_due && !tuning_due && !pwm_limit_due;
    wire send_trim=frame_ready && choose_trim,send_board=frame_ready && choose_board,send_monitor=frame_ready && choose_monitor;
    wire extension_busy=choose_pwm_limit || choose_tuning || choose_response || choose_detach || choose_trim || choose_board || choose_monitor;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin trim_due<=0;board_due<=0;monitor_due<=0;monitor_count<=0;monitor_snapshot<=0;previous_board_flags<=0;end
      else begin
        previous_board_flags<=board_flags;
        if(trim_rx_valid || (query_rx_valid && (trim_loaded || monitor_enabled)))trim_due<=1;else if(send_trim)trim_due<=0;
        if(board_rx_valid || query_rx_valid || reciprocal_rx_valid || amplitude_key_load || previous_board_flags!=board_flags)board_due<=1;else if(send_board)board_due<=0;
        if(!monitor_enabled)begin monitor_count<=0;monitor_due<=0;end
        else if(monitor_count>=CLK_HZ/100-1)begin
          monitor_count<=0;monitor_due<=1;
          monitor_snapshot<={board_flags,panel_center,board_config[15:0],trim_monitor_status,flags,{4'd0,adc_code},applied_command,monitor_velocity_q8,arm_position,internal_target,arm_target_full};
        end else begin monitor_count<=monitor_count+1'b1;if(send_monitor)monitor_due<=0;end
      end
    end
    wire choose_ui=!extension_busy && ui_due;
    wire send_ui=frame_ready && choose_ui;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)begin ui_due<=0;ui_sequence<=0;ui_reply<=0;end
      else begin
        if(send_ui)ui_due<=0;
        if(ui_notice_valid)begin
          ui_sequence<=ui_sequence+1'b1;ui_due<=1;
          ui_reply<={(run_request ? arm_target_full : 32'sd0),4'b0,zero_code,4'b0,free_mode,manual_mode,remote_run,run_request,
                     4'b0,ui_notice,4'b0,ui_action_hold,ui_sequence+16'd1};
        end else if(query_rx_valid && LOCAL_UI)begin
          ui_due<=1;
          ui_reply<={(run_request ? arm_target_full : 32'sd0),4'b0,zero_code,4'b0,free_mode,manual_mode,remote_run,run_request,
                     8'b0,8'b0,ui_sequence};
        end
      end
    wire choose_config=!extension_busy && !ui_due && config_due;
    wire choose_live_pid=!extension_busy && !ui_due && !config_due && live_pid_due;
    wire choose_live_pd=!extension_busy && !ui_due && !config_due && !live_pid_due && live_pd_due;
    wire choose_deadzone=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && deadzone_due;
    wire choose_swing=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && swing_due;
    wire choose_period=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && period_due;
    wire choose_outer_period=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && !period_due && outer_period_due;
    wire choose_wrap=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && !period_due && !outer_period_due && wrap_due;
    wire choose_tilt=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && !period_due && !outer_period_due && !wrap_due && tilt_due;
    wire choose_target=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && !period_due && !outer_period_due && !wrap_due && !tilt_due && target_due;
    wire choose_motor=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && !period_due && !outer_period_due && !wrap_due && !tilt_due && !target_due && motor_due;
    wire choose_adc=!extension_busy && !ui_due && !config_due && !live_pid_due && !live_pd_due && !deadzone_due && !swing_due && !period_due && !outer_period_due && !wrap_due && !tilt_due && !target_due && !motor_due && adc_due;
    wire send_config=frame_ready && choose_config;
    wire send_live_pid=frame_ready && choose_live_pid;
    wire send_live_pd=frame_ready && choose_live_pd;
    wire send_deadzone=frame_ready && choose_deadzone;
    wire send_swing=frame_ready && choose_swing;
    wire send_period=frame_ready && choose_period;
    wire send_outer_period=frame_ready && choose_outer_period;
    wire send_wrap=frame_ready && choose_wrap;
    wire send_tilt=frame_ready && choose_tilt;
    wire send_target=frame_ready && choose_target;
    wire send_motor=frame_ready && choose_motor;
    wire send_adc=frame_ready && choose_adc;
    reg motor_stby_sampled;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) motor_stby_sampled<=0;
        else motor_stby_sampled<=motor_stby;
    wire [7:0] flags={state==COMPLETE || (state==SWING && stage2_active),state==SWING,config_loaded,motor_stby_sampled,!adc_bad,
                       state==FAULT,state==ACTIVE,run_request};
    reg [143:0] motor_snapshot;
    wire [239:0] packet_payload=choose_pwm_limit ? {pwm_output_limit,pwm_limit_result} : choose_tuning ? {correction_rate_q8,6'd0,correction_strength,tuning_result} : choose_response ? {7'd0,response_fast,response_result} : choose_detach ? {232'd0,detach_result} : choose_trim ? {trim_result,trim_config} : choose_board ? (triplet_limits_loaded ? {reverse_turn_limit,large_step_limit,small_step_limit,board_flags,board_result,board_config} : board_reply_extended ? {large_step_limit,small_step_limit,board_flags,board_result,board_config} : {board_flags,board_result,board_config}) : choose_monitor ? monitor_snapshot : choose_ui ? ui_reply : choose_config ? config_echo : choose_live_pid ? live_pid_echo : choose_live_pd ? live_pd_echo : choose_deadzone ? pwm_deadzone :
        choose_swing ? swing_echo : choose_period ? period_echo :
        choose_outer_period ? outer_period_us : choose_wrap ? wrap_echo :
        choose_tilt ? tilt_echo : choose_target ? arm_target_echo :
        choose_motor ? motor_snapshot : {7'd0,adc_bad,4'd0,adc_code};
// 遥测与配置回显共用发送仲裁，各帧在发送时锁存对应状态。
    uart_packet_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.MAX_PAYLOAD_BYTES(30)) u_tx (
        .clk(clk),.rst_n(rst_n),.frame_valid(send_pwm_limit || send_tuning || send_response || send_detach || send_trim || send_board || send_monitor || send_ui || send_config || send_live_pid || send_live_pd || send_deadzone || send_swing || send_period || send_outer_period || send_wrap || send_tilt || send_target || send_motor || send_adc),
        .frame_ready(frame_ready),.frame_type(choose_pwm_limit ? 8'h49 : choose_tuning ? 8'h47 : choose_response ? 8'h45 : choose_detach ? 8'h3c : choose_trim ? 8'h38 : choose_board ? 8'h41 : choose_monitor ? 8'h39 : choose_ui ? 8'h36 : choose_config ? 8'h33 : choose_live_pid ? 8'h2d : choose_live_pd ? 8'h2f : choose_deadzone ? 8'h2b : choose_swing ? (swing_echo_length==20 ? 8'h29 : 8'h27) : choose_period ? 8'h05 : choose_outer_period ? 8'h11 : choose_wrap ? 8'h06 : choose_tilt ? 8'h08 : choose_target ? 8'h0d : choose_motor ? 8'h35 : 8'h34),
        .frame_length(choose_pwm_limit ? 8'd3 : choose_tuning ? 8'd4 : choose_response ? 8'd2 : choose_detach ? 8'd1 : choose_trim ? 8'd9 : choose_board ? (triplet_limits_loaded ? 8'd18 : board_reply_extended ? 8'd16 : 8'd12) : choose_monitor ? 8'd29 : choose_ui ? 8'd11 : choose_config ? 8'd30 : choose_live_pid ? 8'd13 : choose_live_pd ? 8'd9 : choose_deadzone ? 8'd2 : choose_swing ? swing_echo_length : choose_period ? 8'd2 : choose_outer_period ? 8'd2 : choose_wrap ? 8'd4 : choose_tilt ? 8'd2 : choose_target ? (arm_target_echo_extended ? 8'd7 : 8'd5) : choose_motor ? 8'd18 : 8'd3),
        .frame_payload(packet_payload),.uart_txd(uart_txd)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_report_count<=0; motor_report_count<=0;
            adc_due<=0; motor_due<=0; config_due<=0; swing_due<=0; period_due<=0; outer_period_due<=0; wrap_due<=0; tilt_due<=0; target_due<=0; deadzone_due<=0;live_pid_due<=0;live_pd_due<=0;
            motor_snapshot<=0;
        end else begin
            if (adc_report_count==ADC_REPORT_DIV-1) begin
                adc_report_count<=0; adc_due<=1;
            end else adc_report_count<=adc_report_count+1'b1;
            if (motor_report_count==MOTOR_REPORT_DIV-1) begin
                motor_report_count<=0; motor_due<=1;
                motor_snapshot<={arm_target_full,{{3{target_tilt[12]}},target_tilt},fault_reason,flags,invalid_count,
                                 applied_command,{{3{pendulum_velocity[12]}},pendulum_velocity},arm_position};
            end else motor_report_count<=motor_report_count+1'b1;
            if (config_accept) config_due<=1;
            if (swing_accept) swing_due<=1;
            if (period_accept) period_due<=1;
            if (outer_period_accept) outer_period_due<=1;
            if (wrap_accept) wrap_due<=1;
            if (tilt_accept || (triplet_limits_loaded && tilt_limit!=profile_tilt_limit[11:0])) tilt_due<=1;
            if (deadzone_accept) deadzone_due<=1;
            if (send_config) config_due<=0;
            if (send_swing) swing_due<=0;
            if (send_period) period_due<=0;
            if (send_outer_period) outer_period_due<=0;
            if (send_wrap) wrap_due<=0;
            if (send_tilt) tilt_due<=0;
            if (send_deadzone) deadzone_due<=0;
            if(send_live_pid)live_pid_due<=0;
            if(live_pid_response)live_pid_due<=1;
            if(send_live_pd)live_pd_due<=0;
            if(live_pd_response)live_pd_due<=1;
            if (send_target) target_due<=0;
            if (arm_target_pending || target_reset || config_accept) target_due<=1;
            if (send_motor) motor_due<=0;
            if (send_adc) adc_due<=0;
            if (query_rx_valid) begin
                config_due<=1; swing_due<=1; period_due<=1; outer_period_due<=1; wrap_due<=1; tilt_due<=1; target_due<=1; deadzone_due<=1;live_pid_due<=1;live_pd_due<=1;
            end
        end
    end
endmodule
`default_nettype wire

`default_nettype none

// 编码器速度估计：根据采样间隔换算位移，统一输出每１０毫秒计数的８位小数值。
module encoder_speed_estimator #(parameter integer CLK_HZ=50000000, VARIABLE_PERIOD=0)(
 input wire clk,rst_n,enable,tick,
 input wire signed [31:0] position,
 input wire [15:0] period_us,
 output reg signed [31:0] velocity_q8,
 output reg signed [31:0] window_counts,
 output reg valid);
localparam integer US_DIV=CLK_HZ>=1000000 ? CLK_HZ/1000000 : 1;
reg [15:0] us_count;
reg [31:0] time_us,last_edge_us,edge_period;
reg signed [31:0] previous_position,window_position;
reg primed,edge_seen,period_valid,direction_negative;

wire signed [17:0] edge_long=$signed({position[16],position[16:0]})-$signed({previous_position[16],previous_position[16:0]});

wire signed [17:0] edge_delta=edge_long==18'sd65536 ? 18'sd65536 :
    $signed(edge_long[16:0]);

wire signed [17:0] window_long=$signed({position[16],position[16:0]})-$signed({window_position[16],window_position[16:0]});
wire signed [17:0] window_delta=window_long==18'sd65536 ? 18'sd65536 :
    $signed(window_long[16:0]);
wire [31:0] age=time_us-last_edge_us;
wire [31:0] interval=time_us-last_edge_us;
wire [31:0] timeout_us=edge_period>=50000 ? 32'd100000 :
    edge_period<=15000 ? 32'd30000 : edge_period<<1;
wire [31:0] denominator=age>edge_period ? age : edge_period;
wire reverse_now=edge_delta!=0 && edge_seen && edge_delta[17]!=direction_negative;
wire signed [17:0] magnitude=window_delta[17] ? -window_delta : window_delta;

reg busy,divide_negative;
reg tick_pending, pending_divide, pending_negative, pending_from_window;
reg [15:0] pending_window_magnitude;
reg signed [31:0] pending_velocity;
reg [31:0] pending_divisor;
reg [39:0] pending_dividend;
reg [5:0] bits_left;
reg [39:0] dividend,quotient;
reg [31:0] divisor;
reg [32:0] remainder;
wire [32:0] shifted_remainder={remainder[31:0],dividend[39]};
wire quotient_bit=shifted_remainder>={1'b0,divisor};
wire [32:0] next_remainder=quotient_bit ? shifted_remainder-{1'b0,divisor} : shifted_remainder;
wire [39:0] next_quotient={quotient[38:0],quotient_bit};
wire [31:0] limited_quotient=next_quotient>8388352 ? 32'd8388352 : next_quotient[31:0];
wire signed [31:0] limited_window=window_delta>32767 ? 32'sd32767 :
    window_delta< -32767 ? -32'sd32767 : window_delta;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  us_count<=0;time_us<=0;last_edge_us<=0;edge_period<=0;
  previous_position<=0;window_position<=0;primed<=0;edge_seen<=0;
  period_valid<=0;direction_negative<=0;velocity_q8<=0;window_counts<=0;
  valid<=0;busy<=0;bits_left<=0;dividend<=0;divisor<=1;quotient<=0;remainder<=0;divide_negative<=0;
  tick_pending<=0;pending_divide<=0;pending_negative<=0;pending_from_window<=0;pending_window_magnitude<=0;pending_velocity<=0;pending_divisor<=1;pending_dividend<=0;
 end else begin
  if(us_count==US_DIV-1)begin us_count<=0;time_us<=time_us+1'b1;end
  else us_count<=us_count+1'b1;
  valid<=0;
  if(!enable)begin
   previous_position<=position;window_position<=position;primed<=0;
   edge_seen<=0;period_valid<=0;edge_period<=0;velocity_q8<=0;window_counts<=0;busy<=0;
   tick_pending<=0;
  end else if(!primed)begin
   previous_position<=position;window_position<=position;primed<=1;last_edge_us<=time_us;
  end else begin
   if(edge_delta!=0)begin
    previous_position<=position;last_edge_us<=time_us;edge_seen<=1;
    direction_negative<=edge_delta[17];

    if(edge_seen && !reverse_now && interval>0 && interval<=100000)begin
     edge_period<=interval;period_valid<=1;
    end else begin edge_period<=0;period_valid<=0;end
   end

   if(tick_pending)begin
    tick_pending<=0;
    if(pending_divide)begin
     busy<=1;bits_left<=40;dividend<=pending_from_window ? pending_window_magnitude*40'd2560000 : pending_dividend;divisor<=pending_divisor;
     quotient<=0;remainder<=0;divide_negative<=pending_negative;
    end else begin velocity_q8<=pending_velocity;valid<=1;end
   end
   if(busy)begin
    remainder<=next_remainder;dividend<=dividend<<1;quotient<=next_quotient;
    bits_left<=bits_left-1'b1;
    if(bits_left==1)begin
     velocity_q8<=divide_negative ? -$signed(limited_quotient) : $signed(limited_quotient);
     busy<=0;valid<=1;
    end
   end
   if(tick && !busy && !tick_pending)begin
    tick_pending<=1;pending_divide<=0;pending_from_window<=0;
    pending_divisor<=denominator==0 ? 32'd1 : denominator;
    pending_dividend<=40'd2560000;
    pending_negative<=direction_negative;
    window_counts<=window_delta;window_position<=position;
    if((window_delta>18'sd1 || window_delta< -18'sd1) && !reverse_now)begin
     if(VARIABLE_PERIOD && period_us!=10000)begin
      pending_divide<=1;
      pending_divisor<=period_us==0 ? 32'd10000 : {16'd0,period_us};
      pending_window_magnitude<=limited_window[31] ? -limited_window[15:0] : limited_window[15:0];
      pending_from_window<=1;
      pending_negative<=limited_window[31];
     end else pending_velocity<=limited_window<<<8;
    end else if(reverse_now || (edge_delta!=0 && (!edge_seen || interval>100000)))begin
     pending_velocity<=edge_delta[17] ? -32'sd256 : 32'sd256;
    end else if(!edge_seen || (!period_valid && age>=30000) ||
                (period_valid && age>=timeout_us))begin
     pending_velocity<=0;
    end else if(!period_valid)begin
     pending_velocity<=direction_negative ? -32'sd256 : 32'sd256;
    end else begin
     pending_divide<=1;
    end
   end
  end
 end
end
endmodule
`default_nettype wire

`default_nettype none

// 运动参考生成器：使用带符号小数状态，在目标与真实编码器反馈之间生成分段限速参考。
module turn_motion_reference #(parameter integer COUNTS_PER_REV=1040)(
 input wire clk,rst_n,ms_tick,enable,start,cancel,
 input wire signed [31:0] goal,actual,
 output wire signed [31:0] reference,output reg ramping);
 reg signed [24:0] reference_q8;
 reg [4:0] elapsed_ms;
 wire signed [24:0] goal_q8={goal[16:0],8'd0};
 wire signed [24:0] remaining=goal_q8-reference_q8;
 wire negative_direction=remaining[24];
 wire signed [24:0] directional_remaining=negative_direction ? -remaining : remaining;
 localparam [24:0] D90=(90*COUNTS_PER_REV*256)/360,D30=(30*COUNTS_PER_REV*256)/360,D10=(10*COUNTS_PER_REV*256)/360;
 localparam [24:0] V120=(120*COUNTS_PER_REV*256)/18000,V80=(80*COUNTS_PER_REV*256)/18000,
                         V40=(40*COUNTS_PER_REV*256)/18000,V20=(20*COUNTS_PER_REV*256)/18000;
 localparam signed [25:0] LEAD=(60*COUNTS_PER_REV*256)/360,REVERSE_LEAD=(90*COUNTS_PER_REV*256)/360;
 localparam [24:0] R60=(60*COUNTS_PER_REV*256)/360,R20=(20*COUNTS_PER_REV*256)/360,R5=(5*COUNTS_PER_REV*256)/360;
 localparam [24:0] RV180=(180*COUNTS_PER_REV*256)/18000,RV120=(120*COUNTS_PER_REV*256)/18000,
                         RV60=(60*COUNTS_PER_REV*256)/18000,RV30=(30*COUNTS_PER_REV*256)/18000;
 wire [24:0] positive_step=directional_remaining>D90 ? V120 : directional_remaining>D30 ? V80 : directional_remaining>D10 ? V40 : V20;
 wire [24:0] reverse_step=directional_remaining>R60 ? RV180 : directional_remaining>R20 ? RV120 : directional_remaining>R5 ? RV60 : RV30;
 wire [24:0] speed_step=negative_direction ? reverse_step : positive_step;
 wire signed [25:0] actual_minus_reference=$signed({actual[16],actual[16:0],8'd0})-$signed({reference_q8[24],reference_q8});
 wire signed [25:0] lead_room=(negative_direction ? REVERSE_LEAD : LEAD)+(negative_direction ? -actual_minus_reference : actual_minus_reference);
 wire [24:0] allowed_step=lead_room<=0 ? 25'd0 : lead_room<$signed({1'b0,speed_step}) ? lead_room[24:0] : speed_step;
 wire signed [24:0] signed_step=negative_direction ? -$signed(allowed_step) : $signed(allowed_step);
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
     if(directional_remaining<=0 || (actual_fits && directional_remaining<=$signed(allowed_step)))begin reference_q8<=goal_q8;ramping<=0;end
     else if(actual_fits)reference_q8<=reference_q8+signed_step;
   end else elapsed_ms<=elapsed_ms+1'b1;
 end
endmodule
`default_nettype wire
