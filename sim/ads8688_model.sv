// 离线仿真与行为参考源码，保留相关激励、模型或预期行为，不参与硬件工程综合。
`timescale 1ns/1ps

module ads8688_model #(parameter CHANNEL=0)(
    input wire cs_n, sclk, sdi, host_rst_n,
    input wire [1:0] fault,
    output wire sdo
);
    reg output_bit=0;
    assign sdo=(fault==1)?1'b0:(fault==2)?1'b1:(!cs_n?output_bit:1'bz);
    reg [7:0] registers[0:127];
    reg [15:0] command, reply, conversion;
    integer edges=0, selected=-1, conversion_index=0, address, i;
    integer manual_count=0, frame_count=0;
    time last_rise=0, last_fall=0;
    function [15:0] pattern(input integer index);
        case (index%8)
            0: pattern=16'h0000;
            1: pattern=16'h0001;
            2: pattern=16'h7fff;
            3: pattern=16'h8000;
            4: pattern=16'ha500;
            5: pattern=16'hffff;
            6: pattern=16'h1234;
            7: pattern=16'haaaa;
        endcase
    endfunction
    initial for (i=0;i<128;i=i+1) registers[i]=0;
    always @(negedge cs_n) begin
        if (last_rise && $time-last_rise<50) $fatal(1,"CS high interval too short");
        edges=0; command=0; output_bit=0;
        conversion=(selected==CHANNEL)?pattern(conversion_index):16'hdead;
    end
    always @(posedge sclk) begin
        if (cs_n) $fatal(1,"SCLK toggled while CS high");
    end
    always @(negedge sclk) if (!cs_n) begin
        edges=edges+1;
        if (edges<=16) command={command[14:0],sdi};
        if (edges==16) begin
            if (command[15]==0 && command!=0) begin
                address=command[15:9];
                reply={command[8]?command[7:0]:registers[address],8'b0};
            end else reply=conversion;
        end

        if (edges>=16 && edges<=31) output_bit <= #20 reply[31-edges];
        else output_bit<=#20 0;
        last_fall=$time;
    end
    always @(posedge cs_n) begin
        if (edges!=0 && host_rst_n) begin
            if (edges!=32) $fatal(1,"Expected 32 clocks, got %0d",edges);
            if ($time-last_fall<50) $fatal(1,"CS hold too short");
            frame_count=frame_count+1;
            if (command==16'h8500) begin
                for(i=0;i<128;i=i+1) registers[i]=0;
                selected=-1; conversion_index=0;
            end else if (command[15:13]==3'b110) begin
                selected=command[12:10]; manual_count=manual_count+1;
                if (selected!=CHANNEL) $fatal(1,"Wrong manual channel %0d",selected);
            end else if (command[15]==0 && command!=0 && command[8])
                registers[command[15:9]]=command[7:0];
            else if (command==0 && selected>=0) conversion_index=conversion_index+1;
        end
        last_rise=$time;
    end
endmodule
