// 触摸命中、区域刷新和像素绘制共用的矩形坐标定义，边界坐标包含端点。

function [8:0] screen_left;input [3:0] id;begin case(id)
 1:screen_left=9'd7;
 2:screen_left=9'd215;
 3:screen_left=9'd111;
 4:screen_left=9'd267;
 5:screen_left=9'd12;
 6:screen_left=9'd85;
 7:screen_left=9'd216;
 8:screen_left=9'd168;
 9:screen_left=9'd241;
 default:screen_left=0;endcase end endfunction
function [8:0] screen_right;input [3:0] id;begin case(id)
 1:screen_right=9'd105;
 2:screen_right=9'd313;
 3:screen_right=9'd209;
 4:screen_right=9'd313;
 5:screen_right=9'd79;
 6:screen_right=9'd152;
 7:screen_right=9'd262;
 8:screen_right=9'd235;
 9:screen_right=9'd308;
 default:screen_right=0;endcase end endfunction
function [7:0] screen_top;input [3:0] id;begin case(id)
 1:screen_top=8'd201;
 2:screen_top=8'd201;
 3:screen_top=8'd201;
 4:screen_top=8'd158;
 5:screen_top=8'd117;
 6:screen_top=8'd117;
 7:screen_top=8'd158;
 8:screen_top=8'd117;
 9:screen_top=8'd117;
 default:screen_top=0;endcase end endfunction
function [7:0] screen_bottom;input [3:0] id;begin case(id)
 1:screen_bottom=8'd232;
 2:screen_bottom=8'd232;
 3:screen_bottom=8'd232;
 4:screen_bottom=8'd195;
 5:screen_bottom=8'd138;
 6:screen_bottom=8'd138;
 7:screen_bottom=8'd195;
 8:screen_bottom=8'd145;
 9:screen_bottom=8'd145;
 default:screen_bottom=0;endcase end endfunction
