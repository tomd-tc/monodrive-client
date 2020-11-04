function result = ego_control_command(forward_amount,right_amount,brake_amount)
% simulator.send_command(ApiMessage(123, EgoControl_ID, true, 
%     {   {"forward_amount", 0.5}, 
%         {"right_amount", 0.0},
%         {"brake_amount", 0.0},
%         {"drive_mode", 1}
%     }));


s = struct("forward_amount", forward_amount, "right_amount", right_amount, "brake_amount", brake_amount,...
    "drive_mode", 1);
result = s;
%result = jsonencode(s);