function result = ego_control_command(forward_amount,right_amount,brake_amount)
    s = struct(...
        "forward_amount", forward_amount,...
        "right_amount", right_amount,...
        "brake_amount", brake_amount,...
        "drive_mode", 1);
    result = s;
end
