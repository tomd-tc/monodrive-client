function result = ego_control_command(sample_time, x,y,yaw)
% [{'frame': [{'name': 'EgoVehicle_0',
%              'orientation': [-2.03e-05,
%                              -1.72e-05,
%                              0.719,
%                              0.694],
%              'position': [-8847.83,
%                           14146.69,
%                           11.72],
%              'tags': ['dynamic',
%                       'vehicle', 'ego'],
%              'velocity': [1.31, 0.042,
%                           14.85]}],
%   'game_time': 32920.484,
%   'time': 1549487213}]
name = "EgoVehicle";
position = [100*x, 100*y, 12.0];
velocity = [0, 0, 0];
tags = ["dynamic", "vehicle", "ego"];
orientation = struct('yaw', yaw*180/pi, 'pitch', 0, 'roll',0);
time = 1549487213;
angular_velocity = [0.0,0.0,0.0];

s = {struct("frame", {{struct('name', name, 'position', position, 'velocity',... 
            velocity, 'tags', tags, 'orientation', orientation, ...
            'angular_velocity', angular_velocity)}},'game_time', sample_time, 'time', time )};
result = s
%result = jsonencode(s);