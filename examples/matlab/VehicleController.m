classdef VehicleController < matlab.System
    % Example vehicle controller

    % Public, tunable properties
    properties
        Kp = 1.0;
        Ki = 0.0;
        Kd = 0.0;
    end
    properties(Nontunable)
    end

    properties(DiscreteState)
        count
    end
    
    % Pre-computed constants
    properties(Access = private)
        % todo
    end
    
    methods(Access = protected)

        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.count = 0;
        end
        
        function [forward, right, brake, count] = stepImpl(obj, x)
            % todo
            time = getCurrentTime(obj);
            sts = getSampleTime(obj);
            
            forward = 1.0;
            right = 0.0;
            brake = 0.0;
            
            obj.count = obj.count + 1;
            count = obj.count;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.count = 0;
        end
        
        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end
        
    end
end