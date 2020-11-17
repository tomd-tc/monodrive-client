classdef Perception < matlab.System
    %PERCEPTION Example image perception system
    %   Sample image perception to simply show camera data
    
    properties
        draw_annotations = false;
    end
    
    methods
        function obj = Perception
            %PERCEPTION Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
    methods (Access = protected)
        function setupImpl(obj)
            % initialize
        end
        function stepImpl(obj, image)
            % display image data
            imshow(image);
        end
    end
end

