classdef Camera < Sensor
    % Camera Sensor
    
    % Public, non-tunable properties
    properties(Nontunable)
        config_path = 'configurations/camera.json'
    end
    
    % Pre-computed constants
    properties(Access = protected)
        image = zeros(1024,1024,3);
        width = 1024
        height = 1024
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Setup super class
            setupImpl@Sensor(obj);
            obj.width = obj.config.stream_dimensions.x;
            obj.height = obj.config.stream_dimensions.y;
        end
        
        function y = parse(obj, data)
            % parse to image
            data_length = 4*obj.width*obj.height;
            if length(data) == data_length
                r = data(3:4:end);
                redChannel = reshape(r, [obj.width,obj.height])';
                g = data(2:4:end);
                greenChannel = reshape(g, [obj.width,obj.height])';
                b = data(1:4:end);
                blueChannel = reshape(b, [obj.width,obj.height])';
                obj.image = cat(3, redChannel, greenChannel, blueChannel);
            end
            y = obj.image;
        end
        
        function resetImpl(obj)
            resetImpl@Sensor(obj);
        end
        
        function c1 = isOutputFixedSizeImpl(obj)
            c1 = true;
        end
        function c1 = isOutputComplexImpl(obj)
            c1 = false;
        end
        function sizeout = getOutputSizeImpl(obj)
            sizeout = [obj.height obj.width 3];
        end
        
        function dataout = getOutputDataTypeImpl(~)
            dataout = 'uint8';
        end
    end
end