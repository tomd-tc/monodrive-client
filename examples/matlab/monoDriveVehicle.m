classdef monoDriveVehicle < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    % Example usage
    
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    
    properties
        
    end
    properties(Nontunable)
        SampleTime = 1.4; % Sample Time
        OffsetTime = 0.2; % Offset Time
        TickTime = 0.1;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    properties(DiscreteState)
        Count
    end
    
    % Pre-computed constants
    properties(Access = private)
        sim= libpointer
    end
    
    
    methods(Access = protected)
        function sts = getSampleTimeImpl(obj)
            switch char(obj.SampleTimeTypeProp)
                case 'Inherited'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'InheritedNotControllable'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'AlternatePropagation','Controllable');
                case 'InheritedErrorConstant'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'ErrorOnPropagation','Constant');
                case 'FixedInMinorStep'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete'
                    sts = createSampleTime(obj,'Type','Discrete',...
                        'SampleTime',obj.SampleTime, ...
                        'OffsetTime',obj.OffsetTime);
                case 'Controllable'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.Count = 0;
            obj.sim = Simulator();
            obj.sim.initialize();
        end
        
        function Count = stepImpl(obj, forward_amount, right_amount, brake_amount)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            obj.Count = obj.Count + 1;
            Time = getCurrentTime(obj);
            sts = getSampleTime(obj);
            if strcmp(sts.Type,'Controllable')
                setNumTicksUntilNextHit(obj,obj.Count);
            end
            obj.sim.step_vehicle(forward_amount, right_amount, brake_amount);
            %obj.sim.sample_sensors();
            Count = obj.Count;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.Count = 0;
        end
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(~,name)
            if strcmp(name,'Count')
                sz = [1 1];
                dt = 'double';
                cp = false;
            else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end
        function dataout = getOutputDataTypeImpl(~)
            dataout = 'double';
        end
        function sizeout = getOutputSizeImpl(~)
            sizeout = [1 1];
        end
        function cplxout = isOutputComplexImpl(~)
            cplxout = false;
        end
        function fixedout = isOutputFixedSizeImpl(~)
            fixedout = true;
        end
        function flag = isInputSizeMutableImpl(~,idx)
            if idx == 1
                flag = true;
            else
                flag = false;
            end
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