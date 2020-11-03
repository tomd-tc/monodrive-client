classdef Trajectory < matlab.System
    % Untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        file_path = ''
        
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        sim = libpointer
    end
    methods(Access = protected)
        function response = setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.sim = Simulator();
            obj.sim.initialize();
            response = obj.sim.configure_scenario();
            %response = obj.send_trajectory();

        end

        function stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
        end

        function response = send_trajectory(obj)
            command = obj.sim.ID_REPLAY_CONFIGURE_TRAJECTORY_COMMAND;
            fid = fopen(obj.file_path,'r','n','UTF-8');
            config = struct(jsondecode(fscanf(fid, '%s')));
            response = obj.sim.send_message(command, config);
            fclose(fid);
        end
    end
end
