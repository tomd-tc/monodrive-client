classdef Configuration < matlab.System
    % monoDrive simulator configuration

    % Public, tunable properties
    properties
        file_path = '';
        %'configurations/simulator.json'
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
            response = obj.configure_simulator();

        end

        function stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
        end

        function response = configure_simulator(obj)
            command = obj.sim.ID_SIMULATOR_CONFIG;
            fid = fopen(obj.file_path,'r','n','UTF-8');
            config = struct(jsondecode(fscanf(fid, '%s')));
            response = obj.sim.send_message(command, config);
            fclose(fid);
        end
    end
end