
coder.extrinsic('json_read');
sim_config_path = '../configurations/simulator.json';
sim_config  = json_read(sim_config_path);

vehicle_config_path = '../configurations/vehicle.json';
vehicle_config = json_read(vehicle_config_path);

scenario_config_path = '../configurations/trajectories/Close_Loop.json';
scenario_config = json_read(scenario_config_path);

global mono;
mono = Simulator();
mono.initialize(sim_config);
mono.configure_simulator(sim_config);
mono.configure_scenario(scenario_config);

% vehicle = monoDriveVehicle();
% vehicle.server_ip = sim_config.server_ip;
% vehicle.server_port = sim_config.server_port;
% vehicle.setup();
% % 
% cam = CameraSensor();
% cam.server_ip = sim_config.server_ip;
% cam.server_port = sim_config.server_port;
% cam.setup();
% % % 
% vp_cam = Viewport_Camera();
% vp_cam.server_ip = sim_config.server_ip;
% vp_cam.server_port = sim_config.server_port;
% vp_cam.setup();
% % 
% for n = 1:100
%        vehicle.step(1.0,0.0,0.0);
%        cam.step();
% end

