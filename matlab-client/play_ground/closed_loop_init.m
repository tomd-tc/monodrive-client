
coder.extrinsic('json_read');
sim_config_path = 'configurations/simulator.json';
sim_config  = json_read(sim_config_path);

vehicle_config_path = 'configurations/vehicle.json';
vehicle_config = json_read(vehicle_config_path);

scenario_config_path = 'configurations/trajectories/scenario_config_single_vehicle.json';
scenario_config = json_read(scenario_config_path);

global mono;
mono = Simulator();
mono.initialize(sim_config);
mono.configure_simulator(sim_config);
mono.configure_scenario(scenario_config);
