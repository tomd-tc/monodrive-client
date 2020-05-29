mono = Simulator();
mono.initialize();
mono.configure_simulator();
mono.configure_scenario();

vehicle = monoDriveVehicle();
% 
vehicle.setup();
% 
cam = Camera();
cam.setup();
% % 
vp_cam = Viewport_Camera();
vp_cam.setup();
% 
for n = 1:100
       vehicle.step(0.0,0.0,0.0);
       mono.sample_sensors();
       pause(.1)
       cam.step();
       
end