sim = Simulator()
sim.initialize()
sim.configure()

vehicle = monoDriveVehicle();
%
vehicle.setup();
%
cam = Camera();
cam.setup();
%
vp_cam = Viewport_Camera();
vp_cam.setup();

for n = 1:100
    vehicle.step(0.1, 0.0, 0.0);
    sim.sample_sensors();
    disp(n);
    cam.step();
end
vehicle.step(0.0, 0.0, 1.0);
