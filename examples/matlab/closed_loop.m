%% closed loop example
% example matlab script which configures the simulator in closed loop
% mode with a camera sensor.
sim = Simulator()
sim.setup()
%
cam = Camera();
cam.setup();
%
vp_cam = ViewportCamera();
vp_cam.setup();

for n = 1:100
    sim.step(0.2, 0.0, 0.0);
    disp(n);
    im = cam.step();
    imshow(im);
end
sim.step_vehicle(0.0, 0.0, 1.0);
