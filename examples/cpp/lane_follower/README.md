# Lane follower example

This directory contains various example implementations of a simple lane follower and
speed controller for the ego vehicle in the monoDrive Simulator.


### Setup and build
Follow instruction in the root directory of this repository.


**Note** that on Windows, these examples will be built to the `Debug` or `Release` folder`
```
./build/examples/cpp/lane_follower/Release/real_time.exe
```


### Lane follower - real time
The simulator is configured in closed loop mode. The exported lane splines (in geojson format)
are loaded and used as a map. The state sensor is used for localization. A simple lane
follower algorithm is used to steer the vehicle along the lane spline. A PID controller
is used for speed control.

Run example
```
./build/examples/cpp/lane_follower/real_time
```



### Lane follower - fixed time step
The simulator is configured in fixed time step mode. Otherwise this example is the same as
described above.

Run example
```
./build/examples/cpp/lane_follower/fixed_step
```


### Lane follower - lidar
This example also configures a Lidar sensor and forwards packets for visualization in VeloView. Otherwise this is the same as the fixed time step example described above.

Run example
```
./build/examples/cpp/lane_follower/fixed_step_lidar
```


### Lane follower - speed curve
This example uses the PID controller with a time series of target speeds to reproduce a speed curve.

Currently the example uses the mock data found in `speed_curve_data.csv`. This can be replaced with your own
data in order to compare or calibrate the vehicle physics model used in the simulator.

Run example
```
./build/examples/cpp/lane_follower/speed_curve
```


### Waypoint follower - batch job

The simulator is configured in closed loop mode. The Waypoint sensor is used as a local map
for lane information. The state sensor is used for localization. A simple lane
follower algorithm is used to steer the vehicle along the waypoints. A PID controller
is used for speed control.

This example also uses the `Jobs` module to support local and cloud batch processing. Job
info and configuration are received programmatically. A lane deviation threshold is used to
trigger early stopping. Results are written with metrics and pass/fail info.


Run single job for development
```
./build/examples/cpp/lane_follower/waypoints --md_simulator examples/config/simulator_straightaway.json --md_scenario examples/config/scenario_multi_vehicle_straightaway.json --md_weather examples/config/weather.json --md_results ./test_results.json
```

Run continuously for local batch processing
```
./build/examples/cpp/lane_follower/waypoints --md_assets ./assets_dir --md_loop
```

For more usage info on the `Jobs` module
```
./build/examples/cpp/lane_follower/waypoints --md_help
```
