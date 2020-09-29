// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

/*
 * Simulator commands are json object that request a specific action or operation
 * to be performed by the simulator. The commands have the following general format:
 *	{
 *		"type": COMMAND (string),
 *		"data": { COMMAND_SPECIFIC_DATA }
 *	}
 * 
 *  The list of supported commands are defined in this file and appear below.
 */

/*
 * Simulator configuration command
 * - configure a simulator session
 *
 * command data: 
 *	{
 *		"id": (string),
 *		"simulation_mode": (0, 1, or 2 (closed loop, replay, pxi)),
 *		"ego_config": {
 *			"camera": {
 *			}
 *		},
 *		"phys_materials": {
 *		},
 *		"traffic_configuration": {
 *		}
 *  }
 * response:
 *		"simulator configuration mode = (0, 1, or 2)"
 */
#define SimulatorConfig_ID			            "SimulatorConfig_ID"

/*
 * EGO vehicle configuration command
 * - configures the ego vehicle used in the simulation run
 */
#define EgoVehicleConfig_ID			            "EgoVehicleConfig_ID"

/*
 * EGO vehicle control command
 * - sends steering and throttle input to EGO vehicle for closed loop simulation
 *
 * command data:
 *	{
 *		"forward_amount": (float),	// throttle amount in the range -1, 1
 *		"right_amount": (float),	// steering amount in the range -1, 1
 *		"brake_amount": (float),	// brake amout in the range -1, 1
 *		"drive_mode": (int)			// -1 (reverse), 0 (neutral), 1+ (drive/gear)
 *	}
 */
#define EgoControl_ID				            "EgoControl_ID" 
#define SampleSensorsCommand_ID                 "SampleSensorsCommand_ID"
#define SampleSensorsAsyncCommand_ID                 "SampleSensorsCommand_ID"
#define SampleSensorListCommand_ID              "SampleSensorListCommand_ID"
/*
 * Activate license command
 */
#define ActivateLicense_ID			            "ActivateLicense"

/*
 * Configure weather command
 */
#define WeatherConfigCommand_ID		            "WeatherConfig"

// Replay Game Mode Commands
#define REPLAY_ConfigureSensorsCommand_ID		"REPLAY_ConfigureSensorsCommand_ID"
#define REPLAY_ReConfigureSensorCommand_ID      "REPLAY_ReConfigureSensorCommand_ID"
#define REPLAY_ConfigureTrajectoryCommand_ID	"REPLAY_ConfigureTrajectoryCommand_ID"
#define REPLAY_StepSimulationCommand_ID			"REPLAY_StepSimulationCommand_ID"
#define REPLAY_StateStepSimulationCommand_ID	"REPLAY_StateStepSimulationCommand_ID"
#define GetStartPoints_ID                       "GetStartPoints"

#define GetVersion_ID				            "GetVersion"
#define GetMapCommand_ID			            "GetMap"
#define ImportMapCommand_ID			            "ImportMap"
#define ClosedLoopConfigCommand_ID              "ClosedLoopConfigCommand_ID"
#define ClosedLoopStepCommand_ID                "ClosedLoopStepCommand_ID"

/*
 * Vehicle lights configuration command
 */
#define VehicleLightsConfigCommand_ID           "VehicleLightsConfig_ID"

 /*
  * EGO autopilot control command
  * - sends settable parameters to autopilot controller
  *
  * command data:
  *	{
  *		"set_speed": (float),	        // set speed for vehicle, in cm/s
  *		"negotiated_speed": (float),	// temporary negotiated speed change cm/s
  *		"lane_change": (int),	        // lane change request (1- passing lane, 0 - no change, -1 slower lane)
  *		"headway": (float),			      // desired headway to front vehicle (cm)
  *   "autopilot_engaged": (bool),  // true if autopilot is engaged, false otherwise
  *   "gear": (string),             // the current gear
  *   "drive_mode": (string),       // driving mode (e.g. "HWA", "Driver")
  *   "manual_override": (bool)     // whether the system is under manual override
  *	}
  */
#define AutopilotControlCommand_ID              "AutopilotControl"
