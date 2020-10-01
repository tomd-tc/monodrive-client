// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include "DataFrame.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "monodrive_msgs/VehicleControl.h"
#include "monodrive_msgs/StateSensor.h"
#include "monodrive_msgs/WaypointSensor.h"
#include "monodrive_msgs/Radar.h"


namespace monodrive_msgs
{

    // This class provides a number of static methods that help convert
    // monoDrive sensor frame data messages to their ROS counterparts, and 
    // vice-versa
    class MessageFactory 
    {
    public:
        static monodrive_msgs::Complex complexToROSComplex(const std::complex<float> value) {
            monodrive_msgs::Complex result;
            result.real = std::real(value);
            result.imag = std::imag(value);
            return result;
        }

        static std::complex<float> ROSComplexTocomplex(const monodrive_msgs::Complex& value) {
            std::complex<float> result(value.real,value.imag);
            return result;
        }

        static geometry_msgs::Vector3 Vec3fToROSVector3(const Vec3f& vec) {
            geometry_msgs::Vector3 result;
            result.x = vec.x;
            result.y = vec.y;
            result.z = vec.z;
            return result;
        }

        static Vec3f ROSVector3ToVec3f(const geometry_msgs::Vector3& vec) {
            Vec3f result;
            result.x = vec.x;
            result.y = vec.y;
            result.z = vec.z;
            return result;
        }

        static geometry_msgs::Point Vec3fToROSPoint(const Vec3f& vec) {
            geometry_msgs::Point result;
            result.x = vec.x;
            result.y = vec.y;
            result.z = vec.z;
            return result;
        }

        static Vec3f ROSPointToVec3f(const geometry_msgs::Point& point) {
            Vec3f result;
            result.x = point.x;
            result.y = point.y;
            result.z = point.z;
            return result;
        }

        static geometry_msgs::Quaternion QuatToROSQuaternion(const ::Quat& quat) {
            geometry_msgs::Quaternion result;
            result.x = quat.x;
            result.y = quat.y;
            result.z = quat.z;
            result.w = quat.w;
            return result;
        }

        static ::Quat ROSQuaternionToQuat(const geometry_msgs::Quaternion& quat) {
            ::Quat result;
            result.x = quat.x;
            result.y = quat.y;
            result.z = quat.z;
            result.w = quat.w;
            return result;
        }

        static geometry_msgs::Transform TransformToROSTransform(const ::Transform& transform) {
            geometry_msgs::Transform result;
            result.translation = Vec3fToROSVector3(transform.position);
            result.rotation = QuatToROSQuaternion(transform.orientation);
            return result;
        }

        static ::Transform ROSTransformToTransform(const geometry_msgs::Transform& transform) {
            ::Transform result;
            result.position = ROSVector3ToVec3f(transform.translation);
            result.orientation = ROSQuaternionToQuat(transform.rotation);
            return result;
        }

        static geometry_msgs::PoseWithCovariance TransformToPose(const ::Transform& transform) {
            geometry_msgs::PoseWithCovariance result;
            result.pose.position = Vec3fToROSPoint(transform.position);
            result.pose.orientation = QuatToROSQuaternion(transform.orientation);
            return result;
        }

        static ::Transform ROSPoseToTransform(const geometry_msgs::PoseWithCovariance& pose) {
            ::Transform result;
            result.position = ROSPointToVec3f(pose.pose.position);
            result.orientation = ROSQuaternionToQuat(pose.pose.orientation);
            return result;
        }

        static geometry_msgs::TwistWithCovariance Vec3ToTwist(const ::Vec3f& linear_velocity, const ::Vec3f& angular_velocity) {
            geometry_msgs::TwistWithCovariance result;
            result.twist.linear = Vec3fToROSVector3(linear_velocity);
            result.twist.angular = Vec3fToROSVector3(angular_velocity);
            return result;
        }

        static monodrive_msgs::Waypoint WaypointToROSWaypoint(const ::Waypoint& waypoint) {
            monodrive_msgs::Waypoint message;
            message.lane_change = waypoint.lane_change;
            message.distance = waypoint.distance;
            message.location = Vec3fToROSVector3(waypoint.location);
            message.rotation = Vec3fToROSVector3(waypoint.rotation);
            return message;
        }

        static ::Waypoint ROSWaypointToWaypoint(const monodrive_msgs::Waypoint& message) {
            ::Waypoint waypoint;
            waypoint.location = ROSVector3ToVec3f(message.location);
            waypoint.rotation = ROSVector3ToVec3f(message.rotation);
            waypoint.distance = message.distance;
            waypoint.lane_change = message.lane_change;
            return waypoint;
        }

        static monodrive_msgs::ActorLane ActorLaneToROSActorLane(const ::ActorLane& actorLane) {
            monodrive_msgs::ActorLane result;
            result.road_id = actorLane.road_id;
            result.lane_id = actorLane.lane_id;
            result.waypoints.resize(actorLane.waypoints.size());
            for (int i = 0; i < actorLane.waypoints.size(); i++) {
                result.waypoints[i] = WaypointToROSWaypoint(actorLane.waypoints[i]);
            }
            return result;
        }

        static ::ActorLane ROSActorLaneToActorLane(const monodrive_msgs::ActorLane& actorLane) {
            ::ActorLane result;
            result.road_id = actorLane.road_id;
            result.lane_id = actorLane.lane_id;
            result.waypoints.resize(actorLane.waypoints.size());
            for (int i = 0; i < actorLane.waypoints.size(); i++) {
                result.waypoints[i] = ROSWaypointToWaypoint(actorLane.waypoints[i]);
            }
            return result;
        }

        static monodrive_msgs::ActorWaypoints ActorWaypointsToROSActorWaypoints(const ::ActorWaypoints& waypoint) {
            monodrive_msgs::ActorWaypoints result;
            result.actor_id = waypoint.actor_id;
            result.actor_road_id = waypoint.actor_road_id;
            result.actor_lane_id = waypoint.actor_lane_id;
            result.actor_waypoint = WaypointToROSWaypoint(waypoint.actor_waypoint);
            
            result.lanes.resize(waypoint.lanes.size());
            for (int i = 0; i < waypoint.lanes.size(); i++) {
                result.lanes[i] = ActorLaneToROSActorLane(waypoint.lanes[i]);
            }

            result.left_lanes.resize(waypoint.left_lanes.size());
            for (int i = 0; i < waypoint.left_lanes.size(); i++) {
                result.left_lanes[i] = ActorLaneToROSActorLane(waypoint.left_lanes[i]);
            }

            result.right_lanes.resize(waypoint.right_lanes.size());
            for (int i = 0; i < waypoint.right_lanes.size(); i++) {
                result.right_lanes[i] = ActorLaneToROSActorLane(waypoint.right_lanes[i]);
            }
            return result;
        }

        static ::ActorWaypoints ROSActorWaypointsToActorWaypoints(const monodrive_msgs::ActorWaypoints& waypoint) {
            ::ActorWaypoints result;
            result.actor_id = waypoint.actor_id;
            result.actor_road_id = waypoint.actor_road_id;
            result.actor_lane_id = waypoint.actor_lane_id;
            result.actor_waypoint = ROSWaypointToWaypoint(waypoint.actor_waypoint);
            
            result.lanes.resize(waypoint.lanes.size());
            for (int i = 0; i < waypoint.lanes.size(); i++) {
                result.lanes[i] = ROSActorLaneToActorLane(waypoint.lanes[i]);
            }

            result.left_lanes.resize(waypoint.left_lanes.size());
            for (int i = 0; i < waypoint.left_lanes.size(); i++) {
                result.left_lanes[i] = ROSActorLaneToActorLane(waypoint.left_lanes[i]);
            }

            result.right_lanes.resize(waypoint.right_lanes.size());
            for (int i = 0; i < waypoint.right_lanes.size(); i++) {
                result.right_lanes[i] = ROSActorLaneToActorLane(waypoint.right_lanes[i]);
            }
            return result;
        }

        static monodrive_msgs::WheelState WheelStateToROSWheelState(const ::WheelState& wheelState) {
            monodrive_msgs::WheelState result;
            result.id = wheelState.id;
            result.speed = wheelState.speed;
            result.transform = TransformToROSTransform(wheelState.pose);
            return result;
        }

        static ::WheelState ROSWheelStateToWheelState(const monodrive_msgs::WheelState& wheelState) {
            ::WheelState result;
            result.id = wheelState.id;
            result.speed = wheelState.speed;
            result.pose = ROSTransformToTransform(wheelState.transform);
            return result;
        }

        static monodrive_msgs::OOBB OOBBToROSOOBB(const ::OOBB& oobb) {
            monodrive_msgs::OOBB result;
            result.name = oobb.name;
            result.orientation = QuatToROSQuaternion(oobb.orientation);
            result.center = Vec3fToROSVector3(oobb.center);
            result.scale = Vec3fToROSVector3(oobb.scale);
            result.extents = Vec3fToROSVector3(oobb.extents);
            return result;
        }

        static ::OOBB ROSOOBBToOOBB(const monodrive_msgs::OOBB& oobb) {
            ::OOBB result;
            result.name = oobb.name;
            result.orientation = ROSQuaternionToQuat(oobb.orientation);
            result.center = ROSVector3ToVec3f(oobb.center);
            result.scale = ROSVector3ToVec3f(oobb.scale);
            result.extents = ROSVector3ToVec3f(oobb.extents);
            return result;
        }

        static monodrive_msgs::VehicleState VehicleStateToROSVehicleState(const ::VehicleState& vehicleState) {
            monodrive_msgs::VehicleState result;
            result.name = vehicleState.state.name;
            result.pose = TransformToPose(vehicleState.state.odometry.pose);
            result.twist = Vec3ToTwist(vehicleState.state.odometry.linear_velocity, vehicleState.state.odometry.angular_velocity);
            result.tags = vehicleState.state.tags;

            result.wheels.resize(vehicleState.wheels.size());
            for (int i = 0; i < vehicleState.wheels.size(); i++) {
                result.wheels[i] = WheelStateToROSWheelState(vehicleState.wheels[i]);
            }
            
            result.oobbs.resize(vehicleState.state.oobbs.size());
            for (int i = 0; i < vehicleState.state.oobbs.size(); i++) {
                result.oobbs[i] = OOBBToROSOOBB(vehicleState.state.oobbs[i]);
            }
            return result;
        }

        static ::VehicleState ROSVehicleStateToVehicleState(const monodrive_msgs::VehicleState& vehicleState) {
            ::VehicleState result;
            result.state.name = vehicleState.name;
            result.state.odometry.pose = ROSPoseToTransform(vehicleState.pose);
            result.state.odometry.linear_velocity = ROSVector3ToVec3f(vehicleState.twist.twist.linear);
            result.state.odometry.angular_velocity = ROSVector3ToVec3f(vehicleState.twist.twist.angular);
            result.state.tags = vehicleState.tags;

            result.wheels.resize(vehicleState.wheels.size());
            for (int i = 0; i < vehicleState.wheels.size(); i++) {
                result.wheels[i] = ROSWheelStateToWheelState(vehicleState.wheels[i]);
            }
            
            result.state.oobbs.resize(vehicleState.oobbs.size());
            for (int i = 0; i < vehicleState.oobbs.size(); i++) {
                result.state.oobbs[i] = ROSOOBBToOOBB(vehicleState.oobbs[i]);
            }
            return result;
        }

        static monodrive_msgs::RadarTarget RadarTargetToROSRadarTarget(const ::RadarTarget& radarTarget) {
            monodrive_msgs::RadarTarget result;
            result.range = radarTarget.range;
            result.aoa = radarTarget.aoa;
            result.velocity = radarTarget.velocity;
            result.rcs = radarTarget.rcs;
            result.target_ids = radarTarget.target_ids;
            return result;
        }

        static ::RadarTarget ROSRadarTargetToRadarTarget(const monodrive_msgs::RadarTarget& radarTarget) {
            ::RadarTarget result;
            result.range = radarTarget.range;
            result.aoa = radarTarget.aoa;
            result.velocity = radarTarget.velocity;
            result.rcs = radarTarget.rcs;
            result.target_ids = radarTarget.target_ids;
            return result;
        }

        static monodrive_msgs::RadarCube RadarCubeFrameToROSRadarCube(const ::RadarCubeFrame& radarCubeFrame) {
            monodrive_msgs::RadarCube result;
            result.num_sweeps = radarCubeFrame.numSweeps;
            result.num_samples_per_sweep = radarCubeFrame.numSamplesPerSweep;
            result.num_elements = radarCubeFrame.numElements;

            result.radar_cube.resize(radarCubeFrame.radar_cube.size());
            for (int i = 0; i < radarCubeFrame.radar_cube.size(); i++) {
                result.radar_cube[i] = complexToROSComplex(radarCubeFrame.radar_cube[i]);
            }
            return result;
        }

        static ::RadarCubeFrame ROSRadarCubeToRadarCubeFrame(const monodrive_msgs::RadarCube& radarCube) {
            ::RadarCubeFrame result(radarCube.num_sweeps, radarCube.num_samples_per_sweep, radarCube.num_elements);

            result.radar_cube.resize(radarCube.radar_cube.size());
            for (int i = 0; i < radarCube.radar_cube.size(); i++) {
                result.radar_cube[i] = ROSComplexTocomplex(radarCube.radar_cube[i]);
            }
            return result;
        }

        static monodrive_msgs::RadarTargetList RadarTargetListFrameToROSRadarTargetList(const ::RadarTargetListFrame& radarTargetListFrame) {
            monodrive_msgs::RadarTargetList result;

            result.targets.resize(radarTargetListFrame.targets.size());
            for (int i = 0; i < radarTargetListFrame.targets.size(); i++) {
                result.targets[i] = RadarTargetToROSRadarTarget(radarTargetListFrame.targets[i]);
            }

            result.gt_targets.resize(radarTargetListFrame.gt_targets.size());
            for (int i = 0; i < radarTargetListFrame.gt_targets.size(); i++) {
                result.gt_targets[i] = RadarTargetToROSRadarTarget(radarTargetListFrame.gt_targets[i]);
            }
            return result;
        }

        static ::RadarTargetListFrame ROSRadarTargetListToRadarTargetListFrame(const monodrive_msgs::RadarTargetList& radarTargetList) {
            ::RadarTargetListFrame result;

            result.targets.resize(radarTargetList.targets.size());
            for (int i = 0; i < radarTargetList.targets.size(); i++) {
                result.targets[i] = ROSRadarTargetToRadarTarget(radarTargetList.targets[i]);
            }

            result.gt_targets.resize(radarTargetList.gt_targets.size());
            for (int i = 0; i < radarTargetList.gt_targets.size(); i++) {
                result.gt_targets[i] = ROSRadarTargetToRadarTarget(radarTargetList.gt_targets[i]);
            }
            return result;
        }

        static sensor_msgs::Imu FromMonoDriveFrame(const ImuFrame& frame) {
            sensor_msgs::Imu message;
            message.orientation_covariance = {0,0,0,0,0,0,0,0,0}; 
            message.angular_velocity = Vec3fToROSVector3(frame.angular_velocity);
            message.angular_velocity_covariance = { 0,0,0,0,0,0,0,0,0 };

            message.linear_acceleration = Vec3fToROSVector3(frame.acceleration);
            message.linear_acceleration_covariance = { 0,0,0,0,0,0,0,0,0 }; 
            return message;
        }

        static ImuFrame ToMonoDriveFrame(const sensor_msgs::Imu& message) {
            ImuFrame frame;
            frame.angular_velocity = ROSVector3ToVec3f(message.angular_velocity);

            frame.acceleration = ROSVector3ToVec3f(message.linear_acceleration);

            frame.timer = message.header.stamp.sec * 19660800;
            frame.checksum = 0;
            frame.time_of_week = 0;
            return frame;
        }

        static monodrive_msgs::WaypointSensor FromMonoDriveFrame(const WaypointFrame& frame) {
            monodrive_msgs::WaypointSensor message;
            message.time = frame.time;
            message.game_time = frame.game_time;
            message.sample_count = frame.sample_count;

            message.actor_waypoints.resize(frame.actor_waypoints.size());
            for (int i = 0; i < frame.actor_waypoints.size(); i++) {
                message.actor_waypoints[i] = ActorWaypointsToROSActorWaypoints(frame.actor_waypoints[i]);
            }
            return message;
        }

        static WaypointFrame ToMonoDriveFrame(const monodrive_msgs::WaypointSensor& message) {
            WaypointFrame frame;
            frame.time = message.time;
            frame.game_time = message.game_time;
            frame.sample_count = message.sample_count;

            frame.actor_waypoints.resize(message.actor_waypoints.size());
            for (int i = 0; i < message.actor_waypoints.size(); i++) {
                frame.actor_waypoints[i] = ROSActorWaypointsToActorWaypoints(message.actor_waypoints[i]);
            }
            return frame;
        }

        static monodrive_msgs::StateSensor FromMonoDriveFrame(const StateFrame& frame) {
            monodrive_msgs::StateSensor message;
            message.time = frame.time;
            message.game_time = frame.game_time;
            message.sample_count = frame.sample_count;

            message.vehicles.resize(frame.vehicles.size());
            for (int i = 0; i < frame.vehicles.size(); i++) {
                message.vehicles[i] = VehicleStateToROSVehicleState(frame.vehicles[i]);
            }

            //message.objects.resize(frame.objects.size());
            //for (int i = 0; i < frame.objects.size(); i++) {
            //    message.objects[i] = FromMonoDriveType(frame.objects[i]);
            //}
            return message;
        }

        static StateFrame ToMonoDriveFrame(const monodrive_msgs::StateSensor& message) {
            StateFrame frame;
            frame.time = message.time;
            frame.game_time = message.game_time;
            frame.sample_count = message.sample_count;

            frame.vehicles.resize(message.vehicles.size());
            for (int i = 0; i < message.vehicles.size(); i++) {
                frame.vehicles[i] = ROSVehicleStateToVehicleState(message.vehicles[i]);
            }
            return frame;
        }

        static sensor_msgs::NavSatFix FromMonoDriveFrame(const GPSFrame& frame) {
            sensor_msgs::NavSatFix message;
            message.latitude = frame.lattitude;
            message.longitude = frame.longitude;
            message.altitude = frame.elevation;
            message.position_covariance = {0,0,0,0,0,0,0,0,0};
            message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
            return message;
        }

        static GPSFrame ToMonoDriveFrame(const sensor_msgs::NavSatFix& message) {
            GPSFrame frame;
            frame.lattitude = message.latitude;
            frame.longitude = message.longitude;
            frame.elevation = message.altitude;
            return frame;
        }

        static monodrive_msgs::Radar FromMonoDriveFrame(const RadarFrame& frame) {
            monodrive_msgs::Radar message;
            message.has_radar_cube = frame.bSendRadarCube;
            message.current_frame_index = frame.currentFrameIndex;
            message.radar_target_list_frame = RadarTargetListFrameToROSRadarTargetList(*frame.radarTargetListFrame);
            message.radar_cube_frame = RadarCubeFrameToROSRadarCube(*frame.radarCubeFrame);
            return message;
        }

        static RadarFrame ToMonoDriveFrame(const monodrive_msgs::Radar& message) {
            RadarFrame frame(message.has_radar_cube, 
                message.radar_cube_frame.num_sweeps, 
                message.radar_cube_frame.num_samples_per_sweep, 
                message.radar_cube_frame.num_elements);
            frame.currentFrameIndex = message.current_frame_index;
            *frame.radarTargetListFrame = ROSRadarTargetListToRadarTargetListFrame(message.radar_target_list_frame);
            *frame.radarCubeFrame = ROSRadarCubeToRadarCubeFrame(message.radar_cube_frame);
            return frame;
        }

    };

} // namespace monodrive_msgs