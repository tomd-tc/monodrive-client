#pragma once

#include "DataFrame.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "monodrive_msgs/VehicleControl.h"
#include "monodrive_msgs/StateSensor.h"
#include "monodrive_msgs/WaypointSensor.h"


namespace monodrive_msgs
{

    class MessageFactory 
    {

        static geometry_msgs::Vector3 FromMonoDriveType(const Vec3f& vec) {
            geometry_msgs::Vector3 result;
            result.x = vec.x;
            result.y = vec.y;
            result.z = vec.z;
            return result;
        }

        static Vec3f ToMonoDriveType(const geometry_msgs::Vector3& vec) {
            Vec3f result;
            result.x = vec.x;
            result.y = vec.y;
            result.z = vec.z;
            return result;
        }


        static monodrive_msgs::Waypoint FromMonoDriveType(const ::Waypoint& waypoint) {
            monodrive_msgs::Waypoint message;
            return message;
        }

        static ::Waypoint ToMonoDriveType(const monodrive_msgs::Waypoint& message) {
            ::Waypoint waypoint;
            waypoint.location = ToMonoDriveType(message.location);
            waypoint.rotation = ToMonoDriveType(message.rotation);
            waypoint.distance = message.distance;
            waypoint.lane_change = message.lane_change;
            return waypoint;
        }

        static monodrive_msgs::ActorLane FromMonoDriveType(const ::ActorLane& actorLane) {
            monodrive_msgs::ActorLane result;
            result.road_id = actorLane.road_id;
            result.lane_id = actorLane.lane_id;
            for (auto& waypoint : actorLane.waypoints) {
                result.waypoints.push_back(FromMonoDriveType(waypoint));
            }
            return result;
        }

        static ::ActorLane ToMonoDriveType(const monodrive_msgs::ActorLane& actorLane) {
            ::ActorLane result;
            result.road_id = actorLane.road_id;
            result.lane_id = actorLane.lane_id;
            for (auto& waypoint : actorLane.waypoints) {
                result.waypoints.push_back(ToMonoDriveType(waypoint));
            }
            return result;
        }

        static monodrive_msgs::ActorWaypoints FromMonoDriveType(const ::ActorWaypoints& waypoint) {
            monodrive_msgs::ActorWaypoints result;
            result.actor_id = waypoint.actor_id;
            result.actor_road_id = waypoint.actor_road_id;
            result.actor_lane_id = waypoint.actor_lane_id;
            result.actor_waypoint = FromMonoDriveType(waypoint.actor_waypoint);
            
            for (auto& lane : waypoint.lanes) {
                result.lanes.push_back(FromMonoDriveType(lane));
            }
            for (auto& lane : waypoint.left_lanes) {
                result.left_lanes.push_back(FromMonoDriveType(lane));
            }
            for (auto& lane : waypoint.right_lanes) {
                result.right_lanes.push_back(FromMonoDriveType(lane));
            }
            return result;
        }

        static ::ActorWaypoints ToMonoDriveType(const monodrive_msgs::ActorWaypoints& waypoint) {
            ::ActorWaypoints result;
            result.actor_id = waypoint.actor_id;
            result.actor_road_id = waypoint.actor_road_id;
            result.actor_lane_id = waypoint.actor_lane_id;
            result.actor_waypoint = ToMonoDriveType(waypoint.actor_waypoint);
            
            for (auto& lane : waypoint.lanes) {
                result.lanes.push_back(ToMonoDriveType(lane));
            }
            for (auto& lane : waypoint.left_lanes) {
                result.left_lanes.push_back(ToMonoDriveType(lane));
            }
            for (auto& lane : waypoint.right_lanes) {
                result.right_lanes.push_back(ToMonoDriveType(lane));
            }
            return result;
        }

        static sensor_msgs::VehiceState FromMonoDriveType(const ::VehicleState& vehicleState) {
            sensor_msgs::VehiceState result;
            result.name = vehicleState.name;
            result.pose = FromMonoDriveType(vehicleState.pose);
            result.twist = FromMonoDriveType(vehicleState.twist);
            result.tags = vehicleState.tags;

            for (auto& wheel : vehicleState.wheels) {
                result.wheels.push_back(FromMonoDriveType(wheel);)
            }
            
            for (auto& oobb : oobbs) {
                result.oobbs.push_back(FromMonoDriveType(oobb));
            }
            return result;
        }

        static ::VehiceState ToMonoDriveType(const monodrive_msgs::VehicleState& vehicleState) {
            ::VehiceState result;
            result.name = vehicleState.name;
            result.pose = FromMonoDriveType(vehicleState.pose);
            result.twist = FromMonoDriveType(vehicleState.twist);
            result.tags = vehicleState.tags;

            for (auto& wheel : vehicleState.wheels) {
                result.wheels.push_back(FromMonoDriveType(wheel);)
            }
            
            for (auto& oobb : oobbs) {
                result.oobbs.push_back(FromMonoDriveType(oobb));
            }
            return result;
        }

        static sensor_msgs::Imu FromMonoDriveFrame(std::string frame_id, const ImuFrame& frame) {
            sensor_msgs::Imu message;
            message.header.stamp.sec = frame.timer / 19660800;
//            message.orientation
            message.orientation_covariance = {0,0,0,0,0,0,0,0,0}; 
            message.angular_velocity = FromMonoDriveType(frame.angular_velocity);
            message.angular_velocity_covariance = { 0,0,0,0,0,0,0,0,0 };

            message.linear_acceleration = FromMonoDriveType(frame.acceleration);
            message.linear_acceleration_covariance = { 0,0,0,0,0,0,0,0,0 }; 
            return message;
        }

        static ImuFrame ToMonoDriveFrame(const sensor_msgs::Imu& message) {
            ImuFrame frame;
            frame.angular_velocity = ToMonoDriveType(message.angular_velocity);

            frame.acceleration = ToMonoDriveType(message.linear_acceleration);

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
            for (auto& actor_wp : frame.actor_waypoints) {
                message.actor_waypoints.push_back(FromMonoDriveType(actor_wp));
            }
            return message;
        }

        static WaypointFrame ToMonoDriveFrame(const monodrive_msgs::WaypointSensor& message) {
            WaypointFrame frame;
            frame.time = message.time;
            frame.game_time = message.game_time;
            frame.sample_count = message.sample_count;
            for (auto& actor_wp : message.actor_waypoints) {
                frame.actor_waypoints.push_back(ToMonoDriveType(actor_wp));
            }
            return frame;
        }

        static monodrive_msgs::StateSensor FromMonoDriveFrame(const StateFrame& frame) {
            monodrive_msgs::StateSensor message;
            message.time = frame.time;
            message.game_time = frame.game_time;
            message.sample_count = frame.sample_count;            
            for (auto& vs : frame.vehicles) {
                message.vehicles.push_back(FromMonoDriveType(vs));
            }
            return message;
        }

        static StateFrame ToMonoDriveFrame(const monodrive_msgs::StateSensor& message) {
            StateFrame frame;
            frame.time = message.time;
            frame.game_time = message.game_time;
            frame.sample_count = message.sample_count;
            for (auto& vs : message.vehicles) {
                frame.vehicles.push_back(ToMonoDriveType(vs));
            }
            return frame;
        }
    };

} // namespace monodrive_msgs