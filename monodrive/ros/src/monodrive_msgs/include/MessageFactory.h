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

        static geometry_msgs::Vector3 Vec3fToROSVector3(const Vec3f& vec) {
            geometry_msgs::Vector3 result;
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

        static Vec3f ROSVector3ToVec3f(const geometry_msgs::Vector3& vec) {
            Vec3f result;
            result.x = vec.x;
            result.y = vec.y;
            result.z = vec.z;
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
            for (auto& waypoint : actorLane.waypoints) {
                result.waypoints.push_back(WaypointToROSWaypoint(waypoint));
            }
            return result;
        }

        static ::ActorLane ROSActorLaneToActorLane(const monodrive_msgs::ActorLane& actorLane) {
            ::ActorLane result;
            result.road_id = actorLane.road_id;
            result.lane_id = actorLane.lane_id;
            for (auto& waypoint : actorLane.waypoints) {
                result.waypoints.push_back(ROSWaypointToWaypoint(waypoint));
            }
            return result;
        }

        static monodrive_msgs::ActorWaypoints ActorWaypointsToROSActorWaypoints(const ::ActorWaypoints& waypoint) {
            monodrive_msgs::ActorWaypoints result;
            result.actor_id = waypoint.actor_id;
            result.actor_road_id = waypoint.actor_road_id;
            result.actor_lane_id = waypoint.actor_lane_id;
            result.actor_waypoint = WaypointToROSWaypoint(waypoint.actor_waypoint);
            
            for (auto& lane : waypoint.lanes) {
                result.lanes.push_back(ActorLaneToROSActorLane(lane));
            }
            for (auto& lane : waypoint.left_lanes) {
                result.left_lanes.push_back(ActorLaneToROSActorLane(lane));
            }
            for (auto& lane : waypoint.right_lanes) {
                result.right_lanes.push_back(ActorLaneToROSActorLane(lane));
            }
            return result;
        }

        static ::ActorWaypoints ROSActorWaypointsToActorWaypoints(const monodrive_msgs::ActorWaypoints& waypoint) {
            ::ActorWaypoints result;
            result.actor_id = waypoint.actor_id;
            result.actor_road_id = waypoint.actor_road_id;
            result.actor_lane_id = waypoint.actor_lane_id;
            result.actor_waypoint = ROSWaypointToWaypoint(waypoint.actor_waypoint);
            
            for (auto& lane : waypoint.lanes) {
                result.lanes.push_back(ROSActorLaneToActorLane(lane));
            }
            for (auto& lane : waypoint.left_lanes) {
                result.left_lanes.push_back(ROSActorLaneToActorLane(lane));
            }
            for (auto& lane : waypoint.right_lanes) {
                result.right_lanes.push_back(ROSActorLaneToActorLane(lane));
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

        static monodrive_msgs::OOBB OOBBToROSOOBB(const ::OOBB& oobb) {
            monodrive_msgs::OOBB result;
            return result;
        }

        static monodrive_msgs::VehicleState VehicleStateToROSVehicleState(const ::VehicleState& vehicleState) {
            monodrive_msgs::VehicleState result;
            result.name = vehicleState.state.name;
            result.pose = TransformToPose(vehicleState.state.odometry.pose);
            result.twist = Vec3ToTwist(vehicleState.state.odometry.linear_velocity, vehicleState.state.odometry.angular_velocity);
            result.tags = vehicleState.state.tags;

            for (auto& wheel : vehicleState.wheels) {
                result.wheels.push_back(WheelStateToROSWheelState(wheel));
            }
            
            for (auto& oobb : vehicleState.state.oobbs) {
                result.oobbs.push_back(OOBBToROSOOBB(oobb));
            }
            return result;
        }

        static ::VehicleState ROSVehicleStateToVehicleState(const monodrive_msgs::VehicleState& vehicleState) {
            ::VehicleState result;
            /*result.name = vehicleState.name;
            result.pose = FromMonoDriveType(vehicleState.pose);
            result.twist = FromMonoDriveType(vehicleState.twist);
            result.tags = vehicleState.tags;

            for (auto& wheel : vehicleState.wheels) {
                result.wheels.push_back(FromMonoDriveType(wheel);)
            }
            
            for (auto& oobb : oobbs) {
                result.oobbs.push_back(FromMonoDriveType(oobb));
            }*/
            return result;
        }

        static sensor_msgs::Imu FromMonoDriveFrame(std::string frame_id, const ImuFrame& frame) {
            sensor_msgs::Imu message;
            message.header.stamp.sec = frame.timer / 19660800;
//            message.orientation
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
            for (auto& actor_wp : frame.actor_waypoints) {
                message.actor_waypoints.push_back(ActorWaypointsToROSActorWaypoints(actor_wp));
            }
            return message;
        }

        static WaypointFrame ToMonoDriveFrame(const monodrive_msgs::WaypointSensor& message) {
            WaypointFrame frame;
            frame.time = message.time;
            frame.game_time = message.game_time;
            frame.sample_count = message.sample_count;
            for (auto& actor_wp : message.actor_waypoints) {
                frame.actor_waypoints.push_back(ROSActorWaypointsToActorWaypoints(actor_wp));
            }
            return frame;
        }

        static monodrive_msgs::StateSensor FromMonoDriveFrame(const StateFrame& frame) {
            monodrive_msgs::StateSensor message;
            message.time = frame.time;
            message.game_time = frame.game_time;
            message.sample_count = frame.sample_count;            
            for (auto& vs : frame.vehicles) {
                message.vehicles.push_back(VehicleStateToROSVehicleState(vs));
            }
            //for (auto& objs : frame.objects) {
            //    message.objects.push_back(FromMonoDriveType(vobjs));
            //}
            return message;
        }

        static StateFrame ToMonoDriveFrame(const monodrive_msgs::StateSensor& message) {
            StateFrame frame;
            frame.time = message.time;
            frame.game_time = message.game_time;
            frame.sample_count = message.sample_count;
            for (auto& vs : message.vehicles) {
                frame.vehicles.push_back(ROSVehicleStateToVehicleState(vs));
            }
            return frame;
        }
    };

} // namespace monodrive_msgs