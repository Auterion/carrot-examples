#pragma once

#include <algorithm> 

#define M_PI_F 3.14159265f

namespace math
{
	namespace trajectory
	{

		struct VehicleDynamicLimits
		{
			float z_accept_rad;
			float xy_accept_rad;

			float max_acc_xy;
			float max_jerk;

			float max_speed_xy;

			// TODO: remove this
			float max_acc_xy_radius_scale;
		};

		/*
		 * Compute the maximum allowed speed at the waypoint assuming that we want to
		 * connect the two lines (current-target and target-next)
		 * with a tangent circle with constant speed and desired centripetal acceleration: a_centripetal = speed^2 / radius
		 * The circle should in theory start and end at the intersection of the lines and the waypoint's acceptance radius.
		 * This is not exactly true in reality since Navigator switches the waypoint so we have to take in account that
		 * the real acceptance radius is smaller.
		 *
		 */
		inline float computeStartXYSpeedFromWaypoints(const Eigen::Vector3f &start_position, const Eigen::Vector3f &target,
													  const Eigen::Vector3f &next_target, float exit_speed, const VehicleDynamicLimits &config)
		{

			Eigen::Vector2f xy_diff = target.head<2>() - next_target.head<2>();
			const float distance_target_next = xy_diff.norm();

			const bool target_next_different = distance_target_next > 0.001f;
			const bool waypoint_overlap = distance_target_next < config.xy_accept_rad;
			const bool has_reached_altitude = std::fabs(target(2) - start_position(2)) < config.z_accept_rad;
			const bool altitude_stays_same = std::fabs(next_target(2) - target(2)) < config.z_accept_rad;

			float speed_at_target = 0.0f;

			if (target_next_different &&
				!waypoint_overlap &&
				has_reached_altitude &&
				altitude_stays_same)
			{

				Eigen::Vector2f target_start = (target - start_position).head<2>().normalized();
				Eigen::Vector2f target_next = (target - next_target).head<2>().normalized();
				const float alpha = acosf(target_start.dot(target_next));
				const float safe_alpha = std::clamp(alpha, 0.f, M_PI_F - std::numeric_limits<float>::epsilon());
				float accel_tmp = config.max_acc_xy_radius_scale * config.max_acc_xy;
				float max_speed_in_turn = computeMaxSpeedInWaypoint(safe_alpha, accel_tmp, config.xy_accept_rad);
				speed_at_target= std::min(std::min(max_speed_in_turn, exit_speed), config.max_speed_xy);
			}

			float start_to_target = (start_position - target).head<2>().norm();
			float max_speed = computeMaxSpeedFromDistance(config.max_jerk, config.max_acc_xy, start_to_target, speed_at_target);

			return std::min(config.max_speed_xy, max_speed);
		}

		/*
		 * This function computes the maximum speed XY that can be travelled, given a set of waypoints and vehicle dynamics
		 *
		 * The first waypoint should be the starting location, and the later waypoints the desired points to be followed.
		 *
		 * @param waypoints the list of waypoints to be followed, the first of which should be the starting location
		 * @param config the vehicle dynamic limits
		 *
		 * @return the maximum speed at waypoint[0] which allows it to follow the trajectory while respecting the dynamic limits
		 */
		template <size_t N>
		float computeXYSpeedFromWaypoints(const Eigen::Vector3f waypoints[N], const VehicleDynamicLimits &config)
		{
			static_assert(N >= 2, "Need at least 2 points to compute speed");

			float max_speed = 0.f;

			for (size_t j = 0; j < N - 1; j++)
			{
				size_t i = N - 2 - j;
				max_speed = computeStartXYSpeedFromWaypoints(waypoints[i],
															 waypoints[i + 1],
															 waypoints[std::min(i + 2, N - 1)],
															 max_speed, config);
			}

			return max_speed;
		}

		/*
		 * Constrain the 3D vector given a maximum XY norm
		 * If the XY norm of the 3D vector is larger than the maximum norm, the whole vector
		 * is scaled down to respect the constraint.
		 * If the maximum norm is small (defined by the "accuracy" parameter),
		 * only the XY components are scaled down to avoid affecting
		 * Z in case of numerical issues
		 */
		inline void clampToXYNorm(Eigen::Vector3f &target, float max_xy_norm, float accuracy = std::numeric_limits<float>::epsilon())
		{
			const float xynorm = target.head<2>().norm();
			const float scale_factor = (xynorm > std::numeric_limits<float>::epsilon())
										   ? max_xy_norm / xynorm
										   : 1.f;

			if (scale_factor < 1.f)
			{
				if (max_xy_norm < accuracy && xynorm < accuracy)
				{
					target.head<2>() = target.head<2>() * scale_factor;
				}
				else
				{
					target *= scale_factor;
				}
			}
		}

		/*
		 * Constrain the 3D vector given a maximum Z norm
		 * If the Z component of the 3D vector is larger than the maximum norm, the whole vector
		 * is scaled down to respect the constraint.
		 * If the maximum norm is small (defined by the "accuracy parameter),
		 * only the Z component is scaled down to avoid affecting
		 * XY in case of numerical issues
		 */
		inline void clampToZNorm(Eigen::Vector3f &target, float max_z_norm, float accuracy = std::numeric_limits<float>::epsilon())
		{
			const float znorm = fabs(target(2));
			const float scale_factor = (znorm > std::numeric_limits<float>::epsilon())
										   ? max_z_norm / znorm
										   : 1.f;

			if (scale_factor < 1.f)
			{
				if (max_z_norm < accuracy && znorm < accuracy)
				{
					target(2) *= scale_factor;
				}
				else
				{
					target *= scale_factor;
				}
			}
		}

	}
}
