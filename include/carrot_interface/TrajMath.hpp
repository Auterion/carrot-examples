/**
 * @file TrajMath.hpp
 *
 * collection of functions used for trajectory generation
 */
#include <eigen3/Eigen/Core>

#pragma once

namespace math
{

namespace trajectory
{

/* Compute the maximum possible speed on the track given the desired speed,
 * remaining distance, the maximum acceleration and the maximum jerk.
 * We assume a constant acceleration profile with a delay of 2*accel/jerk
 * (time to reach the desired acceleration from opposite max acceleration)
 * Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
 *
 * @param jerk maximum jerk
 * @param accel maximum acceleration
 * @param braking_distance distance to the desired point
 * @param final_speed the still-remaining speed of the vehicle when it reaches the braking_distance
 *
 * @return maximum speed
 */
inline float computeMaxSpeedFromDistance(const float jerk, const float accel, const float braking_distance,
		const float final_speed)
{
	auto sqr = [](float f) {return f * f;};
	float b =  4.0f * sqr(accel) / jerk;
	float c = - 2.0f * accel * braking_distance - sqr(final_speed);
	float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));

	// don't slow down more than the end speed, even if the conservative accel ramp time requests it
	return fmaxf(max_speed, final_speed);
}

/* Compute the maximum tangential speed in a circle defined by two line segments of length "d"
 * forming a V shape, opened by an angle "alpha". The circle is tangent to the end of the
 * two segments as shown below:
 *      \\
 *      | \ d
 *      /  \
 *  __='___a\
 *      d
 *  @param alpha angle between the two line segments
 *  @param accel maximum lateral acceleration
 *  @param d length of the two line segments
 *
 *  @return maximum tangential speed
 */
inline float computeMaxSpeedInWaypoint(const float alpha, const float accel, const float d)
{
	float tan_alpha = tanf(alpha / 2.0f);
	float max_speed_in_turn = sqrtf(accel * d * tan_alpha);

	return max_speed_in_turn;
}

/* Compute the braking distance given a maximum acceleration, maximum jerk and a maximum delay acceleration.
 * We assume a constant acceleration profile with a delay of accel_delay_max/jerk
 * (time to reach the desired acceleration from opposite max acceleration)
 * Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
 *
 * @param velocity initial velocity
 * @param jerk maximum jerk
 * @param accel maximum target acceleration during the braking maneuver
 * @param accel_delay_max the acceleration defining the delay described above
 *
 * @return braking distance
 */
inline float computeBrakingDistanceFromVelocity(const float velocity, const float jerk, const float accel,
		const float accel_delay_max)
{
	return velocity * (velocity / (2.0f * accel) + accel_delay_max / jerk);
}

/* Compute the maximum distance between a point and a circle given a direction vector pointing from the point
 * towards the circle. The point can be inside or outside the circle.
 *                  _
 *               ,=' '=,               __
 *    P-->------/-------A   Distance = PA
 *       Dir   |    x    |
 *              \       /
 *               "=,_,="
 * Equation to solve: ||(point - circle_pos) + direction_unit * distance_to_circle|| = radius
 *
 * @param pos position of the point
 * @param circle_pos position of the center of the circle
 * @param radius radius of the circle
 * @param direction vector pointing from the point towards the circle
 *
 * @return longest distance between the point to the circle in the direction indicated by the vector or NAN if the
 * vector does not point towards the circle
 */
inline float getMaxDistanceToCircle(const Eigen::Vector2f &pos, const Eigen::Vector2f &circle_pos, float radius,
				    const Eigen::Vector2f &direction)
{
	Eigen::Vector2f center_to_pos = pos - circle_pos;
	const float b = 2.f * center_to_pos.dot(direction.normalized());
	const float c = center_to_pos.squaredNorm() - radius * radius;
	const float delta = b * b - 4.f * c;

	float distance_to_circle;

	if (delta >= 0.f && (direction.squaredNorm() > (0.f * 0.f))) {
		distance_to_circle = fmaxf((-b + sqrtf(delta)) / 2.f, 0.f);

	} else {
		// Never intersecting the circle
		distance_to_circle = NAN;
	}

	return distance_to_circle;
}

} /* namespace traj */
} /* namespace math */
