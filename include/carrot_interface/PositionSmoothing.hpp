#pragma once

#include <cmath>
#include "VelocitySmoothing.hpp"
#include "TrajMath.hpp"
#include "AlphaFilter.hpp"

#include "carrot_interface/TrajectoryConstraints.hpp"

#include <eigen3/Eigen/Core>
#include <chrono>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>

#include <cstdio>
#include <float.h>
#include <algorithm>

using namespace mavsdk;

/**
 * @brief Class that generates setpoints for a smooth trajectory
 * to position waypoints.
 *
 * This is achieved by first generating an unsmoothed velocity setpoint
 * which then gets smoothed using the VelocitySmoothing library.
 */
class PositionSmoothing
{

public:
	PositionSmoothing(std::shared_ptr<System> system)
		: _action(system),
		  _offboard(system),
		  _telemetry(system),
		  _param(system)
	{

		set_px4_params();

		_telemetry.subscribe_position_velocity_ned([&](Telemetry::PositionVelocityNed position_velocity_ned)
												   { _position_velocity_ned = position_velocity_ned; });

		_telemetry.subscribe_attitude_euler([&](Telemetry::EulerAngle euler_angle)
											{ _euler_angle = euler_angle; });
	}

	void set_px4_params()
	{

		// Fetch PX4 parameters
		std::pair<Param::Result, float> _param_mpc_xy_err_max = _param.get_param_float("MPC_XY_ERR_MAX");
		std::pair<Param::Result, float> _param_nav_mc_alt_rad = _param.get_param_float("NAV_MC_ALT_RAD");
		std::pair<Param::Result, float> _param_mpc_acc_hor = _param.get_param_float("MPC_ACC_HOR");
		std::pair<Param::Result, float> _param_mpc_jerk_auto = _param.get_param_float("MPC_JERK_AUTO");
		std::pair<Param::Result, float> _param_mpc_xy_traj_p = _param.get_param_float("MPC_XY_TRAJ_P");
		std::pair<Param::Result, float> _param_mpc_xy_vel_max = _param.get_param_float("MPC_XY_VEL_MAX");
		std::pair<Param::Result, float> _param_mpc_xy_cruise = _param.get_param_float("MPC_XY_CRUISE");
		std::pair<Param::Result, float> _param_nav_acc_rad = _param.get_param_float("NAV_ACC_RAD");

		if (_param_nav_acc_rad.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get NAV_ACC_RAD");
		}
		else
		{
			std::cout << "NAV_ACC_RAD: " << _param_nav_acc_rad.second << std::endl;
			setTargetAcceptanceRadius(_param_nav_acc_rad.second);
		}

		if (_param_mpc_xy_err_max.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get MPC_XY_ERR_MAX");
		}
		else
		{
			std::cout << "MPC_XY_ERR_MAX: " << _param_mpc_xy_err_max.second << std::endl;
			setMaxAllowedHorizontalError(_param_mpc_xy_err_max.second);
		}

		if (_param_nav_mc_alt_rad.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get NAV_MC_ALT_RAD");
		}
		else
		{
			std::cout << "NAV_MC_ALT_RAD: " << _param_nav_mc_alt_rad.second << std::endl;
			setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.second);
		}

		if (_param_mpc_xy_traj_p.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get MPC_XY_TRAJ_P");
		}
		else
		{
			std::cout << "MPC_XY_TRAJ_P: " << _param_mpc_xy_traj_p.second << std::endl;
			setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.second);
		}

		if (_param_mpc_xy_cruise.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get MPC_XY_CRUISE");
		}
		else
		{
			std::cout << "MPC_XY_CRUISE: " << _param_mpc_xy_cruise.second << std::endl;
			setCruiseSpeed(_param_mpc_xy_cruise.second);
		}

		if (_param_mpc_xy_vel_max.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get MPC_XY_VEL_MAX");
		}
		else
		{
			std::cout << "MPC_XY_VEL_MAX: " << _param_mpc_xy_vel_max.second << std::endl;
			Eigen::Vector3f max_speeds{_param_mpc_xy_vel_max.second, _param_mpc_xy_vel_max.second, _param_mpc_xy_vel_max.second};
			setMaxVelocityXYZ(max_speeds);
		}

		if (_param_mpc_acc_hor.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get MPC_ACC_HOR");
		}
		else
		{
			std::cout << "MPC_ACC_HOR: " << _param_mpc_acc_hor.second << std::endl;
			Eigen::Vector3f max_acceleration{_param_mpc_acc_hor.second, _param_mpc_acc_hor.second, _param_mpc_acc_hor.second};
			setMaxAccelerationXYZ(max_acceleration);
		}

		if (_param_mpc_jerk_auto.first != Param::Result::Success)
		{
			throw std::runtime_error("Unable to get MPC_JERK_AUTO");
		}
		else
		{
			std::cout << "MPC_JERK_AUTO: " << _param_mpc_jerk_auto.second << std::endl;
			Eigen::Vector3f max_jerk{_param_mpc_jerk_auto.second, _param_mpc_jerk_auto.second, _param_mpc_jerk_auto.second};
			setMaxJerkXYZ(max_jerk);
		}
	}

	bool startOffboardMode()
	{
		std::cout << "Starting Offboard velocity control in NED coordinates\n";

		// Send it once before starting offboard, otherwise it will be rejected.
		mavsdk::Offboard::VelocityNedYaw stay{};
		_offboard.set_velocity_ned(stay);

		mavsdk::Offboard::Result offboard_result = _offboard.start();
		if (offboard_result != mavsdk::Offboard::Result::Success)
		{
			std::cerr << "Offboard start failed: " << offboard_result << '\n';
			return false;
		}

		Eigen::Vector3f local_position{_position_velocity_ned.position.north_m, _position_velocity_ned.position.east_m,
									   _position_velocity_ned.position.down_m};

		Eigen::Vector3f local_velocity{_position_velocity_ned.velocity.north_m_s,
									   _position_velocity_ned.velocity.east_m_s,
									   _position_velocity_ned.velocity.down_m_s};

		// No acceleration estimate available, set to zero
		Eigen::Vector3f local_acceleration{0.f, 0.f, 0.f};

		// reset(local_acceleration, local_velocity, local_position);

		// std::this_thread::sleep_for(std::chrono::seconds(1));

		std::cout << "Offboard started & trajectory reset to current position\n";

		return true;
	}

	bool stopOffboardMode()
	{

		mavsdk::Offboard::Result offboard_result = _offboard.stop();
		if (offboard_result != Offboard::Result::Success)
		{
			std::cerr << "Offboard stop failed: " << offboard_result << '\n';
			return false;
		}
		std::cout << "Offboard stopped\n";

		return true;
	}

	void reset(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &velocity, const Eigen::Vector3f &position)
	{
		for (size_t i = 0; i < 3; i++)
		{
			_trajectory[i].reset(acceleration(i), velocity(i), position(i));
		}
	}

	inline Eigen::Vector3f getCurrentAccelerationXYZ() const
	{
		return {_trajectory[0].getCurrentAcceleration(), _trajectory[1].getCurrentAcceleration(), _trajectory[2].getCurrentAcceleration()};
	}

	inline Eigen::Vector3f getCurrentVelocityXYZ() const
	{
		return {_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity(), _trajectory[2].getCurrentVelocity()};
	}

	inline Eigen::Vector3f getCurrentPositionXYZ() const
	{
		return {_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition(), _trajectory[2].getCurrentPosition()};
	}

	inline void setMaxJerkXYZ(const Eigen::Vector3f &jerk)
	{
		_trajectory[0].setMaxJerk(jerk(0));
		_trajectory[1].setMaxJerk(jerk(1));
		_trajectory[2].setMaxJerk(jerk(2));
	}

	inline void setMaxAccelerationXYZ(const Eigen::Vector3f &accel)
	{
		_trajectory[0].setMaxAccel(accel(0));
		_trajectory[1].setMaxAccel(accel(1));
		_trajectory[2].setMaxAccel(accel(2));
	}

	inline void setMaxVelocityXYZ(const Eigen::Vector3f &vel)
	{
		_trajectory[0].setMaxVel(vel(0));
		_trajectory[1].setMaxVel(vel(1));
		_trajectory[2].setMaxVel(vel(2));
	}

	inline void setMaxAllowedHorizontalError(float error)
	{
		_max_allowed_horizontal_error = error;
	}

	inline void setVerticalAcceptanceRadius(float radius)
	{
		_vertical_acceptance_radius = radius;
	}

	inline void setCruiseSpeed(float speed)
	{
		_cruise_speed = speed;
	}

	inline void setHorizontalTrajectoryGain(float gain)
	{
		_horizontal_trajectory_gain = gain;
	}

	inline void setTargetAcceptanceRadius(float radius)
	{
		_target_acceptance_radius = radius;
	}

	template <typename Integer>
	Integer wrap(Integer x, Integer low, Integer high)
	{
		const auto range = high - low;

		if (x < low)
		{
			x += range * ((low - x) / range + 1);
		}

		// return low + (x - low) % range;
		return low + fmod((x - low), range);
	}

	/**
	 * Wrap value in range [-π, π)
	 */
	template <typename Type>
	Type wrap_pi(Type x)
	{
		return wrap(x, Type(-M_PI), Type(M_PI));
	}

	void limitYawRate(float &_yaw_setpoint, const float &delta_time)
	{
		const float yawrate_max = 45.f * (M_PI / 180.0); // MPC_YAWRAUTO_MAX

		_yaw_sp_aligned = true;

		if (std::isfinite(_yaw_setpoint) && std::isfinite(_yaw_sp_prev))
		{
			// Limit the rate of change of the yaw setpoint
			const float dyaw_desired = wrap_pi(_yaw_setpoint - _yaw_sp_prev);
			const float dyaw_max = yawrate_max * delta_time;
			const float dyaw = std::clamp(dyaw_desired, -dyaw_max, dyaw_max);
			const float yaw_setpoint_sat = wrap_pi(_yaw_sp_prev + dyaw);

			// The yaw setpoint is aligned when it is within tolerance
			_yaw_sp_aligned = fabsf(wrap_pi(_yaw_setpoint - yaw_setpoint_sat)) < 12.f * (M_PI / 180.0);

			_yaw_setpoint = yaw_setpoint_sat;

			if (!std::isfinite(_yawspeed_setpoint) && (delta_time > FLT_EPSILON))
			{
				// Create a feedforward using the filtered derivative
				_yawspeed_filter.setParameters(delta_time, .2f);
				_yawspeed_filter.update(dyaw / delta_time);
				_yawspeed_setpoint = _yawspeed_filter.getState();
				std::cout << "Yaw rate post filter: " << _yawspeed_setpoint * (180 / M_PI) << std::endl;
			}
		}

		_yaw_sp_prev = std::isfinite(_yaw_setpoint) ? _yaw_setpoint : _euler_angle.yaw_deg * (M_PI / 180.0);

		if (std::isfinite(_yawspeed_setpoint))
		{
			// The yaw setpoint is aligned when its rate is not saturated
			_yaw_sp_aligned = _yaw_sp_aligned && (fabsf(_yawspeed_setpoint) < yawrate_max);
			std::cout << "Yaw rate pre-clamp: " << _yawspeed_setpoint * (180 / M_PI) << std::endl;
			std::cout << "Yaw rate max: " << yawrate_max * (180 / M_PI) << std::endl;
			_yawspeed_setpoint = std::clamp(_yawspeed_setpoint, -yawrate_max, yawrate_max);
			std::cout << "Yaw rate post-clamp: " << _yawspeed_setpoint * (180 / M_PI) << std::endl;
		}
	}

	void fly_to_position_local_ned(
		const Eigen::Vector3f &target_local_position_ned, float &target_yaw)
	{

		_target_yaw = target_yaw;

		auto now = std::chrono::high_resolution_clock::now();
		auto epoch = now.time_since_epoch();
		auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(epoch);

		const double timestamp = microseconds.count();

		Eigen::Vector3f velocity_setpoint{0.f, 0.f, 0.f};

		for (int i = 0; i < target_local_position_ned.size(); ++i)
		{
			if (!std::isfinite(target_local_position_ned[i]))
			{
				return;
			}
		}

		velocity_setpoint = _generateUnsmoothedVelocitySetpoint(target_local_position_ned);

		for (int i = 0; i < velocity_setpoint.size(); ++i)
		{
			if (!std::isfinite(velocity_setpoint[i]))
			{
				return;
			}
		}

		Eigen::Vector3f local_position{_position_velocity_ned.position.north_m, _position_velocity_ned.position.east_m,
									   _position_velocity_ned.position.down_m};

		/* Slow down the trajectory by decreasing the integration time based on the position error.
		 * This is only performed when the drone is behind the trajectory
		 */
		Eigen::Vector2f position_trajectory_xy(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
		Eigen::Vector2f local_position_xy(local_position.head<2>());
		Eigen::Vector2f vel_traj_xy(_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity());
		Eigen::Vector2f drone_to_trajectory_xy(position_trajectory_xy - local_position_xy);
		float local_position_xy_error = drone_to_trajectory_xy.norm();

		float time_stretch = 1.f - std::clamp(local_position_xy_error / _max_allowed_horizontal_error, 0.f, 1.f);

		// Don't stretch time if the drone is ahead of the position setpoint
		if (drone_to_trajectory_xy.dot(vel_traj_xy) < 0.f)
		{
			time_stretch = 1.f;
		}

		const float delta_time = std::min((timestamp - _last_update), _timeout) / 1e6f;

		_last_update = timestamp;

		for (int i = 0; i < 3; ++i)
		{
			_trajectory[i].updateTraj(delta_time, time_stretch);
		}

		for (int i = 0; i < 3; ++i)
		{
			_trajectory[i].updateDurations(velocity_setpoint(i));
		}

		VelocitySmoothing::timeSynchronization(_trajectory, 3);

		limitYawRate(_target_yaw, delta_time);

		std::cout << "Target yaw: " << _target_yaw * (180 / M_PI) << std::endl;

		std::cout << "_yawspeed_setpoint: " << _yawspeed_setpoint * (180 / M_PI) << std::endl;

		const Offboard::PositionNedYaw position_ned_yaw{_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition(),
														_trajectory[2].getCurrentPosition(), _target_yaw * (180 / M_PI)};
		const Offboard::VelocityNedYaw velocity_ned_yaw{_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity(),
														_trajectory[2].getCurrentVelocity(), _yawspeed_setpoint * (180 / M_PI)};
		const Offboard::AccelerationNed acceleration_ned{_trajectory[0].getCurrentAcceleration(), _trajectory[1].getCurrentAcceleration(),
														 _trajectory[2].getCurrentAcceleration()};

		_offboard.set_position_velocity_acceleration_ned(position_ned_yaw, velocity_ned_yaw, acceleration_ned);
	}

private:
	mavsdk::Action _action;
	mavsdk::Offboard _offboard;
	mavsdk::Telemetry _telemetry;
	mavsdk::Param _param;

	Telemetry::PositionVelocityNed _position_velocity_ned;
	Telemetry::EulerAngle _euler_angle;

	float _max_allowed_horizontal_error{0.f};
	float _vertical_acceptance_radius{0.f};
	float _cruise_speed{0.f};
	float _horizontal_trajectory_gain{0.f};
	float _target_acceptance_radius{0.f};
	double _last_update{0.f};
	bool _yaw_sp_aligned{false};
	float _yaw_sp_prev{NAN};
	float _yawspeed_setpoint{NAN};
	float _target_yaw{NAN};
	AlphaFilter<float> _yawspeed_filter;

	VelocitySmoothing _trajectory[3]; ///< Trajectories in x, y and z directions
	float _max_speed_previous{0.f};

	double _timeout = 500000; /**< maximal time in us before a loop or data times out */

	const Eigen::Vector3f _generateUnsmoothedVelocitySetpoint(const Eigen::Vector3f &target)
	{

		Eigen::Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
								 _trajectory[1].getCurrentPosition(),
								 _trajectory[2].getCurrentPosition());
		Eigen::Vector3f vec{(target - pos_traj)};
		Eigen::Vector3f u_pos_traj_to_dest;
		if (vec.norm() > std::numeric_limits<float>::epsilon())
		{
			u_pos_traj_to_dest = vec.normalized();
		}
		else
		{
			u_pos_traj_to_dest = Eigen::Vector3f::Zero();
		}

		float xy_speed = _getMaxXYSpeed(target);
		const float z_speed = _getMaxZSpeed(target);

		Eigen::Vector3f velocity_setpoint = u_pos_traj_to_dest * sqrt(xy_speed * xy_speed + z_speed * z_speed);
		math::trajectory::clampToXYNorm(velocity_setpoint, xy_speed, 0.5f);
		math::trajectory::clampToZNorm(velocity_setpoint, z_speed, 0.5f);

		return velocity_setpoint;
	}

	float _getMaxXYSpeed(const Eigen::Vector3f &target) const
	{
		Eigen::Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
								 _trajectory[1].getCurrentPosition(),
								 _trajectory[2].getCurrentPosition());

		math::trajectory::VehicleDynamicLimits config;
		config.z_accept_rad = _vertical_acceptance_radius;			  //_param_nav_mc_alt_rad
		config.xy_accept_rad = _target_acceptance_radius;			  // from triplet
		config.max_acc_xy = _trajectory[0].getMaxAccel();			  //_param_mpc_acc_hor
		config.max_jerk = _trajectory[0].getMaxJerk();				  //_param_mpc_jerk_auto
		config.max_speed_xy = _cruise_speed;						  // from triplet
		config.max_acc_xy_radius_scale = _horizontal_trajectory_gain; //_param_mpc_xy_traj_p

		Eigen::Vector3f pos_to_target[2] = {pos_traj, target};

		return math::trajectory::computeXYSpeedFromWaypoints<2>(pos_to_target, config);
	}

	float _getMaxZSpeed(const Eigen::Vector3f &target) const
	{

		Eigen::Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
								 _trajectory[1].getCurrentPosition(),
								 _trajectory[2].getCurrentPosition());

		const float distance_start_target = fabs(target(2) - pos_traj(2));
		const float arrival_z_speed = 0.f;

		float max_speed = std::min(_trajectory[2].getMaxVel(), math::trajectory::computeMaxSpeedFromDistance(
																   _trajectory[2].getMaxJerk(), _trajectory[2].getMaxAccel(),
																   distance_start_target, arrival_z_speed));

		return max_speed;
	}
};
