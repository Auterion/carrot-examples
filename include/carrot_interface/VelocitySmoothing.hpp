#pragma once

#include <math.h>

struct Trajectory
{
	float j; //< jerk
	float a; //< acceleration
	float v; //< velocity
	float x; //< position
};

/**
 * @class VelocitySmoothing
 *
 *    |T1| T2 |T3|
 *     ___
 *   __| |____   __ Jerk
 *            |_|
 *        ___
 *       /   \	 Acceleration
 *   ___/     \___
 *             ___
 *           ;"
 *          /
 *         / 	 Velocity
 *        ;
 *   ----"
 */
class VelocitySmoothing
{
public:
	VelocitySmoothing(float initial_accel = 0.f, float initial_vel = 0.f, float initial_pos = 0.f)
	{
		reset(initial_accel, initial_vel, initial_pos);
	}
	~VelocitySmoothing() = default;

	/**
	 * Reset the state.
	 * @param accel Current acceleration
	 * @param vel Current velocity
	 * @param pos Current position
	 */
	void reset(float accel, float vel, float pos)
	{
		_state.j = 0.f;
		_state.a = accel;
		_state.v = vel;
		_state.x = pos;

		_state_init = _state;
	}

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint. This should be called on every cycle
	 * and before updateTraj().
	 * @param vel_setpoint velocity setpoint input
	 */
	void updateDurations(float vel_setpoint)
	{
		_vel_sp = std::clamp(vel_setpoint, -_max_vel, _max_vel);
		_local_time = 0.f;
		_state_init = _state;

		_direction = computeDirection();

		if (_direction != 0)
		{
			updateDurationsMinimizeTotalTime();
		}
		else
		{
			_T1 = _T2 = _T3 = 0.f;
		}
	}

	/**
	 * Generate the trajectory (acceleration, velocity and position) by integrating the current jerk
	 * @param dt integration period
	 * @param time_stretch (optional) used to scale the integration period. This can be used to slow down
	 * or fast-forward the trajectory
	 */
	void updateTraj(float dt, float time_stretch = 1.f)
	{
		_local_time += dt * time_stretch;
		float t_remain = _local_time;

		float t1 = std::min(t_remain, _T1);
		_state = evaluatePoly(_max_jerk, _state_init.a, _state_init.v, _state_init.x, t1, _direction);
		t_remain -= t1;

		if (t_remain > 0.f)
		{
			float t2 = std::min(t_remain, _T2);
			_state = evaluatePoly(0.f, _state.a, _state.v, _state.x, t2, 0.f);
			t_remain -= t2;
		}

		if (t_remain > 0.f)
		{
			float t3 = std::min(t_remain, _T3);
			_state = evaluatePoly(_max_jerk, _state.a, _state.v, _state.x, t3, -_direction);
			t_remain -= t3;
		}

		if (t_remain > 0.f)
		{
			_state = evaluatePoly(0.f, 0.f, _state.v, _state.x, t_remain, 0.f);
		}
	}

	/**
	 * Getters and setters
	 */
	float getMaxJerk() const { return _max_jerk; }
	void setMaxJerk(float max_jerk) { _max_jerk = max_jerk; }

	float getMaxAccel() const { return _max_accel; }
	void setMaxAccel(float max_accel) { _max_accel = max_accel; }

	float getMaxVel() const { return _max_vel; }
	void setMaxVel(float max_vel) { _max_vel = max_vel; }

	float getCurrentJerk() const { return _state.j; }
	void setCurrentAcceleration(const float accel) { _state.a = _state_init.a = accel; }
	float getCurrentAcceleration() const { return _state.a; }
	void setCurrentVelocity(const float vel) { _state.v = _state_init.v = vel; }
	float getCurrentVelocity() const { return _state.v; }
	void setCurrentPosition(const float pos) { _state.x = _state_init.x = pos; }
	float getCurrentPosition() const { return _state.x; }

	float getVelSp() const { return _vel_sp; }

	float getT1() const { return _T1; }
	float getT2() const { return _T2; }
	float getT3() const { return _T3; }
	float getTotalTime() const { return _T1 + _T2 + _T3; }

	template <typename T>
	int sign(T val) const
	{
		return (val == T(0)) ? 0 : static_cast<int>(std::copysign(T(1), val));
	}

	/**
	 * Synchronize several trajectories to have the same total time. This is required to generate
	 * straight lines.
	 * The resulting total time is the one of the longest trajectory.
	 * @param traj an array of VelocitySmoothing objects
	 * @param n_traj the number of trajectories to be synchronized
	 */
	static void timeSynchronization(VelocitySmoothing *traj, int n_traj)
	{
		float desired_time = 0.f;
		int longest_traj_index = 0;

		for (int i = 0; i < n_traj; i++)
		{
			const float T123 = traj[i].getTotalTime();

			if (T123 > desired_time)
			{
				desired_time = T123;
				longest_traj_index = i;
			}
		}

		if (desired_time > std::numeric_limits<float>::epsilon())
		{
			for (int i = 0; i < n_traj; i++)
			{
				if ((i != longest_traj_index) && (traj[i].getTotalTime() < desired_time))
				{
					traj[i].updateDurationsGivenTotalTime(desired_time);
				}
			}
		}
	}

private:
	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * @param T123 desired total time of the trajectory
	 */
	void updateDurationsGivenTotalTime(float T123)
	{
		float jerk_max_T1 = _direction * _max_jerk;
		float delta_v = _vel_sp - _state.v;

		// compute increasing acceleration time
		_T1 = computeT1(T123, _state.a, delta_v, jerk_max_T1, _max_accel);

		// compute decreasing acceleration time
		_T3 = computeT3(_T1, _state.a, jerk_max_T1);

		// compute constant acceleration time
		_T2 = computeT2(T123, _T1, _T3);
	}

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * Minimize the total time of the trajectory
	 */
	void updateDurationsMinimizeTotalTime()
	{
		float jerk_max_T1 = _direction * _max_jerk;
		float delta_v = _vel_sp - _state.v;

		// compute increasing acceleration time
		_T1 = computeT1(_state.a, delta_v, jerk_max_T1, _max_accel);

		// compute decreasing acceleration time
		_T3 = computeT3(_T1, _state.a, jerk_max_T1);

		// compute constant acceleration time
		_T2 = computeT2(_T1, _T3, _state.a, delta_v, jerk_max_T1);
	}

	/**
	 * Compute the direction of the jerk to be applied in order to drive the current state
	 * to the desired one
	 */
	int computeDirection() const
	{
		// Compute the velocity at which the trajectory will be
		// when the acceleration will be zero
		float vel_zero_acc = computeVelAtZeroAcc();

		/* Depending of the direction, start accelerating positively or negatively */
		int direction = sign(_vel_sp - vel_zero_acc);

		if (direction == 0)
		{
			// If by braking immediately the velocity is exactly
			// the require one with zero acceleration, then brake
			direction = sign(_state.a);
		}

		return direction;
	}

	/**
	 * Compute the velocity at which the trajectory will be if the maximum jerk is applied
	 * during the time required to cancel the current acceleration
	 */
	float computeVelAtZeroAcc() const
	{
		float vel_zero_acc = _state.v;

		if (std::fabs(_state.a) > std::numeric_limits<float>::epsilon())
		{
			float j_zero_acc = -sign(_state.a) * _max_jerk; // Required jerk to reduce the acceleration
			float t_zero_acc = -_state.a / j_zero_acc;		// Required time to cancel the current acceleration
			vel_zero_acc = _state.v + _state.a * t_zero_acc + 0.5f * j_zero_acc * t_zero_acc * t_zero_acc;
		}

		return vel_zero_acc;
	}

	/**
	 * Compute increasing acceleration time
	 */
	inline float computeT1(float a0, float v3, float j_max, float a_max) const
	{
		float delta = 2.f * a0 * a0 + 4.f * j_max * v3;

		if (delta < 0.f)
		{
			// Solution is not real
			return 0.f;
		}

		float sqrt_delta = std::sqrt(delta);
		float T1_plus = (-a0 + 0.5f * sqrt_delta) / j_max;
		float T1_minus = (-a0 - 0.5f * sqrt_delta) / j_max;

		float T3_plus = a0 / j_max + T1_plus;
		float T3_minus = a0 / j_max + T1_minus;

		float T1 = 0.f;

		if (T1_plus >= 0.f && T3_plus >= 0.f)
		{
			T1 = T1_plus;
		}
		else if (T1_minus >= 0.f && T3_minus >= 0.f)
		{
			T1 = T1_minus;
		}

		T1 = saturateT1ForAccel(a0, j_max, T1, a_max);

		return std::max(T1, 0.f);
	}

	/**
	 * Compute increasing acceleration time using total time constraint
	 */
	inline float computeT1(float T123, float a0, float v3, float j_max, float a_max) const
	{
		float a = -j_max;
		float b = j_max * T123 - a0;
		float delta = T123 * T123 * j_max * j_max + 2.f * T123 * a0 * j_max - a0 * a0 - 4.f * j_max * v3;

		if (delta < 0.f)
		{
			// Solution is not real
			return 0.f;
		}

		float sqrt_delta = std::sqrt(delta);
		float denominator_inv = 1.f / (2.f * a);
		float T1_plus = std::max((-b + sqrt_delta) * denominator_inv, 0.f);
		float T1_minus = std::max((-b - sqrt_delta) * denominator_inv, 0.f);

		float T3_plus = a0 / j_max + T1_plus;
		float T3_minus = a0 / j_max + T1_minus;

		float T1 = 0.f;

		if ((T1_plus >= 0.f && T3_plus >= 0.f) && ((T1_plus + T3_plus) <= T123))
		{
			T1 = T1_plus;
		}
		else if ((T1_minus >= 0.f && T3_minus >= 0.f) && ((T1_minus + T3_minus) <= T123))
		{
			T1 = T1_minus;
		}

		T1 = saturateT1ForAccel(a0, j_max, T1, a_max);

		return T1;
	}

	/**
	 * Saturate T1 in order to respect the maximum acceleration constraint
	 */
	inline float saturateT1ForAccel(float a0, float j_max, float T1, float a_max) const
	{
		/* Check maximum acceleration, saturate and recompute T1 if needed */
		float accel_T1 = a0 + j_max * T1;
		float T1_new = T1;

		if (accel_T1 > a_max)
		{
			T1_new = (a_max - a0) / j_max;
		}
		else if (accel_T1 < -a_max)
		{
			T1_new = (-a_max - a0) / j_max;
		}

		return T1_new;
	}

	/**
	 * Compute constant acceleration time
	 */
	inline float computeT2(float T1, float T3, float a0, float v3, float j_max) const
	{
		float T2 = 0.f;

		float den = a0 + j_max * T1;

		if (std::abs(den) > std::numeric_limits<float>::epsilon())
		{
			T2 = (-0.5f * T1 * T1 * j_max - T1 * T3 * j_max - T1 * a0 + 0.5f * T3 * T3 * j_max - T3 * a0 + v3) / den;
		}

		return std::max(T2, 0.f);
	}
	/**
	 * Compute constant acceleration time using total time constraint
	 */
	inline float computeT2(float T123, float T1, float T3) const
	{
		float T2 = T123 - T1 - T3;
		return std::max(T2, 0.f);
	}

	/**
	 * Compute decreasing acceleration time
	 */
	inline float computeT3(float T1, float a0, float j_max) const
	{
		float T3 = a0 / j_max + T1;
		return std::max(T3, 0.f);
	}

	/**
	 * Compute the jerk, acceleration, velocity and position
	 * of a jerk-driven polynomial trajectory at a given time t
	 * @param j jerk
	 * @param a0 initial acceleration at t = 0
	 * @param v0 initial velocity
	 * @param x0 initial postion
	 * @param t current time
	 * @param d direction
	 */
	inline Trajectory evaluatePoly(float j, float a0, float v0, float x0, float t, int d) const
	{
		Trajectory traj;
		float jt = d * j;
		float t2 = t * t;
		float t3 = t2 * t;

		traj.j = jt;
		traj.a = a0 + jt * t;
		traj.v = v0 + a0 * t + 0.5f * jt * t2;
		traj.x = x0 + v0 * t + 0.5f * a0 * t2 + 1.f / 6.f * jt * t3;

		return traj;
	}

	/* Input */
	float _vel_sp{0.0f};

	/* Constraints */
	float _max_jerk = 4.f;
	float _max_accel = 3.f;
	float _max_vel = 5.f;

	/* State (previous setpoints) */
	Trajectory _state{};
	int _direction{0};

	/* Initial conditions */
	Trajectory _state_init{};

	/* Duration of each phase */
	float _T1 = 0.f; ///< Increasing acceleration [s]
	float _T2 = 0.f; ///< Constant acceleration [s]
	float _T3 = 0.f; ///< Decreasing acceleration [s]

	float _local_time = 0.f; ///< Current local time
};
