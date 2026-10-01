/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PgfModel.hpp
 *
 * 13-state quadrotor dynamics and the derivatives the Pontryagin gradient flow
 * needs. Mirrors quad_mpc/model.py in the companion flight stack, which is the
 * single source of truth; PgfTest.cpp checks this port against vectors it
 * generated.
 *
 *   state x (13) = [p_NED(3), v_NED(3), q_wxyz(4), omega_FRD(3)]
 *   input u  (4) = per-rotor thrust [N], index i -> MOTOR{i+1}
 *
 * Two structural facts about this model make the onboard flow cheap, and the
 * unit tests assert both:
 *
 *   1. The dynamics are affine in u, so <w, f_uu> is identically zero and the
 *      flow's Q_uu is a constant multiple of the identity. No 4x4 inverse is
 *      ever needed for it.
 *   2. f_u depends only on the attitude, and <w, f_ux> is nonzero only in the
 *      four quaternion columns, where every rotor's row is identical. So the
 *      whole (4 x 13) matrix collapses to one 4-vector.
 */

#pragma once

#include <matrix/math.hpp>

namespace pgf
{

static constexpr int NX = 13;   ///< state dimension
static constexpr int NU = 4;    ///< number of rotors

static constexpr int IDX_P = 0;   ///< position, NED [m]
static constexpr int IDX_V = 3;   ///< velocity, NED [m/s]
static constexpr int IDX_Q = 6;   ///< attitude quaternion (w, x, y, z)
static constexpr int IDX_W = 10;  ///< body rates, FRD [rad/s]

using StateVector = matrix::Vector<float, NX>;
using InputVector = matrix::Vector<float, NU>;
using InputJacobian = matrix::Matrix<float, NX, NU>;   ///< f_u
using GainMatrix = matrix::Matrix<float, NU, NX>;      ///< Q_ux and <w, f_ux>

/**
 * Quadrotor dynamics evaluated from a fixed parameter set.
 *
 * The parameters are set once from PGF_* and must match the numbers the Jetson
 * MPC built its value expansion with.
 *
 * Rotor i sits at (x_i, y_i) in the body frame and pushes along -z_body. Its
 * yaw reaction torque is spin_i * km_over_kf * T_i, with spin_i = +1 for a
 * rotor turning counter-clockwise seen from above. That matches PX4's control
 * allocator, where CA_ROTOR{i}_KM = spin_i * km_over_kf
 * (ActuatorEffectivenessRotors.cpp: moment = ct*position.cross(axis) - ct*km*axis).
 */
class Model
{
public:
	/**
	 * @param mass          [kg]
	 * @param inertia_diag  [kg m^2] diagonal of the body inertia tensor
	 * @param rotor_pos_x   [m] rotor hub x coordinates, body FRD
	 * @param rotor_pos_y   [m] rotor hub y coordinates, body FRD
	 * @param rotor_spin    +1 counter-clockwise from above, -1 clockwise
	 * @param km_over_kf    [m] yaw torque per Newton of rotor thrust
	 */
	void setParameters(float mass, const matrix::Vector3f &inertia_diag,
			   const float rotor_pos_x[NU], const float rotor_pos_y[NU],
			   const float rotor_spin[NU], float km_over_kf, float gravity = 9.81f);

	bool isValid() const { return _valid; }

	/** Thrust each rotor must produce to hold the vehicle level [N]. */
	float hoverThrust() const { return _mass * _gravity / static_cast<float>(NU); }

	/** xdot = f(x, u). */
	StateVector f(const StateVector &x, const InputVector &u) const;

	/** f_u, which depends on the attitude alone. */
	InputJacobian fu(const StateVector &x) const;

	/**
	 * <w, f_ux> = d/dx (f_u' w), a plain (NU, NX) matrix.
	 *
	 * Only the quaternion columns are nonzero and every row is the same, so
	 * this returns that single 4-vector rather than the full matrix.
	 * <w, f_uu> is identically zero and has no accessor.
	 */
	matrix::Vector<float, 4> wfuxQuatRow(const StateVector &x, const StateVector &w) const;

	/** Body torque produced by a per-rotor thrust vector [N m]. */
	matrix::Vector3f torque(const InputVector &u) const { return _torque_alloc * u; }

private:
	float _mass{0.f};
	float _gravity{9.81f};
	matrix::Vector3f _inertia{};
	matrix::Vector3f _inertia_inv{};
	matrix::Matrix<float, 3, NU> _torque_alloc{};   ///< G_tau
	bool _valid{false};
};

/** Unit-normalize the quaternion block of a state in place. */
void normalizeQuaternion(StateVector &x);

/**
 * Invert the rotor thrust curve T(eta) = a eta^2 + b eta + c for the normalized
 * motor command, clamped to [0, 1].
 *
 * The coefficients come from bench data via
 * quadrotor-flight-stack/scripts/fit_thrust_map.py. THR_MDL_FAC must be 0 or
 * PX4's mixer applies a second, different curve on top of this one.
 *
 * @return 0 for a non-finite thrust or an unset curve, so a misconfigured
 *         vehicle idles rather than commanding something arbitrary.
 */
float thrustToNormalized(float thrust, float a, float b, float c);

} // namespace pgf
