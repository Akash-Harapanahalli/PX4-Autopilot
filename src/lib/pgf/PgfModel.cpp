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

#include "PgfModel.hpp"

#include <mathlib/mathlib.h>

namespace pgf
{

void Model::setParameters(float mass, const matrix::Vector3f &inertia_diag,
			  const float rotor_pos_x[NU], const float rotor_pos_y[NU],
			  const float rotor_spin[NU], float km_over_kf, float gravity)
{
	_mass = mass;
	_gravity = gravity;
	_inertia = inertia_diag;

	_valid = mass > 1e-3f && inertia_diag(0) > 0.f && inertia_diag(1) > 0.f
		 && inertia_diag(2) > 0.f;

	if (!_valid) {
		return;
	}

	for (int i = 0; i < 3; i++) {
		_inertia_inv(i) = 1.f / inertia_diag(i);
	}

	// tau = sum_i T_i * (r_i x (-z_body)) + spin_i * km_over_kf * T_i * z_body
	for (int i = 0; i < NU; i++) {
		_torque_alloc(0, i) = -rotor_pos_y[i];
		_torque_alloc(1, i) = rotor_pos_x[i];
		_torque_alloc(2, i) = rotor_spin[i] * km_over_kf;
	}
}

StateVector Model::f(const StateVector &x, const InputVector &u) const
{
	StateVector out{};

	if (!_valid) {
		return out;
	}

	const matrix::Quatf q(x(IDX_Q), x(IDX_Q + 1), x(IDX_Q + 2), x(IDX_Q + 3));
	const matrix::Vector3f omega(x(IDX_W), x(IDX_W + 1), x(IDX_W + 2));

	// pdot = v
	for (int i = 0; i < 3; i++) {
		out(IDX_P + i) = x(IDX_V + i);
	}

	// vdot = g*z_world + R(q) * (-sum(T)) * z_body / m
	float thrust_total = 0.f;

	for (int i = 0; i < NU; i++) {
		thrust_total += u(i);
	}

	const matrix::Vector3f accel =
		matrix::Vector3f(0.f, 0.f, _gravity)
		+ q.rotateVector(matrix::Vector3f(0.f, 0.f, -thrust_total)) / _mass;

	for (int i = 0; i < 3; i++) {
		out(IDX_V + i) = accel(i);
	}

	// qdot = 0.5 * q (x) [0, omega]
	const matrix::Quatf qdot = q * matrix::Quatf(0.f, omega(0), omega(1), omega(2)) * 0.5f;

	for (int i = 0; i < 4; i++) {
		out(IDX_Q + i) = qdot(i);
	}

	// omegadot = I^-1 (tau - omega x I omega)
	const matrix::Vector3f I_omega = _inertia.emult(omega);
	const matrix::Vector3f omega_dot =
		_inertia_inv.emult(_torque_alloc * u - omega.cross(I_omega));

	for (int i = 0; i < 3; i++) {
		out(IDX_W + i) = omega_dot(i);
	}

	return out;
}

InputJacobian Model::fu(const StateVector &x) const
{
	InputJacobian out{};

	if (!_valid) {
		return out;
	}

	const matrix::Quatf q(x(IDX_Q), x(IDX_Q + 1), x(IDX_Q + 2), x(IDX_Q + 3));

	// Every rotor pushes along the same body axis, so each column of the
	// velocity block is identical.
	const matrix::Vector3f dv = q.rotateVector(matrix::Vector3f(0.f, 0.f, -1.f / _mass));

	for (int i = 0; i < NU; i++) {
		for (int r = 0; r < 3; r++) {
			out(IDX_V + r, i) = dv(r);
			out(IDX_W + r, i) = _inertia_inv(r) * _torque_alloc(r, i);
		}
	}

	// The position and quaternion blocks do not depend on u at all.
	return out;
}

matrix::Vector<float, 4> Model::wfuxQuatRow(const StateVector &x, const StateVector &w) const
{
	matrix::Vector<float, 4> out{};

	if (!_valid) {
		return out;
	}

	// Only f_u's velocity block varies with the state, and only through q:
	//   f_u(:, i) restricted to v  =  -(1/m) * R(q) * z_body  =  -(1/m) * R[:, 2]
	// so every row of <w, f_ux> is  -(1/m) * d/dq (w_v . R[:,2]).
	// The body-rate block contributes nothing: I^-1 * G is constant.
	//
	// The third column is taken as
	//   R[:,2] = [2(xz + wy), 2(yz - wx), 1 - 2(x^2 + y^2)]
	// and NOT the equivalent-on-the-unit-sphere w^2 - x^2 - y^2 + z^2. The two
	// agree wherever ||q|| = 1 but have different derivatives off it, and the
	// value function this flow differentiates was built by acados from the
	// first form (quad_mpc/model.py::quat_to_rotm). Using the other one leaves
	// f and f_u correct while silently corrupting <w, f_ux>.
	const float qw = x(IDX_Q), qx = x(IDX_Q + 1), qy = x(IDX_Q + 2), qz = x(IDX_Q + 3);
	const float w0 = w(IDX_V), w1 = w(IDX_V + 1), w2 = w(IDX_V + 2);
	const float k = -2.f / _mass;

	out(0) = k * (w0 * qy - w1 * qx);
	out(1) = k * (w0 * qz - w1 * qw - 2.f * w2 * qx);
	out(2) = k * (w0 * qw + w1 * qz - 2.f * w2 * qy);
	out(3) = k * (w0 * qx + w1 * qy);
	return out;
}

float thrustToNormalized(float thrust, float a, float b, float c)
{
	if (!PX4_ISFINITE(thrust)) {
		return 0.f;
	}

	if (a <= 0.f) {
		// Linear curve. Not just a degenerate case: an ideal thrust-controlled
		// actuator has one, and so does the SIH simulator.
		if (b <= 0.f) {
			return 0.f;   // unset or non-physical; idle rather than guess
		}

		return math::constrain((thrust - c) / b, 0.f, 1.f);
	}

	const float disc = b * b - 4.f * a * (c - thrust);
	const float eta = (-b + sqrtf(math::max(disc, 0.f))) / (2.f * a);
	return math::constrain(eta, 0.f, 1.f);
}

void normalizeQuaternion(StateVector &x)
{
	const float n = sqrtf(x(IDX_Q) * x(IDX_Q) + x(IDX_Q + 1) * x(IDX_Q + 1)
			      + x(IDX_Q + 2) * x(IDX_Q + 2) + x(IDX_Q + 3) * x(IDX_Q + 3));

	if (n < 1e-6f) {
		x(IDX_Q) = 1.f;
		x(IDX_Q + 1) = x(IDX_Q + 2) = x(IDX_Q + 3) = 0.f;
		return;
	}

	for (int i = 0; i < 4; i++) {
		x(IDX_Q + i) /= n;
	}
}

} // namespace pgf
