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
 * @file PgfTest.cpp
 *
 * Checks the C++ flow against the Python original it was ported from. The
 * expected values in PgfTestVectors.hpp come from real acados solves, so the
 * value Hessians carry realistic conditioning (~1e4) rather than the benign
 * conditioning of a random symmetric matrix.
 */

#include <gtest/gtest.h>

#include "PgfFlow.hpp"
#include "PgfTestVectors.hpp"

using namespace pgf;
using namespace pgf_test_vectors;

namespace
{

Flow makeFlow()
{
	Flow flow;
	flow.model().setParameters(kMass, matrix::Vector3f(kInertia), kRotorX, kRotorY,
				   kRotorSpin, kKmOverKf, kGravity);
	FlowConfig cfg;
	cfg.r_scalar = kRScalar;
	cfg.alpha_c = kAlphaC;
	cfg.w_prox = 0.f;
	cfg.u_min = kUMin;
	cfg.u_max = kUMax;
	flow.setConfig(cfg);
	return flow;
}

StateVector toState(const float v[NX])
{
	StateVector out;

	for (int i = 0; i < NX; i++) {
		out(i) = v[i];
	}

	return out;
}

InputVector toInput(const float v[NU])
{
	InputVector out;

	for (int i = 0; i < NU; i++) {
		out(i) = v[i];
	}

	return out;
}

/** Unpack the row-major upper triangle the DDS message carries. */
matrix::SquareMatrix<float, NX> unpackHessian(const float packed[91])
{
	matrix::SquareMatrix<float, NX> S;
	int k = 0;

	for (int i = 0; i < NX; i++) {
		for (int j = i; j < NX; j++) {
			S(i, j) = packed[k];
			S(j, i) = packed[k];
			k++;
		}
	}

	EXPECT_EQ(k, 91);
	return S;
}

Payload toPayload(const Case &c)
{
	Payload p;
	p.x0 = toState(c.x0);
	p.x1 = toState(c.x1);
	p.s0 = toState(c.s0);
	p.s1 = toState(c.s1);
	p.S0 = unpackHessian(c.s0_upper);
	p.u0 = toInput(c.u0);
	p.u_nom0 = toInput(c.u_nom0);
	p.u_nom1 = toInput(c.u_nom1);
	p.dtau = kDtau;
	return p;
}

/** Mixed relative/absolute comparison; single precision against a float64 original. */
void expectClose(float actual, float expected, float rel, float abs_tol, const char *what, int i)
{
	const float tol = fmaxf(abs_tol, rel * fabsf(expected));
	EXPECT_NEAR(actual, expected, tol) << what << " element " << i;
}

} // namespace

TEST(PgfModel, HoverIsAnEquilibrium)
{
	Flow flow = makeFlow();
	StateVector x{};
	x(IDX_Q) = 1.f;
	InputVector u;
	u.setAll(flow.model().hoverThrust());

	const StateVector f = flow.model().f(x, u);

	for (int i = 0; i < NX; i++) {
		EXPECT_NEAR(f(i), 0.f, 1e-5f) << "element " << i;
	}
}

TEST(PgfModel, TorqueSignsMatchPx4)
{
	Flow flow = makeFlow();
	const Model &m = flow.model();

	// Rotor 0 sits front-right and turns counter-clockwise seen from above.
	InputVector only_first{};
	only_first(0) = 1.f;
	const matrix::Vector3f tau = m.torque(only_first);

	EXPECT_GT(tau(1), 0.f) << "a front rotor pushing up must pitch the nose up (+y, FRD)";
	EXPECT_LT(tau(0), 0.f) << "a right-hand rotor pushing up must roll left (-x, FRD)";
	EXPECT_GT(tau(2), 0.f) << "a CCW-from-above rotor must yaw nose-right (+z, FRD)";

	// Hover produces no net torque: the diagonal spin pairs cancel.
	InputVector hover;
	hover.setAll(m.hoverThrust());
	const matrix::Vector3f tau_hover = m.torque(hover);

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(tau_hover(i), 0.f, 1e-6f) << "axis " << i;
	}
}

TEST(PgfModel, QuaternionNormIsConserved)
{
	Flow flow = makeFlow();

	for (int c = 0; c < kNumCases; c++) {
		const StateVector x = toState(kCases[c].x);
		const StateVector f = flow.model().f(x, toInput(kCases[c].u));

		float d = 0.f;

		for (int i = 0; i < 4; i++) {
			d += x(IDX_Q + i) * f(IDX_Q + i);
		}

		EXPECT_NEAR(2.f * d, 0.f, 1e-5f) << "case " << c;
	}
}

TEST(PgfModel, DynamicsMatchPython)
{
	Flow flow = makeFlow();

	for (int c = 0; c < kNumCases; c++) {
		const Case &tc = kCases[c];
		// The expected values were produced after the sign alignment, so mirror
		// it here: alignment lives in Flow, and f is evaluated on the result.
		StateVector x = toState(tc.x);
		const StateVector x_hat = toState(tc.x0)
					  + (toState(tc.x1) - toState(tc.x0)) * tc.frac;
		float dot = 0.f;

		for (int i = 0; i < 4; i++) {
			dot += x(IDX_Q + i) * x_hat(IDX_Q + i);
		}

		if (dot < 0.f) {
			for (int i = 0; i < 4; i++) {
				x(IDX_Q + i) = -x(IDX_Q + i);
			}
		}

		const StateVector f = flow.model().f(x, toInput(tc.u));

		for (int i = 0; i < NX; i++) {
			expectClose(f(i), tc.f[i], 1e-4f, 1e-5f, "f", i);
		}

		const InputJacobian fu = flow.model().fu(x);

		for (int i = 0; i < NX; i++) {
			for (int j = 0; j < NU; j++) {
				expectClose(fu(i, j), tc.f_u[i * NU + j], 1e-4f, 1e-6f, "f_u", i * NU + j);
			}
		}

		const matrix::Vector<float, 4> row = flow.model().wfuxQuatRow(x, toState(tc.w));

		for (int i = 0; i < 4; i++) {
			expectClose(row(i), tc.wfux_row[i], 1e-4f, 1e-5f, "wfux_row", i);
		}
	}
}

TEST(PgfFlow, OptimalityResidualMatchesPython)
{
	Flow flow = makeFlow();

	for (int c = 0; c < kNumCases; c++) {
		const Case &tc = kCases[c];
		flow.reset(toInput(tc.u));
		const InputVector q_u =
			flow.optimalityResidual(toPayload(tc), toState(tc.x), tc.frac);

		for (int i = 0; i < NU; i++) {
			expectClose(q_u(i), tc.q_u[i], 2e-3f, 1e-4f, "q_u", i);
		}
	}
}

TEST(PgfFlow, SemiImplicitStepMatchesPython)
{
	Flow flow = makeFlow();

	for (int c = 0; c < kNumCases; c++) {
		const Case &tc = kCases[c];
		flow.reset(toInput(tc.u));
		FlowDiagnostics diag;
		const InputVector u_next =
			flow.update(toPayload(tc), toState(tc.x), tc.frac, kDtPhys, &diag);

		EXPECT_FALSE(diag.solve_failed) << "case " << c;

		for (int i = 0; i < NU; i++) {
			expectClose(u_next(i), tc.u_next[i], 2e-3f, 1e-4f, "u_next", i);
		}
	}
}

TEST(PgfFlow, QuaternionSignAlignmentIsInvariant)
{
	// Negating the measured quaternion is the same attitude, so the flow must
	// produce the same input. Without alignment, S0 (x - xhat) sees a step of
	// twice the quaternion and the control diverges.
	Flow flow = makeFlow();

	for (int c = 0; c < kNumCases; c++) {
		const Case &tc = kCases[c];
		const Payload p = toPayload(tc);

		flow.reset(toInput(tc.u));
		const InputVector a = flow.update(p, toState(tc.x), tc.frac, kDtPhys);

		StateVector flipped = toState(tc.x);

		for (int i = 0; i < 4; i++) {
			flipped(IDX_Q + i) = -flipped(IDX_Q + i);
		}

		flow.reset(toInput(tc.u));
		const InputVector b = flow.update(p, flipped, tc.frac, kDtPhys);

		for (int i = 0; i < NU; i++) {
			EXPECT_NEAR(a(i), b(i), 1e-4f) << "case " << c << " element " << i;
		}
	}
}

TEST(PgfFlow, FrozenExpansionDropsTheTauTerm)
{
	// frac == 1 means a payload is late, so the expansion no longer drifts and
	// Q_utau must vanish. Build a second payload whose endpoints both equal the
	// original node 1: evaluated at frac == 0 it has the same xhat, shat and
	// unom, but zero drift by construction. The two must agree.
	Flow flow = makeFlow();

	for (int c = 0; c < kNumCases; c++) {
		const Case &tc = kCases[c];

		const Payload p = toPayload(tc);
		flow.reset(toInput(tc.u));
		const InputVector at_one = flow.update(p, toState(tc.x), 1.f, kDtPhys);

		Payload frozen = p;
		frozen.x0 = p.x1;
		frozen.s0 = p.s1;
		frozen.u_nom0 = p.u_nom1;
		flow.reset(toInput(tc.u));
		const InputVector no_drift = flow.update(frozen, toState(tc.x), 0.f, kDtPhys);

		for (int i = 0; i < NU; i++) {
			EXPECT_NEAR(at_one(i), no_drift(i), 1e-4f) << "case " << c << " element " << i;
		}
	}
}

TEST(PgfFlow, InputStaysWithinThrustLimits)
{
	Flow flow = makeFlow();

	// Drive the flow hard from a badly wrong input and check it never leaves
	// the physically achievable thrust band.
	const Case &tc = kCases[0];
	const Payload p = toPayload(tc);
	InputVector far;
	far.setAll(kUMax);
	flow.reset(far);

	for (int k = 0; k < 400; k++) {
		const InputVector u = flow.update(p, toState(tc.x), 1.f, kDtPhys);

		for (int i = 0; i < NU; i++) {
			ASSERT_GE(u(i), kUMin) << "tick " << k << " element " << i;
			ASSERT_LE(u(i), kUMax) << "tick " << k << " element " << i;
			ASSERT_TRUE(PX4_ISFINITE(u(i))) << "tick " << k << " element " << i;
		}
	}
}

TEST(PgfModel, ThrustMapInvertsTheFittedCurve)
{
	// Fit for the iFlight Xing 2306 2450kv on 4S with a HQ v1s 5x4.3x3PC, from
	// quadrotor-flight-stack/scripts/fit_thrust_map.py.
	const float a = 9.156609f, b = 4.372645f, c = 0.222751f;
	const float t_min = c;
	const float t_max = a + b + c;

	// Round trip: eta -> thrust -> eta.
	for (int i = 0; i <= 20; i++) {
		const float eta = i / 20.f;
		const float thrust = a * eta * eta + b * eta + c;
		EXPECT_NEAR(thrustToNormalized(thrust, a, b, c), eta, 1e-4f) << "eta " << eta;
	}

	// Endpoints and saturation.
	EXPECT_NEAR(thrustToNormalized(t_min, a, b, c), 0.f, 1e-5f);
	EXPECT_NEAR(thrustToNormalized(t_max, a, b, c), 1.f, 1e-5f);
	EXPECT_FLOAT_EQ(thrustToNormalized(t_max * 2.f, a, b, c), 1.f);
	EXPECT_FLOAT_EQ(thrustToNormalized(-5.f, a, b, c), 0.f);

	// A misconfigured vehicle must idle, not command something arbitrary.
	EXPECT_FLOAT_EQ(thrustToNormalized(5.f, 0.f, 0.f, 0.f), 0.f);
	EXPECT_FLOAT_EQ(thrustToNormalized(NAN, a, b, c), 0.f);

	// A linear curve is a real configuration, not a degenerate one: the SIH
	// simulator's rotors are T = T_MAX * eta.
	EXPECT_NEAR(thrustToNormalized(2.5f, 0.f, 5.f, 0.f), 0.5f, 1e-6f);
	EXPECT_NEAR(thrustToNormalized(0.f, 0.f, 5.f, 0.f), 0.f, 1e-6f);
	EXPECT_NEAR(thrustToNormalized(5.f, 0.f, 5.f, 0.f), 1.f, 1e-6f);
	EXPECT_FLOAT_EQ(thrustToNormalized(99.f, 0.f, 5.f, 0.f), 1.f);

	// Monotonic, or the inverse would be ambiguous.
	float prev = -1.f;

	for (int i = 0; i <= 50; i++) {
		const float eta = thrustToNormalized(t_min + (t_max - t_min) * i / 50.f, a, b, c);
		EXPECT_GT(eta, prev) << "sample " << i;
		prev = eta;
	}

	// Hover for a 0.75 kg vehicle should land at a plausible racing-quad throttle.
	const float hover = 0.75f * 9.80665f / 4.f;
	const float eta_hover = thrustToNormalized(hover, a, b, c);
	EXPECT_GT(eta_hover, 0.1f);
	EXPECT_LT(eta_hover, 0.5f);
}

TEST(PgfFlow, RejectsInvalidArguments)
{
	Flow flow = makeFlow();
	const Case &tc = kCases[0];
	Payload p = toPayload(tc);
	const InputVector start = toInput(tc.u);

	// A zero dt or a payload with no interval must hold the input, not divide by zero.
	flow.reset(start);
	FlowDiagnostics diag;
	EXPECT_EQ(flow.update(p, toState(tc.x), 0.f, 0.f, &diag), start);
	EXPECT_TRUE(diag.solve_failed);

	p.dtau = 0.f;
	flow.reset(start);
	EXPECT_EQ(flow.update(p, toState(tc.x), 0.f, kDtPhys, &diag), start);
	EXPECT_TRUE(diag.solve_failed);
}
