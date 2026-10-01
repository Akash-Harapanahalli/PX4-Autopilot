# Pontryagin Gradient Flow controller status. Logged for tuning, not bridged to DDS.

uint64 timestamp	# [us] time since system start (microseconds)

uint32 seq		# seq of the value expansion currently driving the flow

float32[4] u		# [N] current per-rotor thrust held by the flow
float32[4] u_norm	# normalized [0, 1] actuator values published

float32 qu_norm		# ||Q_u||, the residual the flow drives to zero
float32 tightness	# ||Q_u|| / (2 * (R + w_prox) * u_max), target 0.1 .. 0.4
float32 frac		# interpolation fraction within the MPC interval
float32 payload_age_ms	# [ms] round trip age of the estimate the MPC solved on;
			# when this exceeds dtau, frac pins at 1 and the
			# expansion stops interpolating

uint8 flags		# see FLAG_* below

uint8 FLAG_FLOW_ACTIVE = 1	# flow integrating and driving the motors
uint8 FLAG_PAYLOAD_STALE = 2	# no fresh value expansion, holding last input
uint8 FLAG_RESET_GATE = 4	# estimator reset, flow gated to zero order hold
uint8 FLAG_ZOH_MODE = 8		# PGF_FLOW_EN disabled, replaying u0 directly
uint8 FLAG_SEQ_MISMATCH = 16	# gradient and Hessian messages out of sync
