/**
 * Auto request offboard mode and arm
 *
 * Simulation only. When set, pgf_sim_bridge requests offboard mode and arms once
 * the companion process is alive and the estimator has had time to settle.
 *
 * @boolean
 * @group PGF
 */
PARAM_DEFINE_INT32(PGF_SIM_ENGAGE, 0);

/**
 * Companion liveness timeout
 *
 * The offboard heartbeat is withheld when no value expansion has arrived within
 * this window, so PX4 runs its own failsafe.
 *
 * @unit ms
 * @min 20.0
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_SIM_TOUT, 200.0f);
