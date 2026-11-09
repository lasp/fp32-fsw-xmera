"""mrpFeedback reference values (port of MRP-Feedback-Validation.nb).

Derives the expected MRP feedback control torque Lr for four cases: with and without reaction wheels
(numRW = 4 or 0), and with the integral term on or off (Ki = 0.01 or Ki = -1).

Each case gives a five-row sequence (``true_lr``). Rows 1, 2 and 4 are a call with integration step
h = 0. Rows 3 and 5 are a call with h = 0.5.

Run ``python test_mrpFeedback_reference.py`` to print the walkthrough. Run with pytest to check the
values against the notebook outputs.
"""

import numpy as np

# --- Setup ------------------------------------------------------------------------------------------
K = 0.15
KI = 0.01
P = 150.0
# The notebook computes this limit once from the global Ki. Calls with a different Ki keep it.
INTEGRAL_LIMIT = 2.0 / KI * 0.1
INERTIA = np.diag([1000.0, 800.0, 800.0])  # [kg m^2]

SIGMA_BR = np.array([0.3, -0.5, 0.7])
OMEGA_BR_B = np.array([0.01, -0.02, 0.015])
OMEGA_RN_B = np.array([-0.02, -0.01, 0.005])
DOMEGA_RN_B = np.array([0.0002, 0.0003, 0.0001])
OMEGA_BN_B = np.array([-0.01, -0.03, 0.02])
ZERO_VECTOR = np.zeros(3)

# Wheel spin axes, one column for each wheel (3x4). The notebook uses 0.577350269190, not 1/sqrt(3).
GS = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.577350269190] * 3]).T
JS = 0.1  # wheel spin inertia [kg m^2]
OMEGA_RW = np.array([10.0, 25.0, 50.0, 100.0])  # wheel speeds [rad/s]
HS = JS * (GS.T @ OMEGA_BN_B + OMEGA_RW)  # wheel angular momentum about the spin axes [N m s]


# --- Formulas ---------------------------------------------------------------------------------------
def int_limit(x):
    """One-sided limit Min(x, integralLimit), as in the notebook. It does not limit negative values."""
    return np.minimum(x, INTEGRAL_LIMIT)


def mrp_feedback(num_rw, ki, ext_torque, delta_omega0, h):
    """Control torque Lr for one call. The integral term is on only if ki > 0."""
    if ki > 0:
        int_sigma = int_limit(K * SIGMA_BR * h)
        z = int_sigma + INERTIA @ (OMEGA_BR_B - delta_omega0)
    else:
        z = np.zeros(3)
    lr0 = K * SIGMA_BR
    lr1 = lr0 + P * OMEGA_BR_B
    lr2 = lr1 + P * ki * z
    h_total = INERTIA @ OMEGA_BN_B
    if num_rw > 0:
        h_total = h_total + GS @ HS
    lr3 = lr2 - np.cross(OMEGA_RN_B + ki * z, h_total)
    lr4 = lr3 - INERTIA @ (DOMEGA_RN_B - np.cross(OMEGA_BN_B, OMEGA_RN_B))
    return -(lr4 + ext_torque)


def true_lr(num_rw, ki, ext_torque, delta_omega0):
    """Five-row Lr sequence: h = 0, 0, 0.5, 0, 0.5."""
    step1 = mrp_feedback(num_rw, ki, ext_torque, delta_omega0, 0.0)
    step2 = mrp_feedback(num_rw, ki, ext_torque, delta_omega0, 0.5)
    return np.array([step1, step1, step2, step1, step2])


# --- Notebook outputs -------------------------------------------------------------------------------
def _sequence(step1, step2):
    return [step1, step1, step2, step1, step2]


# (numRW, Ki) -> expected true_lr(numRW, Ki, zeroVector, zeroVector)
EXPECTED = {
    (4, KI): _sequence(
        [-18.980451627443838, 25.0494926211355, -21.682200993691666],
        [-19.015983854866544, 25.09802349694307, -21.765700842076527],
    ),
    (0, KI): _sequence(
        [-16.115000000000002, 25.064999999999998, -23.495],
        [-16.14215, 25.1124, -23.5829],
    ),
    (0, -1.0): _sequence(
        [-1.435, 3.865, -1.495],
        [-1.435, 3.865, -1.495],
    ),
    (4, -1.0): _sequence(
        [-1.5840975403785, 4.114355900630834, -1.5926783602523331],
        [-1.5840975403785, 4.114355900630834, -1.5926783602523331],
    ),
}


# --- Checks -----------------------------------------------------------------------------------------
def test_true_lr():
    for (num_rw, ki), expected in EXPECTED.items():
        np.testing.assert_allclose(
            true_lr(num_rw, ki, ZERO_VECTOR, ZERO_VECTOR),
            expected,
            rtol=1e-12,
            atol=1e-15,
            err_msg=f"numRW={num_rw}, Ki={ki}",
        )


if __name__ == "__main__":
    np.set_printoptions(precision=12)
    for num_rw, ki in EXPECTED:
        print(f"trueLr[numRW={num_rw}, Ki={ki}]:")
        print(true_lr(num_rw, ki, ZERO_VECTOR, ZERO_VECTOR))
