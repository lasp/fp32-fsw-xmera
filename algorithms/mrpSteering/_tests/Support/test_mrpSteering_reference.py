"""mrpSteering reference values (port of MRP-Steering-Validation.nb).

Derives the expected outputs of the MRP steering law and the omega-servo control torque:

- Steering law: the arctan-limited body rate command omega_B*/R and its derivative.
- Omega-servo control: the torque terms Lr1 ... Lr5 with integration step h = 0 and h = 0.5.

Run ``python test_mrpSteering_reference.py`` to print the walkthrough. Run with pytest to check the
values against the notebook outputs.

The notebook "Check" section also held hand-typed Lr3/Lr4/Lr5 reference values. These values were
stale: they were about 0.33 per axis away from the values that the notebook itself computed. This port
does not keep them, and it checks against the computed values.
"""

import numpy as np

# --- Setup ------------------------------------------------------------------------------------------
K1 = 0.15
K3 = 1.0
KI = 0.01
P = 150.0
OMEGA_MAX = np.radians(1.5)  # [rad/s]
L = np.zeros(3)  # known external torque [N m]
OMEGA_RW = np.array([10.0, 25.0, 50.0, 100.0])  # wheel speeds [rad/s]
JS = 0.1  # wheel spin inertia [kg m^2]
# Wheel spin axes, one row for each wheel (4x3).
GS = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0] / np.sqrt(3.0)])
INERTIA = np.diag([1000.0, 800.0, 800.0])  # [kg m^2]

SIGMA_BR = np.array([0.3, -0.5, 0.7])
OMEGA_BR_B = np.array([0.010, -0.020, 0.015])  # notebook setup only; the control law does not use it
OMEGA_RN_B = np.array([-0.02, -0.01, 0.005])
DOMEGA_RN_B = np.array([0.0002, 0.0003, 0.0001])
SIGMA_BN = np.array([0.25, -0.45, 0.75])  # notebook setup only; the control law does not use it
OMEGA_BN_B = np.array([-0.015, -0.012, 0.005])


# --- Formulas ---------------------------------------------------------------------------------------
def tilde(v):
    """Skew-symmetric cross-product matrix [v x]."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def bmat_mrp(sigma):
    """MRP kinematic matrix B(sigma). This is the same as BmatMRP in the RigidBodyKinematics package."""
    return (1.0 - sigma @ sigma) * np.eye(3) + 2.0 * tilde(sigma) + 2.0 * np.outer(sigma, sigma)


def steering_rate(sigma):
    """omega_B*/R = -(2 omega_max / pi) * atan((K1 sigma + K3 sigma^3) * pi / (2 omega_max))."""
    return -np.arctan((K1 * sigma + K3 * sigma**3) * np.pi / (2.0 * OMEGA_MAX)) * 2.0 * OMEGA_MAX / np.pi


def steering_rate_derivative(sigma, omega_bstar_r):
    """d(omega_B*/R)/dt, with d(sigma)/dt = 1/4 B(sigma) omega_B*/R."""
    x = K1 * sigma + K3 * sigma**3
    scale = (K1 + 3.0 * K3 * sigma**2) / (1.0 + x**2 * (np.pi / (2.0 * OMEGA_MAX)) ** 2)
    return -scale * 0.25 * (bmat_mrp(sigma) @ omega_bstar_r)


def servo_control(h):
    """Return the omega-servo intermediate values for integration step h, as a dict."""
    omega_bstar_r = steering_rate(SIGMA_BR)
    domega_bstar_r = steering_rate_derivative(SIGMA_BR, omega_bstar_r)
    omega_bstar_n = omega_bstar_r + OMEGA_RN_B
    delta_omega = OMEGA_BN_B - omega_bstar_n
    z = delta_omega * h
    lr1 = P * delta_omega
    lr2 = lr1 + KI * z
    h_total = INERTIA @ OMEGA_BN_B + JS * GS.T @ (OMEGA_RW + GS @ OMEGA_BN_B)
    lr3 = lr2 - np.cross(omega_bstar_n, h_total)
    lr4 = lr3 - INERTIA @ (domega_bstar_r + DOMEGA_RN_B - np.cross(OMEGA_BN_B, OMEGA_RN_B))
    lr5 = lr4 + L  # no sign flip here (mrpFeedback uses -(Lr4 + L))
    return {
        "omegaBstarRvecB": omega_bstar_r,
        "domegaBstarRvecB": domega_bstar_r,
        "omegaBstarNvecB": omega_bstar_n,
        "deltaOmegavecB": delta_omega,
        "zvecB": z,
        "Lr1": lr1,
        "Lr2": lr2,
        "Lr3": lr3,
        "Lr4": lr4,
        "Lr5": lr5,
    }


# --- Notebook outputs -------------------------------------------------------------------------------
GS4_EXPECTED = [0.5773502691896258, 0.5773502691896258, 0.5773502691896258]

_COMMON = {
    "omegaBstarRvecB": [-0.02238868910536782, 0.02479425158177426, -0.02556018490859001],
    "domegaBstarRvecB": [0.0001877666495969893, -0.000039123358331975947, 0.000035636948864815126],
    "omegaBstarNvecB": [-0.04238868910536782, 0.01479425158177426, -0.020560184908590007],
    "deltaOmegavecB": [0.02738868910536782, -0.02679425158177426, 0.02556018490859001],
    "Lr1": [4.108303365805173, -4.0191377372661385, 3.834027736288501],
}

EXPECTED = {
    0.0: {
        **_COMMON,
        "zvecB": [0.0, 0.0, 0.0],
        "Lr2": [4.108303365805173, -4.0191377372661385, 3.834027736288501],
        "Lr3": [3.9170566818554593, -4.814541482727046, 3.655979391522506],
        "Lr4": [3.51929003225847, -5.043242796061465, 3.4754698324306537],
        "Lr5": [3.51929003225847, -5.043242796061465, 3.4754698324306537],
    },
    0.5: {
        **_COMMON,
        "zvecB": [0.01369434455268391, -0.01339712579088713, 0.012780092454295004],
        "Lr2": [4.1084403092507, -4.019271708524047, 3.8341555372130443],
        "Lr3": [3.917193625300986, -4.814675453984955, 3.656107192447049],
        "Lr4": [3.5194269757039964, -5.043376767319374, 3.475597633355197],
        "Lr5": [3.5194269757039964, -5.043376767319374, 3.475597633355197],
    },
}


# --- Checks -----------------------------------------------------------------------------------------
def test_wheel_axis():
    np.testing.assert_allclose(GS[3], GS4_EXPECTED, rtol=1e-15)


def test_servo_control():
    for h, expected in EXPECTED.items():
        actual = servo_control(h)
        for name, value in expected.items():
            np.testing.assert_allclose(actual[name], value, rtol=1e-12, atol=1e-15, err_msg=f"h={h}, {name}")


if __name__ == "__main__":
    np.set_printoptions(precision=17)
    for h in EXPECTED:
        print(f"Omega-servo control with h = {h}:")
        for name, value in servo_control(h).items():
            print(f"  {name:17s} = {value}")
