"""Magnitude of covarImageN vs covarAttitudeN in the cobConverter COM heading covariance.

Checks covarImageN + covarAttitudeN against the module's covar_N, then reports one-at-a-time sweeps
around a nominal case and a Monte Carlo over MC_RANGES.

Run:  python cobConverterCovarianceBudget.py
"""
import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_cobConverter as tc  # noqa: E402  (reuses the covariance helpers the pytest validates)
from xmera.architecture import messaging  # noqa: E402
from xmera.utilities import RigidBodyKinematics as rbk  # noqa: E402
from xmera.utilities import SimulationBaseClass, macros  # noqa: E402

RAD2ARCSEC = 180.0 / np.pi * 3600.0
ATTITUDE_COVARIANCE = 1e-5 * np.eye(3)  # [-] config.attitudeCovariance (error-MRP covariance, body frame)
MC_SAMPLES = 10000

NOMINAL = dict(
    pixels=100.0,  # [-] detected pixel count
    distance=5000e3,  # [m] spacecraft range
    radius=25e3,  # [m] object radius
    radiusUncertaintyRatio=0.32,  # [-] sigma_R / R
    positionSigma=np.sqrt(50e3),  # [m] per-axis filter position 1-sigma
    phaseAngleDeg=45.0,  # [deg] Sun phase angle
    sunAzimuthDeg=30.0,  # [deg] Sun azimuth about the line of sight (sets phi)
    cobFraction=0.5,  # [-] COB position along the image diagonal (0.5 = center, 1 = corner)
    resolution=512.0,  # [px] square sensor
    fovDeg=20.0,  # [deg] square field of view
)

SWEEPS = dict(
    pixels=np.logspace(0, 5, 11),
    distance=np.logspace(5, 8, 7),
    radiusUncertaintyRatio=np.linspace(0.0, 0.5, 6),
    positionSigma=np.logspace(0, 5, 6),
    phaseAngleDeg=np.linspace(0.0, 150.0, 7),
    cobFraction=np.linspace(0.5, 1.0, 6),
    resolution=np.array([128, 256, 512, 1024, 2048, 4096], dtype=float),
)

# (low, high, log-uniform?)
MC_RANGES = dict(
    pixels=(1.0, 1e5, True),
    distance=(1e5, 1e8, True),
    radiusUncertaintyRatio=(0.0, 0.5, False),
    positionSigma=(1.0, 1e5, True),
    phaseAngleDeg=(0.0, 150.0, False),
    sunAzimuthDeg=(0.0, 360.0, False),
    cobFraction=(0.0, 1.0, False),
    resolution=(128.0, 4096.0, True),
)

RATIO = "tr(covarAttitudeN)/tr(covarImageN)"


def scenario(p):
    """Test-style geometry: camera boresight on the target, Sun at the requested phase angle and azimuth."""
    r_N = np.array([-p["distance"], -300e3, 0.0])
    v_N = np.array([8e3, 0.0, 0.0])
    h1 = r_N / np.linalg.norm(r_N)
    h3 = np.cross(h1, v_N) / np.linalg.norm(np.cross(h1, v_N))
    h2 = np.cross(h3, h1)
    alpha = np.deg2rad(p["phaseAngleDeg"])
    psi = np.deg2rad(p["sunAzimuthDeg"])
    return SimpleNamespace(
        r_N=r_N,
        v_N=v_N,
        dcm_BN=np.array([h1, h2, h3]),
        dcm_CB=np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]]),
        sun_N=np.cos(alpha) * h1 + np.sin(alpha) * (np.cos(psi) * h2 + np.sin(psi) * h3),
        cob=np.full(2, p["resolution"] * p["cobFraction"]),
        camera=SimpleNamespace(fieldOfView=[np.deg2rad(p["fovDeg"])] * 2, resolution=[p["resolution"]] * 2),
    )


def components(p):
    """(covarImageN, covarAttitudeN), mirroring CobConverterAlgorithm::updateState."""
    s = scenario(p)
    dcm_NC = (s.dcm_CB @ s.dcm_BN).T
    K = tc.compute_camera_calibration_matrix(s.camera)
    alpha = np.arccos(np.clip(s.r_N @ s.sun_N / np.linalg.norm(s.r_N), -1.0, 1.0))
    sun_C = dcm_NC.T @ s.sun_N
    phi = np.arctan2(sun_C[1], sun_C[0])
    tanBeta = p["radius"] * tc.phase_angle_correction(alpha) / np.linalg.norm(s.r_N)
    com = [s.cob[0] - tanBeta * K[0, 0] * np.cos(phi), s.cob[1] - tanBeta * K[1, 1] * np.sin(phi)]
    rhat_COM_C, _ = tc.mapState(com, s.camera)

    covarImage_C = tc.mapComCovar(p["pixels"], s.camera, rhat_COM_C, s.r_N, p["radius"], alpha, s.sun_N,
                                  p["radiusUncertaintyRatio"] * p["radius"], phi, p["positionSigma"] ** 2 * np.eye(3),
                                  tanBeta)
    skew_B = tc.skew(s.dcm_CB.T @ rhat_COM_C)
    covarAttitudeN = 16.0 * s.dcm_BN.T @ skew_B @ ATTITUDE_COVARIANCE @ skew_B.T @ s.dcm_BN
    return dcm_NC @ covarImage_C @ dcm_NC.T, covarAttitudeN


def module_covar_N(p):
    """covar_N published by the cobConverter module for scenario p."""
    s = scenario(p)
    sim = SimulationBaseClass.SimBaseClass()
    rate = macros.sec2nano(0.5)
    sim.CreateNewProcess("proc").addTask(sim.CreateNewTask("task", rate))

    module = tc.cobConverter.CobConverter()
    module.radius = p["radius"]
    module.radiusUncertainty = p["radiusUncertaintyRatio"] * p["radius"]
    module.attitudeCovariance = ATTITUDE_COVARIANCE
    module.fieldOfViewX = module.fieldOfViewY = np.deg2rad(p["fovDeg"])
    module.resolutionX = module.resolutionY = p["resolution"]
    module.bodyToCameraMrp = rbk.C2MRP(s.dcm_CB)
    sim.AddModelToTask("task", module, module)

    cob = messaging.OpNavCOBMsgF32Payload()
    cob.centerOfBrightness = list(s.cob)
    cob.pixelsFound = int(p["pixels"])
    cob.valid = True
    nav = messaging.FilterMsgF32Payload()
    nav.numberOfStates = 6
    nav.state = np.concatenate([s.r_N, s.v_N])
    nav.covar = np.diag([p["positionSigma"] ** 2] * 3 + [0.01] * 3).flatten()
    att = messaging.NavAttMsgF32Payload()
    att.sigma_BN = rbk.C2MRP(s.dcm_BN)
    att.vehSunPntBdy = s.dcm_BN @ s.sun_N
    msgs = [messaging.OpNavCOBMsgF32().write(cob), messaging.FilterMsgF32().write(nav),
            messaging.NavAttMsgF32().write(att)]
    module.opnavCOBInMsg.subscribeTo(msgs[0])
    module.opnavFilterInMsg.subscribeTo(msgs[1])
    module.navAttInMsg.subscribeTo(msgs[2])

    log = module.opnavUnitVecOutMsg.recorder()
    sim.AddModelToTask("task", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(rate)
    sim.ExecuteSimulation()
    return np.array(log.covar_N[0]).reshape(3, 3)


def metrics(p):
    """1-sigma [arcsec] of each term along its worst direction, and the trace ratio."""
    covarImageN, covarAttitudeN = components(p)
    sigma = lambda P: np.sqrt(max(np.linalg.eigvalsh(P)[-1], 0.0)) * RAD2ARCSEC  # noqa: E731
    return sigma(covarImageN), sigma(covarAttitudeN), np.trace(covarAttitudeN) / np.trace(covarImageN)


def consistency_check():
    cases = [NOMINAL,
             {**NOMINAL, "cobFraction": 0.95, "phaseAngleDeg": 120.0},
             {**NOMINAL, "pixels": 5000.0, "distance": 1e6}]
    for p in cases:
        expected = sum(components(p))
        # fp32 module: relative error on large entries, ~eps * max|P| on the near-zero ones
        np.testing.assert_allclose(module_covar_N(p), expected, rtol=1e-4, atol=1e-5 * np.abs(expected).max(),
                                   err_msg="covarImageN + covarAttitudeN != module covar_N")
    print(f"Consistency check passed: covarImageN + covarAttitudeN == module covar_N ({len(cases)} cases)\n")


def sweeps():
    print("Nominal: " + ", ".join(f"{k}={v:.4g}" for k, v in NOMINAL.items()))
    print("sigImg/sigAtt: 1-sigma [arcsec] of covarImageN/covarAttitudeN along the worst direction\n")
    for name, values in SWEEPS.items():
        print(f"{name[:12]:>12}{'sigImg[arcsec]':>15}{'sigAtt[arcsec]':>15}{RATIO:>38}")
        for v in values:
            sigImg, sigAtt, ratio = metrics({**NOMINAL, name: v})
            print(f"{v:>12.4g}{sigImg:>15.3g}{sigAtt:>15.3g}{ratio:>38.3g}")
        print()


def monte_carlo(seed=0):
    rng = np.random.default_rng(seed)
    ratios = []
    for _ in range(MC_SAMPLES):
        p = dict(NOMINAL)
        for name, (low, high, log) in MC_RANGES.items():
            p[name] = 10 ** rng.uniform(np.log10(low), np.log10(high)) if log else rng.uniform(low, high)
        p["pixels"] = float(np.round(p["pixels"]))
        ratios.append(metrics(p)[2])
    ratios = np.array(ratios)
    p5, p50, p95 = np.percentile(ratios, [5, 50, 95])
    print(f"Monte Carlo ({MC_SAMPLES} samples over MC_RANGES)")
    print(f"  {RATIO}  p5={p5:.3g}  median={p50:.3g}  p95={p95:.3g}  mean={ratios.mean():.3g}")
    print(f"  fraction with tr(covarAttitudeN) > tr(covarImageN): {np.mean(ratios > 1):.2f}")


if __name__ == "__main__":
    consistency_check()
    sweeps()
    monte_carlo()
